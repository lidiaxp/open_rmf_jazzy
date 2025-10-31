#!/usr/bin/env python3

import sys
import argparse
import yaml
import time
import threading
import asyncio
import nudged

import rclpy
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.duration import Duration

import rmf_adapter
from rmf_adapter import Adapter, Transformation
import rmf_adapter.easy_full_control as rmf_easy

from .RobotClientAPI import RobotAPI


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------
def compute_transforms(level, coords, node=None):
    """Compute RMF↔Robot coordinate transformation using reference landmarks."""
    rmf_coords = coords['rmf']
    robot_coords = coords['robot']
    tf = nudged.estimate(rmf_coords, robot_coords)
    if node:
        mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
        node.get_logger().info(f"Transformation error estimate for {level}: {mse}")
    return Transformation(tf.get_rotation(), tf.get_scale(), tf.get_translation())


# ------------------------------------------------------------------------------
# RobotAdapter class 
# ------------------------------------------------------------------------------
class RobotAdapter:
    def __init__(self, name, configuration, node, api: RobotAPI, fleet_handle):
        self.name = name
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle

        self.execution = None
        self.update_handle = None

    # --------------------------------------------------------------------------
    # RMF callback bindings
    # --------------------------------------------------------------------------
    def make_callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(category, description, execution)
        )

    # --------------------------------------------------------------------------
    # Command callbacks
    # --------------------------------------------------------------------------
    def navigate(self, destination, execution):
        """Send navigation goal via RobotAPI (Nav2)."""
        self.execution = execution
        self.node.get_logger().info(
            f"[{self.name}] Navigating to {destination.position} on map {destination.map}"
        )
        self.api.navigate(
            self.name,
            destination.position,
            destination.map,
            destination.speed_limit
        )

    def stop(self, activity):
        """Stop current navigation."""
        if self.execution and self.execution.identifier.is_same(activity):
            self.node.get_logger().warn(f"[{self.name}] Stop requested")
            self.api.stop(self.name)
            self.execution = None

    def execute_action(self, category: str, description: dict, execution):
        """Handle custom RMF actions."""
        self.node.get_logger().info(f"[{self.name}] Executing custom action: {category}")
        self.execution = execution
        self.api.start_activity(self.name, category, description)
        execution.finished()

    # --------------------------------------------------------------------------
    # Robot state update
    # --------------------------------------------------------------------------
    def update(self, state):
        activity_identifier = None
        execution = self.execution
        if execution:
            if self.api.is_command_completed():
                execution.finished()
                self.execution = None
            else:
                activity_identifier = execution.identifier

        self.update_handle.update(state, activity_identifier)


# ------------------------------------------------------------------------------
# Parallel helper 
# ------------------------------------------------------------------------------
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)
    return run_in_parallel


# ------------------------------------------------------------------------------
# update_robot 
# ------------------------------------------------------------------------------
@parallel
def update_robot(robot: RobotAdapter):
    """Collect current data from RobotAPI and update RMF state."""
    data = robot.api.get_data(robot.name)
    if data is None:
        return

    state = rmf_easy.RobotState(
        data.map,
        data.position,
        data.battery_soc
    )

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name,
            state,
            robot.configuration,
            robot.make_callbacks()
        )
        return

    robot.update(state)


# ------------------------------------------------------------------------------
# Main adapter entry point
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Fleet adapter for TurtleBot4 using Nav2 through RobotClientAPI"
    )
    parser.add_argument("-c", "--config_file", type=str, required=True, help="Path to config.yaml")
    parser.add_argument("-n", "--nav_graph", type=str, required=True, help="Path to nav_graph file")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="")
    parser.add_argument("-sim", "--use_sim_time", action="store_true")
    args = parser.parse_args(args_without_ros[1:])

    print("Starting fleet adapter...")

    # Load config and nav graph
    config_path = args.config_file
    nav_graph_path = args.nav_graph
    fleet_config = rmf_easy.FleetConfiguration.from_config_files(config_path, nav_graph_path)
    assert fleet_config, f"Failed to parse config file [{config_path}]"

    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # Node & adapter
    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f"{fleet_name}_command_handle")
    adapter = Adapter.make(f"{fleet_name}_fleet_adapter")
    adapter.start()

    # Simulation time
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    # Transform references
    for level, coords in config_yaml["reference_coordinates"].items():
        tf = compute_transforms(level, coords, node)
        fleet_config.add_robot_coordinates_transformation(level, tf)

    fleet_handle = adapter.add_easy_fleet(fleet_config)

    # Initialize RobotAPI
    fleet_mgr_yaml = config_yaml["fleet_manager"]
    api = RobotAPI(fleet_mgr_yaml)

    # Build RobotAdapter instances
    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name, robot_config, node, api, fleet_handle
        )

    # Update frequency
    update_period = 1.0 / config_yaml["rmf_fleet"].get("robot_state_update_frequency", 10.0)

    def update_loop():
        asyncio.set_event_loop(asyncio.new_event_loop())
        while rclpy.ok():
            now = node.get_clock().now()
            update_jobs = [update_robot(r) for r in robots.values()]
            asyncio.get_event_loop().run_until_complete(asyncio.wait(update_jobs))

            next_wakeup = now + Duration(nanoseconds=update_period * 1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, daemon=True)
    update_thread.start()

    # Executor for ROS node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)
    rclpy_executor.spin()

    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
