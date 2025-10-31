# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

# ---------- small helpers ----------
def _yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class _ROSBridge(Node):
    """Singleton-ish ROS glue used by RobotAPI instances."""
    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get(cls, ns: str, odom_topic: str, cmd_vel_topic: str, nav2_action: str):
        with cls._lock:
            if cls._instance is None:
                # Ensure rclpy is started once
                if not rclpy.ok():
                    rclpy.init(args=None)

                cls._instance = _ROSBridge(ns, odom_topic, cmd_vel_topic, nav2_action)
                # spin in bg thread
                cls._instance._spin_thread = threading.Thread(
                    target=rclpy.spin, args=(cls._instance,), daemon=True
                )
                cls._instance._spin_thread.start()
            return cls._instance

    def __init__(self, ns: str, odom_topic: str, cmd_vel_topic: str, nav2_action: str):
        super().__init__(f"robot_api_bridge_{ns.strip('/').replace('/', '_') or 'root'}", namespace=ns or "")
        self._latest_xyyaw: Optional[Tuple[float, float, float]] = None
        self._goal_handle = None
        self._goal_done = True   # idle at start
        self._nav2_status: Optional[int] = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._nav_client = ActionClient(self, NavigateToPose, nav2_action)

        self.get_logger().info(
            f"[ROSBridge] odom={odom_topic}, cmd_vel={cmd_vel_topic}, nav2={nav2_action}"
        )

    # --------------- callbacks ---------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._latest_xyyaw = (float(p.x), float(p.y), _yaw_from_quat(q.x, q.y, q.z, q.w))

    # --------------- high-level ops ----------
    def have_pose(self) -> bool:
        return self._latest_xyyaw is not None

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        return self._latest_xyyaw

    def publish_zero(self):
        self._cmd_pub.publish(Twist())

    def action_ready(self, timeout_sec: float = 5.0) -> bool:
        if self._nav_client.server_is_ready():
            return True
        return self._nav_client.wait_for_server(timeout_sec=timeout_sec)

    async def send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        from builtin_interfaces.msg import Time
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"  # adapter handles frame alignment
        now = self.get_clock().now().to_msg()
        goal.pose.header.stamp = Time(sec=now.sec, nanosec=now.nanosec)
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        cz = math.cos(yaw * 0.5); sz = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.z = sz
        goal.pose.pose.orientation.w = cz

        self._goal_done = False
        send_future = self._nav_client.send_goal_async(goal)
        goal_handle = await send_future
        if not goal_handle.accepted:
            self._goal_done = True
            self._nav2_status = None
            self.get_logger().warn("NavigateToPose goal rejected")
            return False

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result = await result_future
        self._goal_handle = None
        self._nav2_status = getattr(result, "status", None)
        self._goal_done = True
        # 4 == SUCCEEDED per action_msgs/GoalStatus
        return self._nav2_status == 4

    async def cancel_goal(self):
        if self._goal_handle is not None:
            try:
                await self._goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Cancel failed: {e}")
        self.publish_zero()
        self._goal_done = True

# ============================
# Your original RobotAPI class
# ============================
class RobotAPI:
    """
    Keeps your original shape, but implements ROS2/Nav2 behavior.
    YAML fields recognized under config_yaml:
      prefix (ignored for ROS)
      user, password (ignored)
      timeout (optional)
      debug (optional)
      namespace: "" or "/tb4_1"
      odom_topic: "/odom" or "/tb4_1/odom"
      cmd_vel_topic: "/cmd_vel" or "/tb4_1/cmd_vel"
      nav2_action: "/navigate_to_pose" or "/tb4_1/navigate_to_pose"
      map_name: "dc_l2"  (optional hint)
    """

    def __init__(self, config_yaml):
        self.prefix = config_yaml.get('prefix', '')
        self.user = config_yaml.get('user', '')
        self.password = config_yaml.get('password', '')
        self.timeout = float(config_yaml.get('timeout', 5.0))
        self.debug = bool(config_yaml.get('debug', False))

        ns = config_yaml.get('namespace', '')
        odom_topic = config_yaml.get('odom_topic', '/odom')
        cmd_vel_topic = config_yaml.get('cmd_vel_topic', '/cmd_vel')
        nav2_action = config_yaml.get('nav2_action', '/navigate_to_pose')
        self._map_name_hint = config_yaml.get('map_name', None)

        self._bridge = _ROSBridge.get(ns, odom_topic, cmd_vel_topic, nav2_action)
        self._last_goal_ok = False

    # ---------------- public API ----------------

    def check_connection(self):
        """True if Nav2 action server is reachable and odom is flowing."""
        ok_nav = self._bridge.action_ready(timeout_sec=self.timeout)
        ok_odom = self._bridge.have_pose()
        return bool(ok_nav and ok_odom)

    def navigate(self, robot_name: str, pose, map_name: str, speed_limit=0.0):
        """
        pose: [x, y, theta] in robot map frame (adapter aligns frames for you)
        returns True if the robot accepted & later succeeded; False on immediate failure.
        """
        # fire-and-wait in a small async helper spun inside rclpy thread
        done_flag = {'ok': False, 'set': False}

        async def _do():
            if not await self._bridge._nav_client.wait_for_server(timeout_sec=self.timeout):
                self._bridge.get_logger().error("Nav2 server unavailable")
                done_flag['ok'] = False; done_flag['set'] = True; return
            ok = await self._bridge.send_nav_goal(float(pose[0]), float(pose[1]), float(pose[2]))
            done_flag['ok'] = ok; done_flag['set'] = True

        # Use create_task to run inside the existing node executor
        self._bridge.create_task(_do())
        # Busy-wait until set (we are inside non-async API)
        import time
        start = time.time()
        while not done_flag['set'] and (time.time() - start) < (self.timeout + 300.0):
            time.sleep(0.05)

        self._last_goal_ok = bool(done_flag['ok'])
        return self._last_goal_ok

    def start_activity(self, robot_name: str, activity: str, label: str):
        """Optional custom behaviors. Not needed for patrol; return True as NOP."""
        return True

    def stop(self, robot_name: str):
        """Cancel current Nav2 goal and publish zero Twist."""
        done_flag = {'set': False}

        async def _do():
            await self._bridge.cancel_goal()
            done_flag['set'] = True

        self._bridge.create_task(_do())
        import time
        start = time.time()
        while not done_flag['set'] and (time.time() - start) < self.timeout:
            time.sleep(0.05)
        return True

    def position(self, robot_name: str):
        """Return [x, y, theta]; None if we haven't received odom yet."""
        p = self._bridge.get_pose()
        if p is None:
            return None
        return [float(p[0]), float(p[1]), float(p[2])]

    def battery_soc(self, robot_name: str):
        """Return SOC in [0,1]; if unknown, assume healthy (1.0)."""
        return 1.0

    def map(self, robot_name: str):
        """Return current map/level name. Use hint if provided."""
        return self._map_name_hint

    def is_command_completed(self):
        """True when the latest navigate() finished (success or failure)."""
        return self._bridge._goal_done

    def get_data(self, robot_name: str):
        """(kept as in your original file)"""
        map_name = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map_name is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map_name, position, battery_soc)
        return None


class RobotUpdateData:
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
