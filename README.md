# 🏗️ RMF Building Map Creation & Simulation Guide

This guide explains how to create a building map in **Traffic Editor**, export it for RMF, and set up the environment for simulation in **Gazebo**.

---

## 📦 1. Environment Setup

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 🗺️ 2. Create the Building Map

1. **Open Traffic Editor** with your building YAML file:
   ```bash
   traffic-editor building.yaml
   ```
2. **Right panel**:  
   - `Hierarchy → Name → Add…` → give your building a name.
3. **Add your PNG image** as the floor plan.
4. **Measurements**:  
   - On the menu, select the **pink line (T)** to add measurements of the environment.
5. **Walls**:  
   - Use the **blue line (W)** to set the walls.  
   - Set wall property:
     ```yaml
     texture_name: beige
     ```
6. **Floor Polygon**:  
   - Add a floor polygon (**F**).  
   - Set properties:
     ```yaml
     ceiling_texture: rubber_pieces
     texture_name: rubber_pieces
     ```
   - **Right-click** to finish.
7. **Lanes**:  
   - Use the **Add Lane** button.  
   - Give each vertex a name and optionally add properties.
8. **Save** the file:  
   - File name format: `<name>.building.yaml`

---

## 🌍 3. Generate the World File

Run the following to generate the Gazebo world:

```bash
ros2 run rmf_building_map_tools building_map_generator gazebo dc_l2.building.yaml world_dc_l2.world ~/.gazebo/models
```

---

## 🏗️ 4. Create `.world` with Embedded Model

1. Copy the meshes:
   ```bash
   cp -r /root/.gazebo/models/building_dc_l2/meshes ./meshes
   ```
<!-- 2. Save the new world as:
   ```
   world_dc_l2_inline.world
   ```
   (with the `model.sdf` content embedded). -->

---

## 🧪 5. Test in Gazebo

```bash
gz sim world_dc_l2.world
```

---

## 🗺️ 6. Generate Navigation Files

```bash
ros2 run rmf_building_map_tools building_map_generator nav dc_l2.building.yaml maps/
ros2 run rmf_building_map_tools building_map_generator navgraph_visualization dc_l2.building.yaml maps/
```

---

## 📂 7. Copy World to RMF Maps

Do **every time** you reopen the docker:

```bash
source install/setup.bash
mkdir -p /root/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/dc_l2/
cp /root/rmf_ws/maps/world_dc_l2_inline.world /root/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/dc_l2/dc_l2.world
```

---

## 🚀 8. Launch Simulation

```bash
cd maps
ros2 launch dc_l2.launch
```
> ⏳ The first run may take longer while Gazebo downloads models.

---

## 📝 9. Test Task Submission

Example task submission (loop task for `tinyRobot`):

```bash
ros2 service call /submit_task rmf_task_msgs/srv/SubmitTask "requester: 'rmf_demos_tasks'
description:
  start_time:
    sec: 0
    nanosec: 0
  priority:
    value: 0
  task_type:
    type: 1  # TYPE_LOOP
  loop:
    task_id: 'loop_tinyRobot_01'
    robot_type: 'tinyRobot'
    num_loops: 1
    start_name: 'dc_door'
    finish_name: 'middle_professor_aisle'"
```

---

## 📚 References

- [RMF Building Map Creation Tutorial (YouTube)](https://www.youtube.com/watch?v=POLjIOs2MaM&t=968s)

- [Google Docs Tutorial](https://docs.google.com/document/d/10Vif_sw5_8SZr7kzbUPEd8Tp6-2p9BgeIjz_cjSJ7so/edit?tab=t.0)


## Gerar o sdf do robô do DC a partir do xacro

```
ros2 run xacro xacro /home/ros/rmf_ws/src/laris_robot/description/robot.urdf.xacro -o /home/ros/robot_laris.urdf
sudo gz sdf -p /home/ros/robot_laris.urdf > /home/ros/rmf_ws/custom_maps/Laris_robot_model/laris_robot.sdf
```
