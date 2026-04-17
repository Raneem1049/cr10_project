# CR10 Project

## Project Overview

`cr10_project` is a ROS 2 Python package for a Dobot CR10-style robot simulation in RViz.

- Part 1 implements custom forward and inverse kinematics, direct `/joint_states` publishing, end-effector tracing, straight-line Cartesian motion, and circular Cartesian motion.
- Part 2 implements parametric pick-and-place of a cube defined by `c, x, y, z, alpha, h`, including mirrored placement and cube orientation visualization.

The project uses custom ROS 2 nodes and kinematics logic only. No MoveIt is used.

## Features

- Analytical IK with FK validation against the URDF model
- Cartesian trajectory generation for line and circle motion
- Custom ROS 2 nodes publishing `/joint_states`
- RViz visualization with robot model, scene markers, and end-effector trace
- Parametric pick-and-place with mirrored target placement
- Pick planning that accounts for cube size and gripper opening limits

## Requirements

- ROS 2 Jazzy
- Python 3
- ROS 2 packages:
  - `ament_index_python`
  - `builtin_interfaces`
  - `geometry_msgs`
  - `launch`
  - `launch_ros`
  - `rclpy`
  - `robot_state_publisher`
  - `rviz2`
  - `sensor_msgs`
  - `std_msgs`
  - `visualization_msgs`

## Build Instructions

```bash
cd /path/to/Project
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run Instructions

### Part 1

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cr10_project part1_demo.launch.py
```

### Part 2

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cr10_project part2_demo.launch.py
```

## Example Parameter Override (Part 2)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cr10_project part2_demo.launch.py \
  c:=0.05 \
  x:=0.46 \
  y:=0.18 \
  z:=0.90 \
  alpha:=0.5235987756 \
  h:=0.65
```

## Project Structure

```text
Project/
├── cr10_project/
│   ├── __init__.py
│   ├── cr10_ik_rviz_node.py
│   ├── cr10_kinematics.py
│   ├── cr10_pick_place.py
│   └── trajectory_generators.py
├── launch/
│   ├── part1_demo.launch.py
│   └── part2_demo.launch.py
├── resource/
│   └── cr10_project
├── rviz/
│   └── project_demo.rviz
├── tests/
│   ├── test_kinematics.py
│   └── test_pick_place.py
├── urdf/
│   └── cr10_project.urdf
├── package.xml
├── setup.py
└── README.md
```

## Notes

- No MoveIt is used anywhere in the project.
- All robot motion is driven by custom IK and custom ROS 2 nodes.
- Part 1 and Part 2 are both launched through standard ROS 2 launch files.
# Project
