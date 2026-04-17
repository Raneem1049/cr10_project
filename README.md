# CR10 Robot Simulation Project

## What Is This Project?

This project simulates a 6-joint robot arm (based on the Dobot CR10) inside RViz, the standard ROS 2 3D visualizer. The robot moves entirely through custom Python code — no external motion-planning library (MoveIt) is used.

The project is split into two parts:

- **Part 1** — The robot moves its end-effector (the tip of the arm) along a straight line and then around a circle. This demonstrates that the kinematics maths is working correctly.
- **Part 2** — The robot picks up a coloured cube from one table, carries it across, and places it on a mirrored target on the opposite side. The cube rotates with the gripper exactly as a real grasped object would.

---

## Requirements

Before running anything, make sure you have the following installed:

- **ROS 2 Jazzy** (the ROS 2 distribution this project was built and tested with)
- **Python 3**
- The following ROS 2 packages (they come with a standard Jazzy desktop install):
  `rclpy`, `robot_state_publisher`, `rviz2`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, `builtin_interfaces`, `launch`, `launch_ros`, `ament_index_python`

---

## How to Build

Open a terminal, navigate to the project folder, and run these commands once:

```bash
cd /path/to/Project
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

> **What this does:**
> - `colcon build` compiles the package and creates the `install/` folder.
> - `source install/setup.bash` makes ROS 2 aware of the newly built package so you can run it.

You only need to build once (or again if you change source files and are not using `--symlink-install`).

---

## How to Run

### Part 1 — Line and Circle Motion

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cr10_project part1_demo.launch.py
```

**What you will see in RViz:**
- The robot arm moves its end-effector along a straight line in 3D space.
- It then sweeps the end-effector around a full circle.
- An orange trace line is drawn showing where the tip has been.

You can also choose the motion mode explicitly:

```bash
# Line motion only
ros2 launch cr10_project part1_demo.launch.py mode:=line

# Circle motion only
ros2 launch cr10_project part1_demo.launch.py mode:=circle

# Keep repeating the demo forever
ros2 launch cr10_project part1_demo.launch.py repeat_demo:=true
```

---

### Part 2 — Pick and Place

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cr10_project part2_demo.launch.py
```

**What you will see in RViz:**
- Two small tables appear — one on each side of the robot.
- An orange cube sits on the left table at the angle set by the `alpha` parameter.
- A transparent blue ghost cube shows where the cube should end up on the right table.
- The robot moves to the cube, closes its gripper, lifts the cube, carries it to the other side, and places it down.
- While the cube is being carried, it rotates with the gripper, matching the final target orientation when released.

#### Customising the Cube (optional)

You can change the cube's size, position, tilt, and table height using these parameters:

| Parameter | Meaning | Default |
|-----------|---------|---------|
| `c` | Side length of the cube (metres) | `0.05` |
| `x` | How far forward the cube is from the robot base (metres) | `0.48` |
| `y` | How far to the side the cube is (metres) | `0.18` |
| `z` | Height of the cube centre above the floor (metres) | `0.90` |
| `alpha` | Rotation of the cube around the vertical axis (radians) | `0.349` (≈ 20°) |
| `h` | Height of the table surface above the robot base frame (metres) | `0.65` |

Example — run with a cube tilted at 30° (π/6 radians):

```bash
ros2 launch cr10_project part2_demo.launch.py \
  c:=0.05 \
  x:=0.46 \
  y:=0.18 \
  z:=0.90 \
  alpha:=0.5235987756 \
  h:=0.65
```

Run once (no loop):

```bash
ros2 launch cr10_project part2_demo.launch.py repeat_demo:=false
```

---

## File-by-File Guide

### `cr10_project/cr10_kinematics.py`

This is the core maths file. It contains two things:

1. **Forward kinematics (FK)** — Given the 6 joint angles, calculate where the tip of the arm ends up in 3D space. Think of it as: *"if I set all the joints to these values, where does the hand end up?"*

2. **Inverse kinematics (IK)** — The reverse: given a target position in 3D space, calculate what joint angles are needed to reach it. The solver also accepts an optional `wrist_yaw` value to control the rotation of the gripper around the vertical axis.

The geometry is taken directly from the URDF file so the maths matches what RViz displays.

---

### `cr10_project/trajectory_generators.py`

This file provides helper functions for generating smooth paths:

- **`interpolate_line`** — Produces a list of evenly spaced 3D points between two positions (used for straight-line motion).
- **`circle_parallel_to_ground`** — Produces a list of 3D points arranged in a circle at a fixed height (used for circular motion).
- **`interpolate_pose`** — Interpolates between two full poses (position + gripper yaw), used to smoothly blend between pick-and-place keyframes.
- **`mirror_point` / `mirror_yaw`** — Flips a point or angle across the Y-axis, used to compute the mirrored placement target in Part 2.

---

### `cr10_project/cr10_ik_rviz_node.py`

This is the **Part 1 ROS 2 node**. It:

1. Generates a line trajectory and/or a circle trajectory using `trajectory_generators.py`.
2. For each point on the path, calls the IK solver to get joint angles.
3. Publishes those joint angles to `/joint_states` at 30 Hz so RViz animates the robot.
4. Also publishes an orange trace line marker showing the path the end-effector has drawn.

---

### `cr10_project/cr10_pick_place.py`

This is the **Part 2 ROS 2 node** plus all the pick-and-place planning logic. It contains:

- **`CubeParameters`** — A simple data container holding the cube's size, position, tilt (`alpha`), and table height (`h`).
- **`source_grasp_pose` / `mirrored_target_pose`** — Calculate the approach and grasp positions for picking up and placing the cube, accounting for the cube's orientation so the gripper jaws align with the cube sides.
- **`build_pick_place_keyframes`** — Defines the sequence of key waypoints the robot visits: home → approach → grasp → lift → transfer → place → return. The transfer path is routed safely away from the robot base to avoid self-collision.
- **`build_pick_place_trajectory`** — Expands the keyframes into a smooth dense trajectory by interpolating between each pair of keyframes.
- **`CR10PickPlaceNode`** — The ROS 2 node that runs at 30 Hz, publishes joint states to animate the robot, and publishes RViz markers for the cube, the ghost target, the two tables, and a status text overlay. When the cube is being carried, its orientation in RViz tracks the real gripper rotation.

---

### `urdf/cr10_project.urdf`

The robot description file. It defines every link (base, upper arm, forearm, wrist, tool) and every joint (what type, where it is, which axis it rotates around, and its angle limits). RViz reads this file to know how to draw the robot. The kinematics code was written to match this file exactly.

---

### `launch/part1_demo.launch.py`

A ROS 2 launch file that starts three things at once for Part 1:
1. `robot_state_publisher` — reads the URDF and publishes the robot's transform tree.
2. `rviz2` — opens the visualiser with the pre-configured display settings.
3. `cr10_ik_rviz_node` — the Part 1 motion node.

---

### `launch/part2_demo.launch.py`

Same structure as Part 1's launch file, but starts the `cr10_pick_place` node instead, and forwards the cube parameters (`c`, `x`, `y`, `z`, `alpha`, `h`) from the command line into the node.

---

### `rviz/project_demo.rviz`

A saved RViz configuration file. It tells RViz which topics to display (robot model, joint states, trace marker, scene markers) and how the camera should be positioned when the demo opens. You do not need to edit this file manually.

---

### `tests/test_kinematics.py`

Automated tests for the kinematics code. They verify things like:
- The robot tip is at the right position when all joints are zero.
- Positive joint 2 moves the arm downward (matches URDF convention).
- Running FK then IK on the same point gives back the original joint values (round-trip test).
- The `reference_joints` feature keeps the arm on the same elbow branch when following a continuous path.

---

### `tests/test_pick_place.py`

Automated tests for the pick-and-place planning. They verify things like:
- The cube centre is computed correctly from the table height parameter.
- The mirrored target flips the X position and orientation correctly.
- Every point in the trajectory is reachable by the IK solver.
- No joint moves by more than 10° between consecutive trajectory samples (smoothness check).
- A cube that is too large for the gripper is correctly rejected.

---

### `package.xml` and `setup.py`

ROS 2 bookkeeping files. `package.xml` declares the package name, version, and dependencies. `setup.py` tells the Python build system where the node entry points are so `ros2 launch` can find them.

---

## Quick Reference

| Goal | Command |
|------|---------|
| Build the package | `colcon build` |
| Run Part 1 (line + circle) | `ros2 launch cr10_project part1_demo.launch.py` |
| Run Part 2 (pick and place) | `ros2 launch cr10_project part2_demo.launch.py` |
| Run Part 2 once (no repeat) | `ros2 launch cr10_project part2_demo.launch.py repeat_demo:=false` |
| Run all tests | `python3 -m pytest tests/ -v` |