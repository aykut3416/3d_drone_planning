# ROS 2 UAV Planning and Control Workspace

This repository contains multiple ROS 2 packages for UAV navigation,
path planning, and PX4 offboard control.

## Packages

- **global_planner_3d**
  - 3D grid-based A* planner
  - Random obstacle generation
  - Path reduction for PX4

- **px4_odometry_vis**
  - Converts PX4 odometry to ENU
  - RViz visualization using mesh markers

- **path_offboard_control**
  - PX4 offboard position control
  - Follows reduced waypoint paths

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- PX4 Autopilot
- Gazebo (gz / ignition)

## Build Instructions

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
