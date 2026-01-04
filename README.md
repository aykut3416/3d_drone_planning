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

- Install QGroundControl-x86_64.AppImage and make it executable and run.
- Install PX4 and Micro XRCE-DDS steps as in https://docs.px4.io/main/en/ros2/user_guide
- Download packages into your ros2 workspace src folder
- Build them with "colcon build"
- run start_sim.sh file in the parent folder of "PX4-Autopilot" folder


