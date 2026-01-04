#!/bin/bash

# Ortak terminal
TERMINAL="gnome-terminal"
pkill -f gz

sleep 2

$TERMINAL --tab --title="PX4 SITL" -- bash -c "
cd ~/PX4-Autopilot
make px4_sitl gz_x500
exec bash
"

sleep 10

$TERMINAL --tab --title="XRCE Agent" -- bash -c "
MicroXRCEAgent udp4 -p 8888
exec bash
"

sleep 5

$TERMINAL --tab --title="Odometry" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
ros2 run px4_odometry_vis px4_odometry_vis_node
exec bash
"

sleep 5

$TERMINAL --tab --title="RViz" -- bash -c "
source /opt/ros/humble/setup.bash
rviz2
exec bash
"

sleep 5

$TERMINAL --tab --title="Offboard" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
ros2 run px4_path_offboard path_offboard_control
exec bash
"

sleep 2

$TERMINAL --tab --title="Plan3D" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
ros2 run global_planner_3d map_and_planner_node 
exec bash
"
