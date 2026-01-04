cmake_minimum_required(VERSION 3.8)
project(px4_path_offboard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(path_offboard_control src/path_offboard_control.cpp)

ament_target_dependencies(path_offboard_control
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS path_offboard_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
