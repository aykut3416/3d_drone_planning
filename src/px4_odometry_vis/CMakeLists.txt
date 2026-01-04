cmake_minimum_required(VERSION 3.8)
project(px4_odometry_vis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(px4_odometry_vis_node
  src/px4_odometry_vis_node.cpp
)

ament_target_dependencies(px4_odometry_vis_node
  rclcpp
  px4_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  px4_odometry_vis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
