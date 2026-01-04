cmake_minimum_required(VERSION 3.8)
project(global_planner_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include)

add_executable(map_and_planner_node
  src/map_and_planner_node.cpp)

ament_target_dependencies(map_and_planner_node
  rclcpp sensor_msgs geometry_msgs nav_msgs)

install(TARGETS map_and_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
