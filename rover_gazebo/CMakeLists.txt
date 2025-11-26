cmake_minimum_required(VERSION 3.5)
project(rover_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

INCLUDE_DIRECTORIES(
  include
)

add_executable(motors_command_parser_node src/motors_command_parser_node.cpp)
target_link_libraries(motors_command_parser_node)
ament_target_dependencies(motors_command_parser_node 
  rclcpp
  rover_msgs
  std_msgs
  trajectory_msgs
)

add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

add_executable(ground_truth_remapper_node src/ground_truth_remapper_node.cpp)
ament_target_dependencies(ground_truth_remapper_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  motors_command_parser_node
  odometry_node
  ground_truth_remapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
