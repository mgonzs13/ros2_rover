
cmake_minimum_required(VERSION 3.5)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rover_description REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
