
cmake_minimum_required(VERSION 3.5)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf robots hooks
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv")
ament_package()
