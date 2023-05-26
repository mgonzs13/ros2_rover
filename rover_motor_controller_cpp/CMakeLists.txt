cmake_minimum_required(VERSION 3.5)
project(rover_motor_controller_cpp)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


INCLUDE_DIRECTORIES(
  include
)

### CONTROLLER NODE
set(CONTROLLER_NODE_SOURCES
  src/controller_node_main.cpp
  src/lx16a/serial.cpp
  src/lx16a/lx16a.cpp
  src/lx16a/motor_controller.cpp
  src/motor_controller/controller_node.cpp
)
add_executable(controller_node ${CONTROLLER_NODE_SOURCES})
target_link_libraries(controller_node pthread boost_system)
ament_target_dependencies(controller_node rclcpp rover_msgs)


### VEL PARSER NODE
set(VEL_PARSER_NODE_SOURCES
  src/vel_parser_node_main.cpp
  src/motor_controller/vel_parser_node.cpp
)
add_executable(vel_parser_node ${VEL_PARSER_NODE_SOURCES})
target_link_libraries(vel_parser_node)
ament_target_dependencies(vel_parser_node rclcpp rover_msgs geometry_msgs)


# INSTALL
install(TARGETS 
  controller_node
  vel_parser_node
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
