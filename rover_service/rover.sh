#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="131.194.112.46:11811"

source /opt/ros/jazzy/setup.bash
source /home/trippy/code/ros2_ws/install/setup.bash
ros2 launch rover_bringup trippy.launch.py
