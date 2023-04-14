#!/bin/bash

cd /root
git clone https://github.com/mgonzs13/ros2_utils_scripts.git
echo "source ${PWD}/ros2_utils_scripts/scripts/all.sh" >>/root/.bashrc
echo "rosconfig -d humble -w /root/ros2_ws" >>/root/.bashrc
