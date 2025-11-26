#!/usr/bin/env python3

# MIT License

# Copyright (c) 2025 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get config file
    config_file = os.path.join(
        get_package_share_directory("rover_gazebo"), "config", "odometry.yaml"
    )

    # Odometry node (calculated from wheels) - C++ version
    odometry_node = Node(
        package="rover_gazebo",
        executable="odometry_node",
        name="odometry_node",
        output="screen",
        parameters=[config_file],
    )

    # Ground truth remapper (makes absolute odometry relative to spawn position) - C++ version
    ground_truth_remapper = Node(
        package="rover_gazebo",
        executable="ground_truth_remapper_node",
        name="ground_truth_remapper",
        output="screen",
    )

    return LaunchDescription([odometry_node, ground_truth_remapper])
