# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

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
import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            "rover_bringup"),
        "config")
    params = os.path.join(config_directory, "ublox.yaml")

    ublox_gps_node = Node(package="ublox_gps",
                          executable="ublox_gps_node",
                          output="both",
                          parameters=[params])

    shutdown_event = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=ublox_gps_node,
        on_exit=[EmitEvent(
            event=Shutdown())],
    ))

    ld = LaunchDescription()
    ld.add_action(ublox_gps_node)
    ld.add_action(shutdown_event)
    return ld
