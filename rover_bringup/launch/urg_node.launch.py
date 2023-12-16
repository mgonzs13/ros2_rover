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
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    sensor_interface = launch.substitutions.LaunchConfiguration(
        "sensor_interface")
    config_filepath = launch.substitutions.LaunchConfiguration(
        "config_filepath")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "sensor_interface", default_value="serial",
            description="sensor_interface: supported: serial, ethernet"),

        launch.actions.DeclareLaunchArgument("config_filepath", default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory("urg_node"), "launch", "urg_node_")),
            sensor_interface, launch.substitutions.TextSubstitution(text=".yaml")]),


        launch_ros.actions.Node(
            package="urg_node",
            executable="urg_node_driver",
            output="screen",
            parameters=[config_filepath]),
    ])
