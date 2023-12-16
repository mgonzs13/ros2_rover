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
    joy_config = launch.substitutions.LaunchConfiguration("joy_config")
    joy_dev = launch.substitutions.LaunchConfiguration("joy_dev")
    config_filepath = launch.substitutions.LaunchConfiguration(
        "config_filepath")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "joy_config", default_value="ps3"),

        launch.actions.DeclareLaunchArgument(
            "joy_dev", default_value="/dev/input/js0"),

        launch.actions.DeclareLaunchArgument("config_filepath", default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory("rover_teleop"), "config", "")),
            joy_config, launch.substitutions.TextSubstitution(text=".yaml")]),

        launch_ros.actions.Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_linux_node",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.3,
                "autorepeat_rate": 0.0,
            }]),

        launch_ros.actions.Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[config_filepath]),
    ])
