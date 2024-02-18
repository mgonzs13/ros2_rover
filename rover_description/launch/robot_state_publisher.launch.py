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


from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        get_package_share_directory("rover_description"),
        "robots",
        "rover.urdf.xacro"]
    )

    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"]
    )

    robot_state_publisher_cmd = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": ParameterValue(
                Command(["xacro", " ", xacro_file]),
                value_type=str)},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )

    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld
