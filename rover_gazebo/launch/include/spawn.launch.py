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
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="0.0",
        description="Initial pose x")

    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="0.0",
        description="Initial pose y")

    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z",
        default_value="0.0",
        description="Initial pose z")

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw")

    ### NODES ###
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "rover",
                   "-topic", "robot_description",
                   "-timeout", "120",
                   "-x", initial_pose_x,
                   "-y", initial_pose_y,
                   "-z", initial_pose_z,
                   "-Y", initial_pose_yaw],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_state_broadcaster_spawner = Node(
        name="joint_state_broadcaster_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    position_controller_spawner = Node(
        name="position_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    velocity_controller_spawner = Node(
        name="velocity_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    ### LAUNCH ###
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                "rover_description"), "launch", "robot_state_publisher.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(spawn_entity_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(position_controller_spawner)
    ld.add_action(velocity_controller_spawner)
    ld.add_action(robot_state_publisher_cmd)

    return ld
