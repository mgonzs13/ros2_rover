# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

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

# This launch file brings up the Kinect sensor and the necessary nodes for the rover.
# It optionally spins up a discovery server, which can be useful if you want to 
# control the robot from a station on another subnet.

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():
    kinect_pkg_share = get_package_share_directory("kinect_ros2")
    rover_bringup_shared_dir = get_package_share_directory("rover_bringup")
    rover_motor_controller_shared_dir = get_package_share_directory(
        "rover_motor_controller_cpp"
    )
    rover_teleop_shared_dir = get_package_share_directory("rover_teleop")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_USE_STDOUT", "1"
    )
    stdout_linebuf2_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # Enable control to cross subnets
    discovery_server_envvar = SetEnvironmentVariable(name="ROS_DISCOVERY_SERVER", value='131.194.112.46:11811')
    rmw_implementation_envvar = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value='rmw_fastrtps_cpp')

    #
    # LAUNCHES
    #
    launch_discovery_server = ExecuteProcess(
            cmd=['fastdds', 'discovery', '-i','0', '-l', '0.0.0.0', '-p', '11811'],
                 name='fastdds_discovery_server',
                 output='screen'
         )

    kinect_node_action_cmd = Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                namespace="kinect",
            )

    teleop_twist_joy_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_teleop_shared_dir, "launch", "joy_teleop.launch.py")
        )
    )

    rover_motor_controller_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_motor_controller_shared_dir, "launch", "motor_controller.launch.py"
            )
        )
    )

    ld = LaunchDescription()
    ld.add_action(discovery_server_envvar)
    ld.add_action(rmw_implementation_envvar)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_linebuf2_envvar)
    ld.add_action(launch_discovery_server)
    ld.add_action(kinect_node_action_cmd)
    ld.add_action(teleop_twist_joy_action_cmd)
    ld.add_action(rover_motor_controller_action_cmd)


    return ld
