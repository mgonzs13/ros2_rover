import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    teleop_twist_joy_shared_dir = get_package_share_directory(
        "teleop_twist_joy")
    rover_motor_controller_shared_dir = get_package_share_directory(
        "rover_motor_controller")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    namespace_action_cmd = PushRosNamespace("rover")

    #
    # NODES
    #

    joy_linux_action_cmd = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node"
    )

    #
    # LAUNCHES
    #

    teleop_twist_joy_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_twist_joy_shared_dir, "teleop-launch.py"))
    )

    rover_motor_controller_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_motor_controller_shared_dir, "motor_controller_launch.py"))
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(namespace_action_cmd)

    ld.add_action(joy_linux_action_cmd)
    ld.add_action(teleop_twist_joy_action_cmd)
    ld.add_action(rover_motor_controller_action_cmd)

    return ld
