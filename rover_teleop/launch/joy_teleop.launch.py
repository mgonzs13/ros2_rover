
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
