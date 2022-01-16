
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
