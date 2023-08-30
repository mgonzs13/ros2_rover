# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory("rover_navigation")
    launch_dir = os.path.join(pkg_dir, "launch")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace")

    use_namespace = LaunchConfiguration("use_namespace")
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true")

    autostart = LaunchConfiguration("autostart")
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="Automatically startup the nav2 stack")

    use_composition = LaunchConfiguration("use_composition")
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="True",
        description="Whether to use composed bringup")

    use_respawn = LaunchConfiguration("use_respawn")
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn", default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.")

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="cmd_vel topic (for remmaping)")

    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees", "navigate_w_replanning_only_if_goal_is_updated.xml"),
        description="Full path to the behavior tree xml file to use")

    remappings = [("/tf", "tf"),
                  ("/tf_static", "tf_static"),
                  ("/cmd_vel", cmd_vel_topic)]

    params_file = os.path.join(pkg_dir, "params", "nav2.yaml")
    param_substitutions = {
        "use_sim_time": use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name="nav2_container",
            package="rclcpp_components",
            executable="component_container_isolated",
            parameters=[configured_params, {"autostart": autostart}],
            remappings=remappings,
            output="screen"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, "navigation.launch.py")),
            launch_arguments={"namespace": namespace,
                              "cmd_vel_topic": cmd_vel_topic,
                              "default_nav_to_pose_bt_xml": default_bt_xml_filename,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file,
                              "use_composition": use_composition,
                              "use_respawn": use_respawn,
                              "container_name": "nav2_container"}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(cmd_vel_topic_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
