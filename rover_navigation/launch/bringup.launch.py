import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory("rover_navigation")
    launch_dir = os.path.join(pkg_dir, "launch")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # Create the launch configuration variables
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="cmd_vel topic (for remmaping)")

    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace")

    use_namespace = LaunchConfiguration("use_namespace")
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Whether to apply a namespace to the navigation stack")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    param_file_name = "nav2" + ".yaml"
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            pkg_dir,
            "params",
            param_file_name))
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes")

    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees", "navigate_w_replanning_time.xml"),
        description="Full path to the behavior tree xml file to use")

    autostart = LaunchConfiguration("autostart")
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True",
        description="Automatically startup the nav2 stack")

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, "navigation.launch.py")),
            launch_arguments={"cmd_vel_topic": cmd_vel_topic,
                              "namespace": namespace,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file,
                              "default_bt_xml_filename": default_bt_xml_filename,
                              "use_lifecycle_mgr": "False",
                              "map_subscribe_transient_local": "True"}.items()),

    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(cmd_vel_topic_cmd)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
