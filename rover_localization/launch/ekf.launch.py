from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    params_file = os.path.join(get_package_share_directory(
        "rover_localization"), "config", "ekf.yaml")

    param_substitutions = {
        "use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="log",
        parameters=[configured_params],
        remappings=[("odometry/filtered", "/odom"),
                    ("accel/filtered", "/accel")])

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)

    ld.add_action(ekf_cmd)

    return ld
