import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    pkg_rover_localization = get_package_share_directory("rover_localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    rgbd_odometry_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "rgbd_odometry.launch.py")
        )
    )

    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "rtabmap.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)

    ld.add_action(rgbd_odometry_cmd)
    ld.add_action(rtabmap_cmd)
    ld.add_action(ekf_cmd)

    return ld
