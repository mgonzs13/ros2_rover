import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_rover_localization = get_package_share_directory("rover_localization")

    rtab_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "rgbd_odometry.launch.py")
        )
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "ekf.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(rtab_client_cmd)
    ld.add_action(ekf_cmd)

    return ld
