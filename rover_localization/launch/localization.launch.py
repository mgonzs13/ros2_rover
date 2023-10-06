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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    pkg_rover_localization = get_package_share_directory("rover_localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    use_slam = LaunchConfiguration("use_slam")
    use_slam_cmd = DeclareLaunchArgument(
        "use_slam",
        default_value="False",
        description="Whether to use octomap instead of rtabmap vslam")

    map_file_path = LaunchConfiguration("map_file_path")
    map_file_path_cmd = DeclareLaunchArgument(
        "map_file_path",
        default_value=os.path.join(
            pkg_rover_localization, "maps", "moon", "moon.yaml"),
        description="Path to map YAML file")

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_params_file_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(
            get_package_share_directory("rover_navigation"), "params", "nav2.yaml"),
        description="Path to nav2 YAML file")

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
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(PythonExpression([use_slam]))
    )

    octomap_to_gridmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "octomap_to_gridmap.launch.py")
        ),
        condition=IfCondition(PythonExpression([use_slam]))
    )

    octomap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "octomap.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file_path,
            "params_file": nav2_params_file
        }.items(),
        condition=UnlessCondition(PythonExpression([use_slam]))
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization,
                         "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    tf_to_base_footprint_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0.22", "0", "0", "0", "base_footprint", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(use_slam_cmd)
    ld.add_action(map_file_path_cmd)
    ld.add_action(nav2_params_file_cmd)

    ld.add_action(rgbd_odometry_cmd)
    ld.add_action(rtabmap_cmd)
    ld.add_action(octomap_to_gridmap_cmd)
    ld.add_action(octomap_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(tf_to_base_footprint_cmd)

    return ld
