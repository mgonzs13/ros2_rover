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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_path = get_package_share_directory("rover_gazebo")

    nav2_planner = LaunchConfiguration("nav2_planner")
    nav2_planner_cmd = DeclareLaunchArgument(
        "nav2_planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)")

    nav2_controller = LaunchConfiguration("nav2_controller")
    nav2_controller_cmd = DeclareLaunchArgument(
        "nav2_controller",
        default_value="RPP",
        choices=["RPP", "TEB"],
        description="Nav2 controller (RPP or TEB)")

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(pkg_path, "worlds", "moon.world"),
            "initial_pose_z": "2.11",
            "nav2_planner": nav2_planner,
            "nav2_controller": nav2_controller
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(nav2_planner_cmd)
    ld.add_action(nav2_controller_cmd)
    ld.add_action(gazebo_cmd)

    return ld
