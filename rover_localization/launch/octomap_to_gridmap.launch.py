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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    octomap_server_node_cmd = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server_node",
        namespace="octomap_server",
        remappings=[("cloud_in", "/cloud_map")],
    )

    octomap_to_gridmap_node_cmd = Node(
        package="rover_localization",
        executable="octomap_to_gridmap",
        name="octomap_to_gridmap",
        output="screen",
        remappings=[("octomap_full", "/octomap_server/octomap_full")],
    )

    ld = LaunchDescription()
    ld.add_action(octomap_server_node_cmd)
    ld.add_action(octomap_to_gridmap_node_cmd)
    return ld
