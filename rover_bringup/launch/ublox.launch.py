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


from logging import shutdown
import os

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            "rover_bringup"),
        "config")
    params = os.path.join(config_directory, "ublox.yaml")

    ublox_gps_node = Node(package="ublox_gps",
                          executable="ublox_gps_node",
                          output="both",
                          parameters=[params])

    shutdown_event = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=ublox_gps_node,
        on_exit=[EmitEvent(
            event=Shutdown())],
    ))

    ld = LaunchDescription()
    ld.add_action(ublox_gps_node)
    ld.add_action(shutdown_event)
    return ld
