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


from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    motors_command_parser_node_cmd = Node(
        name="motors_command_parser_node",
        package="rover_gazebo",
        executable="motors_command_parser_node",
        output="log",
    )

    vel_parser_node_cmd = Node(
        name="vel_parser_node",
        package="rover_motor_controller_cpp",
        executable="vel_parser_node",
        output="log",
    )

    ld = LaunchDescription()

    ld.add_action(motors_command_parser_node_cmd)
    ld.add_action(vel_parser_node_cmd)

    return ld
