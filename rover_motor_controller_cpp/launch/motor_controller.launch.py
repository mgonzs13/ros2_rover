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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


def generate_launch_description():

    pkg_name = "rover_motor_controller_cpp"
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    #
    # ARGS
    #

    hardware_distances = LaunchConfiguration("hardware_distances")
    declare_hardware_distances_cmd = DeclareLaunchArgument(
        "hardware_distances",
        default_value="[23.0, 25.5, 28.5, 26.0]",
        description="Rover hardware distances")

    enc_min = LaunchConfiguration("enc_min")
    declare_enc_min_cmd = DeclareLaunchArgument(
        "enc_min",
        default_value="250",
        description="enc_min")

    speed_factor = LaunchConfiguration("speed_factor")
    declare_speed_factor_cmd = DeclareLaunchArgument(
        "speed_factor",
        default_value="10",
        description="Speed [-100, +100] * 6 = [-1000, +1000]")

    enc_max = LaunchConfiguration("enc_max")
    declare_enc_max_cmd = DeclareLaunchArgument(
        "enc_max",
        default_value="750",
        description="enc_max")

    motor_controller_device = LaunchConfiguration("motor_controller_device")
    declare_motor_controller_device_cmd = DeclareLaunchArgument(
        "motor_controller_device",
        default_value="/dev/ttyUSB0",
        description="Motor controller device")

    baud_rate = LaunchConfiguration("baud_rate")
    declare_baud_rate_cmd = DeclareLaunchArgument(
        "baud_rate",
        default_value="750",
        description="baud rate")

    #
    # NODES
    #

    vel_parser_node_cmd = Node(
        package=pkg_name,
        executable="vel_parser_node",
        name="vel_parser_node",
        parameters=[{"hardware_distances": hardware_distances,
                     "enc_min": enc_min,
                     "enc_max": enc_max,
                     "speed_factor": speed_factor}]
    )

    controller_node_cmd = Node(
        package=pkg_name,
        executable="controller_node",
        name="controller_node",
        parameters=[{"motor_controller_device": motor_controller_device,
                     "baud_rate": baud_rate}]
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_hardware_distances_cmd)
    ld.add_action(declare_enc_min_cmd)
    ld.add_action(declare_enc_max_cmd)
    ld.add_action(declare_speed_factor_cmd)
    ld.add_action(declare_motor_controller_device_cmd)
    ld.add_action(declare_baud_rate_cmd)

    ld.add_action(vel_parser_node_cmd)
    ld.add_action(controller_node_cmd)

    return ld
