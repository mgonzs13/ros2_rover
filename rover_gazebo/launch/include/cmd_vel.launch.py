# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


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
