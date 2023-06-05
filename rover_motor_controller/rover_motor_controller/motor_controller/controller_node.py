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


"""
Motor controller node.
"""

from rclpy.node import Node

from rover_msgs.msg import MotorsCommand
from rover_motor_controller.lx16a import MotorController


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # declaring params
        self.declare_parameter("motor_controller_device",
                               "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        # getting params
        motor_controller_device = self.get_parameter(
            "motor_controller_device").get_parameter_value().string_value
        baud_rate = self.get_parameter(
            "baud_rate").get_parameter_value().integer_value

        self.motor_controller = MotorController(
            motor_controller_device, baud_rate)

        # sub
        self.subscription = self.create_subscription(
            MotorsCommand,
            "motors_command",
            self.callback,
            10)

    def callback(self, msg: MotorsCommand) -> None:
        """
        Callback function called when a MotorsCommand message is received from /motors_command topic
        :param list msg: A list of corner and motor lists values
        """

        # Send angle values to corner motors
        self.motor_controller.corner_to_position(msg.corner_motor)

        # Send speed values to drive motors
        self.motor_controller.send_motor_duty(msg.drive_motor)

    def shutdown(self) -> None:
        """
        Stop motors
        """

        self.get_logger().info("Killing motors")
        self.motor_controller.kill_motors()
