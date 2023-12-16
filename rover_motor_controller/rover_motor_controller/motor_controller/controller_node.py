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
