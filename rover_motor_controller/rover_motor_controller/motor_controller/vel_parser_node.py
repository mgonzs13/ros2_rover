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

import math
from typing import List

from rclpy.node import Node

from geometry_msgs.msg import Twist
from rover_msgs.msg import MotorsCommand

# Radius from 255 (0 degrees) to 55 centimeters (45 degrees)
MAX_RADIUS = 255
MIN_RADIUS = 55


class VelParserNode(Node):

    def __init__(self):
        super().__init__("vel_parser_node")

        # declaring params
        self.declare_parameter("hardware_distances",
                               [23.0, 25.5, 28.5, 26.0])
        self.declare_parameter("enc_min", 250)
        self.declare_parameter("enc_max", 750)

        # Speed [-100, +100] * 6 = [-600, +600]
        self.declare_parameter("speed_factor", 10)

        # getting params
        hardware_distances = self.get_parameter(
            "hardware_distances").get_parameter_value().double_array_value
        enc_min = self.get_parameter(
            "enc_min").get_parameter_value().integer_value
        enc_max = self.get_parameter(
            "enc_max").get_parameter_value().integer_value
        self.speed_factor = self.get_parameter(
            "speed_factor").get_parameter_value().integer_value

        self.d1 = hardware_distances[0]
        self.d2 = hardware_distances[1]
        self.d3 = hardware_distances[2]
        self.d4 = hardware_distances[3]

        self.enc_min = enc_min
        self.enc_max = enc_max

        # pubs and subs
        self.publisher = self.create_publisher(
            MotorsCommand, "motors_command", 10)

        self.subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.callback,
            10)

    def callback(self, msg: Twist) -> None:
        """
        Callback function called when a Twist message is received from /cmd_vel topic
        :param list msg: Command velocity message
        """

        motors_command = MotorsCommand()

        norm_speed = self.normalize(msg.linear.x, -1, 1, -100, 100)
        norm_steering = self.normalize(msg.angular.z, -1, 1, -100, 100) * -1

        new_speeds = self.calculate_velocity(norm_speed, norm_steering)
        new_ticks = self.calculate_target_tick(
            self.calculate_target_deg(norm_steering))

        motors_command.drive_motor = [int(ele) for ele in new_speeds]
        motors_command.corner_motor = [int(ele) for ele in new_ticks]

        self.publisher.publish(motors_command)

    @staticmethod
    def normalize(value: float, old_min: float, old_max:
                  float, new_min: float, new_max: float) -> float:
        return (new_max - new_min) * ((value - old_min) / (old_max - old_min)) + new_min

    @staticmethod
    def deg_to_tick(deg: float, e_min: float, e_max: float) -> float:
        """
        Converts a degrees to tick value
        :param float deg  : Degrees value desired
        :param int e_min: The minimum encoder value based on physical stop
        :param int e_max: The maximum encoder value based on physical stop
        """

        temp = (e_max + e_min) / 2 + ((e_max - e_min) / 90) * deg

        if temp < e_min:
            temp = e_min
        elif temp > e_max:
            temp = e_max

        return temp

    def calculate_velocity(self, v: float, r: float) -> List[float]:
        """
        Returns a list of speeds for each individual drive motor based on current turning radius
        :param float v: Drive speed command range from -100 to 100
        :param float r: Current turning radius range from -100 to 100
        """

        if v == 0:
            return [0] * 6

        if abs(r) <= 5:
            # No turning radius, all wheels same speed
            # Go ahead/Go back
            velocity = [v, v, v, -v, -v, -v]
        else:
            # Get radius in centimeters (MAX_RADIUS (255) to MIN_RADIUS (55))
            radius = MAX_RADIUS - \
                (((MAX_RADIUS - MIN_RADIUS) * abs(r)) / 100.0)

            a = math.pow(self.d2, 2)  # Back - D2
            b = math.pow(self.d3, 2)  # Front - D3

            c = math.pow(radius + self.d1, 2)  # Front/Back - Farthest
            d = math.pow(radius - self.d1, 2)  # Front/Back - Closest

            e = radius - self.d4  # Center - Closest
            f = radius + self.d4  # Center - Farthest

            if radius < 111:
                # Front - Farthest wheel is the Farthest
                rx = math.sqrt(b + c)
            else:
                # Center - Farthest wheel is the Farthest
                rx = f

            # Get speed of each wheel
            abs_v1 = abs(v) * (math.sqrt(b + c)) / rx
            abs_v2 = abs(v) * (f / rx)
            abs_v3 = abs(v) * (math.sqrt(a + c)) / rx
            abs_v4 = (abs(v) * math.sqrt(b + d)) / rx
            abs_v5 = abs(v) * (e / rx)
            abs_v6 = (abs(v) * math.sqrt(a + d)) / rx

            if v < 0:
                # Go back
                if r < 0:
                    # Turn Left
                    velocity = [-abs_v4, -abs_v5, -
                                abs_v6, abs_v1, abs_v2, abs_v3]
                else:
                    # Turn Right
                    velocity = [-abs_v1, -abs_v2, -
                                abs_v3, abs_v4, abs_v5, abs_v6]
            else:
                # Go ahead
                if r < 0:
                    # Turn Left
                    velocity = [abs_v4, abs_v5,
                                abs_v6, -abs_v1, -abs_v2, -abs_v3]
                else:
                    # Turn Right
                    velocity = [abs_v1, abs_v2,
                                abs_v3, -abs_v4, -abs_v5, -abs_v6]

        speed = [self.speed_factor * i for i in velocity]

        # Set the speeds between the range [-max_speed, +max_speed]
        return speed

    def calculate_target_deg(self, radius: float) -> List[float]:
        """
        Takes a turning radius and calculates what angle [degrees] each corner should be at
        :param float radius: Radius drive command, ranges from -100 (left) to +100 (right)
        """

        # Scaled from MAX_RADIUS (255) to MIN_RADIUS (55) centimeters
        if radius == 0:
            r = MAX_RADIUS
        elif -100 <= radius <= 100:
            r = MAX_RADIUS - abs(radius) * int(MAX_RADIUS / 100)
        else:
            r = MAX_RADIUS

        if r == MAX_RADIUS:
            return [0] * 4

        # Turn Right - Turn Left
        # Front Left - Front Right
        ang7 = math.degrees(math.atan(self.d3 / (abs(r) + self.d1)))

        # Front Right - Front Left
        ang8 = math.degrees(math.atan(self.d3 / (abs(r) - self.d1)))

        # Back Left - Back Right
        ang9 = math.degrees(math.atan(self.d2 / (abs(r) + self.d1)))

        # Back Right - Back Left
        ang10 = math.degrees(math.atan(self.d2 / (abs(r) - self.d1)))

        if radius < 0:
            # Turn Left
            angles = [-ang8, -ang7, ang10, ang9]
        else:
            # Turn Right
            angles = [ang7, ang8, -ang9, -ang10]

        return angles

    def calculate_target_tick(self, tar_enc: float) -> List[float]:
        """
        Takes the target angle and gets what encoder tick that value is for position control
        :param list [float] tar_enc: List of target angles in degrees for each corner
        """

        tick = []

        for i in range(4):
            tick.append(self.deg_to_tick(
                tar_enc[i], self.enc_min, self.enc_max))

        return tick
