
"""
Lewansoul wrapper.
"""

from typing import List

from .lx16a import LX16A
from .lx16a_consts import (
    MOTOR_LEFT_FRONT,
    MOTOR_LEFT_MIDDLE,
    MOTOR_LEFT_BACK,
    MOTOR_RIGHT_FRONT,
    MOTOR_RIGHT_MIDDLE,
    MOTOR_RIGHT_BACK,
    SERVO_LEFT_FRONT,
    SERVO_RIGHT_FRONT,
    SERVO_LEFT_BACK,
    SERVO_RIGHT_BACK
)


class MotorController(object):
    """
    MotorController class contains the methods necessary to send commands to
    the motor controllers for the corner and drive motors.
    """

    def __init__(self, serial_port: str, baud_rate: int):

        self.lx16a = LX16A(serial_port, baud_rate, timeout=1)

        drive_ticks = [0, 0, 0, 0, 0, 0]

        self.lx16a.set_motor_mode(MOTOR_LEFT_FRONT, drive_ticks[0])
        self.lx16a.set_motor_mode(MOTOR_LEFT_MIDDLE, drive_ticks[1])
        self.lx16a.set_motor_mode(MOTOR_LEFT_BACK, drive_ticks[2])
        self.lx16a.set_motor_mode(MOTOR_RIGHT_FRONT, drive_ticks[3])
        self.lx16a.set_motor_mode(MOTOR_RIGHT_MIDDLE, drive_ticks[4])
        self.lx16a.set_motor_mode(MOTOR_RIGHT_BACK, drive_ticks[5])
        self.lx16a.set_servo_mode(SERVO_LEFT_FRONT)
        self.lx16a.set_servo_mode(SERVO_RIGHT_FRONT)
        self.lx16a.set_servo_mode(SERVO_LEFT_BACK)
        self.lx16a.set_servo_mode(SERVO_RIGHT_BACK)

    def corner_to_position(self, corner_ticks: List[int]) -> None:
        """
        Method to send position commands to the corner motors

        :param list corner_ticks: A list of ticks for each of the corner motors to move to
        """

        self.lx16a.move_prepare(SERVO_LEFT_FRONT, corner_ticks[0])
        self.lx16a.move_prepare(SERVO_RIGHT_FRONT, corner_ticks[1])
        self.lx16a.move_prepare(SERVO_LEFT_BACK, corner_ticks[2])
        self.lx16a.move_prepare(SERVO_RIGHT_BACK, corner_ticks[3])
        self.lx16a.move_start()

    def send_motor_duty(self, drive_ticks: List[int]) -> None:
        """
        Method to send position commands to the drive motors

        :param list drive_ticks: A list of ticks for each of the drice motors to speed to
        """

        self.lx16a.set_motor_mode(MOTOR_LEFT_FRONT, drive_ticks[0])
        self.lx16a.set_motor_mode(MOTOR_LEFT_MIDDLE, drive_ticks[1])
        self.lx16a.set_motor_mode(MOTOR_LEFT_BACK, drive_ticks[2])
        self.lx16a.set_motor_mode(MOTOR_RIGHT_FRONT, drive_ticks[3])
        self.lx16a.set_motor_mode(MOTOR_RIGHT_MIDDLE, drive_ticks[4])
        self.lx16a.set_motor_mode(MOTOR_RIGHT_BACK, drive_ticks[5])

    def kill_motors(self) -> None:
        """
        Stops drive motors and align corner motors
        """

        # Align corner motors
        self.corner_to_position([500, 500, 500, 500])
        # Stop drive motors
        self.send_motor_duty([0, 0, 0, 0, 0, 0])

    def get_corner_position(self, servo_id: int) -> int:
        return self.lx16a.get_position(servo_id)

    def get_motor_speed(self, servo_id: int) -> int:
        return self.lx16a.get_motor_speed(servo_id)

    def get_mode(self, servo_id: int) -> int:
        return self.lx16a.get_mode(servo_id)

    def get_status(self, servo_id: int) -> int:
        return self.lx16a.is_motor_on(servo_id)

    def move(self, servo_id: int, position: int, time: int = 1000) -> None:
        self.lx16a.move(servo_id, position, time)

    def led_turn_off(self, servo_id: int) -> None:
        self.lx16a.led_off(servo_id)

    def led_turn_on(self, servo_id: int) -> None:
        self.lx16a.led_on(servo_id)
