
"""
Lewansoul wrapper.
"""


import serial

from .lewansoul import ServoController

MOTOR_LEFT_FRONT = 1
MOTOR_LEFT_MIDDLE = 2
MOTOR_LEFT_BACK = 3
MOTOR_RIGHT_FRONT = 4
MOTOR_RIGHT_MIDDLE = 5
MOTOR_RIGHT_BACK = 6

SERVO_LEFT_FRONT = 7
SERVO_RIGHT_FRONT = 8
SERVO_LEFT_BACK = 9
SERVO_RIGHT_BACK = 10


class MotorController(object):
    """
    MotorController class contains the methods necessary to send commands to
    the motor controllers for the corner and drive motors.
    """

    def __init__(self, serial_port: str, baud_rate: int):

        self.lw_controller = ServoController(
            serial.Serial(serial_port, baud_rate, timeout=1))

        drive_ticks = [0, 0, 0, 0, 0, 0]

        self.lw_controller.set_motor_mode(MOTOR_LEFT_FRONT, drive_ticks[0])
        self.lw_controller.set_motor_mode(MOTOR_LEFT_MIDDLE, drive_ticks[1])
        self.lw_controller.set_motor_mode(MOTOR_LEFT_BACK, drive_ticks[2])
        self.lw_controller.set_motor_mode(MOTOR_RIGHT_FRONT, drive_ticks[3])
        self.lw_controller.set_motor_mode(MOTOR_RIGHT_MIDDLE, drive_ticks[4])
        self.lw_controller.set_motor_mode(MOTOR_RIGHT_BACK, drive_ticks[5])
        self.lw_controller.set_servo_mode(SERVO_LEFT_FRONT)
        self.lw_controller.set_servo_mode(SERVO_RIGHT_FRONT)
        self.lw_controller.set_servo_mode(SERVO_LEFT_BACK)
        self.lw_controller.set_servo_mode(SERVO_RIGHT_BACK)

    def corner_to_position(self, corner_ticks):
        """
        Method to send position commands to the corner motors

        :param list corner_ticks: A list of ticks for each of the corner motors to move to
        """

        servo_front_left = self.lw_controller.servo(SERVO_LEFT_FRONT)
        servo_front_right = self.lw_controller.servo(SERVO_RIGHT_FRONT)
        servo_back_left = self.lw_controller.servo(SERVO_LEFT_BACK)
        servo_back_right = self.lw_controller.servo(SERVO_RIGHT_BACK)

        servo_front_left.move_prepare(corner_ticks[0])
        servo_front_right.move_prepare(corner_ticks[1])
        servo_back_left.move_prepare(corner_ticks[2])
        servo_back_right.move_prepare(corner_ticks[3])

        self.lw_controller.move_start()

    def send_motor_duty(self, drive_ticks):
        """
        Method to send position commands to the drive motors

        :param list drive_ticks: A list of ticks for each of the drice motors to speed to
        """

        self.lw_controller.set_motor_mode(MOTOR_LEFT_FRONT, drive_ticks[0])
        self.lw_controller.set_motor_mode(MOTOR_LEFT_MIDDLE, drive_ticks[1])
        self.lw_controller.set_motor_mode(MOTOR_LEFT_BACK, drive_ticks[2])
        self.lw_controller.set_motor_mode(MOTOR_RIGHT_FRONT, drive_ticks[3])
        self.lw_controller.set_motor_mode(MOTOR_RIGHT_MIDDLE, drive_ticks[4])
        self.lw_controller.set_motor_mode(MOTOR_RIGHT_BACK, drive_ticks[5])

    def kill_motors(self):
        """
        Stops drive motors and align corner motors
        """

        # Align corner motors
        self.corner_to_position([500, 500, 500, 500])
        # Stop drive motors
        self.send_motor_duty([0, 0, 0, 0, 0, 0])

    def get_corner_position(self, servo_id):
        return self.lw_controller.get_position(servo_id)

    def get_motor_speed(self, servo_id):
        return self.lw_controller.get_motor_speed(servo_id)

    def get_mode(self, servo_id):
        return self.lw_controller.get_mode(servo_id)

    def get_status(self, servo_id):
        return self.lw_controller.is_motor_on(servo_id)

    def move(self, servo_id, position, time=1000):
        self.lw_controller.move(servo_id, position, time)

    def led_turn_off(self, servo_id):
        self.lw_controller.led_off(servo_id)

    def led_turn_on(self, servo_id):
        self.lw_controller.led_on(servo_id)
