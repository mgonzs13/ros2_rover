
"""
Lewansoul interface.
"""

import logging
import threading
from typing import List

from serial import Serial
from serial.serialutil import Timeout

from .lx16a_consts import *


####
# aux functions
# ####
def lower_byte(value: int) -> int:
    return int(value) % 256


def higher_byte(value: int):
    return int(value / 256) % 256


def word(low: int, high: int) -> int:
    return int(low) + int(high) * 256


def clamp(range_min: int, range_max: int, value: int) -> int:
    return min(range_max, max(range_min, value))


class TimeoutError(RuntimeError):
    """
    TimeoutError class.
    """

    pass


LOGGER = logging.getLogger("lx16a")


class LX16A(object):
    """
    LX16A class.
    """

    def __init__(self, serial_port: str, baud_rate: int, timeout: int = 1):
        self._serial = Serial(serial_port, baud_rate, timeout=1)
        self._timeout = timeout
        self._lock = threading.RLock()

    ####
    # serial functions
    ####
    def _command(self, servo_id: int, command: int, params: int) -> None:
        if isinstance(params, list):
            length = 3 + len(params)
            checksum = 255 - \
                ((servo_id + length + command + sum(params)) % 256)

            command_list = []
            command_list = [SERVO_FRAME_HEADER,
                            SERVO_FRAME_HEADER, servo_id, length, command]
            command_list += params
            command_list += [checksum]
        else:
            length = 3 + 1
            checksum = 255 - ((servo_id + length + command + params) % 256)

            command_list = []
            command_list = [SERVO_FRAME_HEADER,
                            SERVO_FRAME_HEADER, servo_id, length, command]
            command_list += [params]
            command_list += [checksum]

        LOGGER.debug("Sending servo control packet: %s", command_list)

        with self._lock:
            self._serial.write(bytearray(command_list))

    def _wait_for_response(self, servo_id: int, command: int, timeout: int = None) -> List[int]:
        timeout = Timeout(timeout or self._timeout)

        def read(size=1):
            self._serial.timeout = timeout.time_left()
            data = self._serial.read(size)

            if len(data) != size:
                raise TimeoutError()

            return data

        while True:
            data = []
            # header part 1
            data += read(1)

            if hex(ord(data[-1])) != SERVO_FRAME_HEADER_STRING:
                continue

            # header part 2
            data += read(1)

            if hex(ord(data[-1])) != SERVO_FRAME_HEADER_STRING:
                continue

            # id, length and command
            data += read(3)
            # id
            sid = ord(data[2])
            # length
            length = ord(data[3])
            # command
            cmd = ord(data[4])

            if length > 7:
                LOGGER.error("Invalid length for packet %s", list(data))
                continue

            # params
            data += read(length - 3) if length > 3 else []
            params_tmp = data[5:]

            params = []
            for valor in params_tmp:
                params.append(ord(valor))

            # Checksum
            data += read(1)
            checksum = ord(data[-1])

            if 255 - (sid + length + cmd + sum(params)) % 256 != checksum:
                LOGGER.error("Invalid checksum for packet %s", list(data))
                continue

            if cmd != command:
                LOGGER.warning(
                    "Got unexpected command %s response %s", cmd, list(data))
                continue

            if servo_id != SERVO_ID_ALL and sid != servo_id:
                LOGGER.warning(
                    "Got command response from unexpected servo %s", sid)
                continue

            return params

    def _query(self, servo_id: int, command: int, timeout: int = None) -> List[int]:
        with self._lock:
            self._command(servo_id, command, [])

            return self._wait_for_response(servo_id, command, timeout=timeout)

    ####
    # servo functions
    ####
    def get_servo_id(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(servo_id, SERVO_ID_READ, timeout=timeout)

        return response[0]

    def set_servo_id(self, servo_id: int, new_servo_id: int) -> None:
        self._command(servo_id, SERVO_ID_WRITE, new_servo_id)

    ####
    # move functions
    ####
    def move(self, servo_id: int, position: int, time: int = 0) -> None:
        position = clamp(0, 1000, position)
        time = clamp(0, 30000, time)

        self._command(
            servo_id, SERVO_MOVE_TIME_WRITE,
            [lower_byte(position), higher_byte(position),
             lower_byte(time), higher_byte(time)]
        )

    def move_prepare(self, servo_id: int, position: int, time: int = 0) -> None:
        position = clamp(0, 1000, position)
        time = clamp(0, 30000, time)

        self._command(
            servo_id, SERVO_MOVE_TIME_WAIT_WRITE,
            [lower_byte(position), higher_byte(position),
             lower_byte(time), higher_byte(time)]
        )

    def move_start(self, servo_id: int = SERVO_ID_ALL) -> None:
        self._command(servo_id, SERVO_MOVE_START, [])

    def move_stop(self, servo_id: int = SERVO_ID_ALL) -> None:
        self._command(servo_id, SERVO_MOVE_STOP, [])

    def get_prepared_move(self, servo_id: int, timeout: int = None) -> List[int]:
        """ Returns servo position and time tuple """
        response = self._query(
            servo_id, SERVO_MOVE_TIME_WAIT_READ, timeout=timeout)
        return word(response[0], response[1]), word(response[2], response[3])

    ####
    # position functions
    ####
    def get_position_offset(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(
            servo_id, SERVO_ANGLE_OFFSET_READ, timeout=timeout)
        deviation = response[0]

        if deviation > 127:
            deviation -= 256

        return deviation

    def get_position_limits(self, servo_id: int, timeout: int = None) -> List[int]:
        response = self._query(
            servo_id, SERVO_ANGLE_LIMIT_READ, timeout=timeout)
        return word(response[0], response[1]), word(response[2], response[3])

    def get_position(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(servo_id, SERVO_POS_READ, timeout=timeout)
        position = word(response[0], response[1])

        if position > 32767:
            position -= 65536

        return position

    def set_position_offset(self, servo_id: int, deviation: int) -> None:
        deviation = clamp(-125, 125, deviation)

        if deviation < 0:
            deviation += 256

        self._command(servo_id, SERVO_ANGLE_OFFSET_ADJUST, deviation)

    def save_position_offset(self, servo_id):
        self._command(servo_id, SERVO_ANGLE_OFFSET_WRITE, [])

    def set_position_limits(self, servo_id: int, min_position: int, max_position: int) -> None:
        min_position = clamp(0, 1000, min_position)
        max_position = clamp(0, 1000, max_position)

        self._command(
            servo_id, SERVO_ANGLE_LIMIT_WRITE,
            [lower_byte(min_position), higher_byte(min_position),
             lower_byte(max_position), higher_byte(max_position)]
        )

    ####
    # voltage functions
    ####
    def get_voltage_limits(self, servo_id: int, timeout: int = None) -> List[int]:
        response = self._query(servo_id, SERVO_VIN_LIMIT_READ, timeout=timeout)
        return word(response[0], response[1]), word(response[2], response[3])

    def get_voltage(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(servo_id, SERVO_VIN_READ, timeout=timeout)
        return word(response[0], response[1])

    def set_voltage_limits(self, servo_id: int, min_voltage: int, max_voltage: int) -> None:
        min_voltage = clamp(4500, 12000, min_voltage)
        max_voltage = clamp(4500, 12000, max_voltage)

        self._command(
            servo_id, SERVO_VIN_LIMIT_WRITE,
            [lower_byte(min_voltage), higher_byte(min_voltage),
             lower_byte(max_voltage), higher_byte(max_voltage)]
        )

    ####
    # voltage functions
    ####
    def get_max_temperature_limit(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(
            servo_id, SERVO_TEMP_MAX_LIMIT_READ, timeout=timeout)
        return response[0]

    def get_temperature(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(servo_id, SERVO_TEMP_READ, timeout=timeout)
        return response[0]

    def set_max_temperature_limit(self, servo_id: int, max_temperature: int) -> None:
        max_temperature = clamp(50, 100, max_temperature)
        self._command(servo_id, SERVO_TEMP_MAX_LIMIT_WRITE, max_temperature)

    ####
    # mode functions
    ####
    def get_mode(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(
            servo_id, SERVO_OR_MOTOR_MODE_READ, timeout=timeout)

        return response[0]

    def get_motor_speed(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(
            servo_id, SERVO_OR_MOTOR_MODE_READ, timeout=timeout)

        if response[2][0] != 1:
            return 0

        speed = word(response[2], response[3])

        if speed > 32767:
            speed -= 65536

        return speed

    def set_servo_mode(self, servo_id: int) -> None:
        self._command(
            servo_id, SERVO_OR_MOTOR_MODE_WRITE,
            [0, 0, 0, 0]
        )

    def set_motor_mode(self, servo_id: int, speed: int = 0) -> None:
        speed = clamp(-1000, 1000, speed)

        if speed < 0:
            speed += 65536

        self._command(
            servo_id, SERVO_OR_MOTOR_MODE_WRITE,
            [1, 0, lower_byte(speed), higher_byte(speed)]
        )

    def is_motor_on(self, servo_id: int, timeout: int = None) -> bool:
        response = self._query(
            servo_id, SERVO_LOAD_OR_UNLOAD_READ, timeout=timeout)
        return response[0] == 1

    def motor_on(self, servo_id: int) -> None:
        self._command(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, 1)

    def motor_off(self, servo_id: int) -> None:
        self._command(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, 0)

    ####
    # LEDS functions
    ####
    def is_led_on(self, servo_id: int, timeout: int = None) -> bool:
        response = self._query(servo_id, SERVO_LED_CTRL_READ, timeout=timeout)
        return response[0] == 0

    def led_on(self, servo_id: int) -> None:
        self._command(servo_id, SERVO_LED_CTRL_WRITE, 0)

    def led_off(self, servo_id: int) -> None:
        self._command(servo_id, SERVO_LED_CTRL_WRITE, 1)

    def get_led_errors(self, servo_id: int, timeout: int = None) -> int:
        response = self._query(servo_id, SERVO_LED_ERROR_READ, timeout=timeout)
        return response[0]

    def set_led_errors(self, servo_id: int, error: int) -> None:
        error = clamp(0, 7, error)
        self._command(servo_id, SERVO_LED_ERROR_WRITE, error)
