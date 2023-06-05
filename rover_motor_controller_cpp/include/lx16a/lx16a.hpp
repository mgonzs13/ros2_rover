// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef LX16A_HPP
#define LX16A_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "lx16a/lx16a_consts.hpp"
#include "lx16a/serial.hpp"

namespace lx16a {

class LX16A {
private:
  std::unique_ptr<Serial> serial;
  std::unique_ptr<std::mutex> mtx;

  void send_command(uint8_t servo_id, uint8_t command,
                    std::vector<uint8_t> params);
  std::vector<uint8_t> wait_for_response(uint8_t servo_id, uint8_t command);
  std::vector<uint8_t> query(uint8_t servo_id, uint8_t command);

public:
  LX16A(std::string serial_port, unsigned int baud_rate);

  uint8_t lower_byte(int value);
  uint8_t higher_byte(int value);
  uint8_t word(int low, int high);
  int clamp(int range_min, int range_max, int value);

  uint8_t get_servo_id(uint8_t servo_id);
  void set_servo_id(uint8_t servo_id, uint8_t new_servo_id);

  void move(uint8_t servo_id, int position, int time = 0);
  void move_prepare(uint8_t servo_id, int position, int time = 0);
  void move_start(uint8_t servo_id = SERVO_ID_ALL);
  void move_stop(uint8_t servo_id = SERVO_ID_ALL);

  std::vector<uint8_t> get_prepared_move(uint8_t servo_id);
  int get_position_offset(uint8_t servo_id);
  std::vector<uint8_t> get_position_limits(int8_t servo_id);
  int get_position(uint8_t servo_id);

  void set_position_offset(uint8_t servo_id, int deviation);
  void save_position_offset(uint8_t servo_id);
  void set_position_limits(uint8_t servo_id, int min_position,
                           int max_position);

  std::vector<uint8_t> get_voltage_limits(uint8_t servo_id);
  uint8_t get_voltage(uint8_t servo_id);
  void set_voltage_limits(uint8_t servo_id, int min_voltage, int max_voltage);

  uint8_t get_max_temperature_limit(uint8_t servo_id);
  uint8_t get_temperature(uint8_t servo_id);
  void set_max_temperature_limit(uint8_t servo_id, int max_temperature);

  uint8_t get_mode(uint8_t servo_id);
  int get_motor_speed(uint8_t servo_id);
  void set_servo_mode(uint8_t servo_id);
  void set_motor_mode(uint8_t servo_id, int speed = 0);
  bool is_motor_on(uint8_t servo_id);
  void motor_on(uint8_t servo_id);
  void motor_off(uint8_t servo_id);

  bool is_led_on(uint8_t servo_id);
  void led_on(uint8_t servo_id);
  void led_off(uint8_t servo_id);
  uint8_t get_led_errors(uint8_t servo_id);
  void set_led_errors(uint8_t servo_id, uint8_t error);
};

} // namespace lx16a
#endif
