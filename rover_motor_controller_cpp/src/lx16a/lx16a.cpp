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

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "lx16a/lx16a.hpp"
#include "lx16a/lx16a_consts.hpp"
#include "lx16a/serial.hpp"

using namespace lx16a;

LX16A::LX16A(std::string serial_port, unsigned int baud_rate) {
  this->serial = std::make_unique<Serial>(Serial(serial_port, baud_rate));
  this->mtx = std::make_unique<std::mutex>();
  this->serial->connect();
}

// serial functions
void LX16A::send_command(uint8_t servo_id, uint8_t command,
                         std::vector<uint8_t> params) {

  unsigned length = 3 + params.size();
  uint8_t sum = 0;
  for (unsigned i = 0; i < params.size(); i++) {
    sum += params.at(i);
  }

  uint8_t checksum = 255 - ((servo_id + length + command + sum) % 256);

  // build msg to send
  std::vector<uint8_t> msg;
  msg.push_back(SERVO_FRAME_HEADER);
  msg.push_back(SERVO_FRAME_HEADER);
  msg.push_back(servo_id);
  msg.push_back(length);
  msg.push_back(command);

  for (unsigned i = 0; i < params.size(); i++) {
    msg.push_back(params.at(i));
  }

  msg.push_back(checksum);

  // send
  const std::lock_guard<std::mutex> lock(*this->mtx);
  this->serial->write(msg);
}

std::vector<uint8_t> LX16A::wait_for_response(uint8_t servo_id,
                                              uint8_t command) {
  while (true) {
    std::vector<uint8_t> data;
    unsigned char buffer;

    // header part 1
    this->serial->read(buffer);
    data.push_back(uint8_t(buffer));

    if (data.at(0) != SERVO_FRAME_HEADER) {
      continue;
    }

    // header part 2
    this->serial->read(buffer);
    data.push_back(uint8_t(buffer));

    if (data.at(1) != SERVO_FRAME_HEADER) {
      continue;
    }

    // id, length and command
    this->serial->read(buffer);
    data.push_back(uint8_t(buffer));
    uint8_t sid = data[2];

    this->serial->read(buffer);
    data.push_back(uint8_t(buffer));
    uint8_t length = data[3];

    this->serial->read(buffer);
    data.push_back(uint8_t(buffer));
    uint8_t cmd = data[4];

    if (length > 7) {
      std::cout << "Invalid length for packet\n";
      continue;
    }

    // params
    uint8_t sum = 0;
    for (uint8_t i = 5; i < length - 3 + 5; i++) {
      this->serial->read(buffer);
      data.push_back(uint8_t(buffer));
      sum += data.at(i);
    }

    // Checksum
    this->serial->read(buffer);
    data.push_back(uint8_t(buffer));
    uint8_t checksum = data.back();

    if ((255 - (sid + length + cmd + sum) % 256) != checksum) {
      std::cout << "Invalid checksum for packet\n";
      continue;
    }

    if (cmd != command) {
      std::cout << "Got unexpected command response\n";
      continue;
    }

    if (servo_id != SERVO_ID_ALL and sid != servo_id) {
      std::cout << "Got command response from unexpected servo\n";
      continue;
    }

    // RESULT
    std::vector<uint8_t> result;

    for (uint8_t i = 5; i < length - 3 + 5; i++) {
      result.push_back(data.at(i));
    }

    return result;
  }
}

std::vector<uint8_t> LX16A::query(uint8_t servo_id, uint8_t command) {
  this->send_command(servo_id, command, {});
  const std::lock_guard<std::mutex> lock(*this->mtx);
  return this->wait_for_response(servo_id, command);
}

// aux functions
uint8_t LX16A::lower_byte(int value) { return value % 256; }
uint8_t LX16A::higher_byte(int value) { return int(value / 256) % 256; }
uint8_t LX16A::word(int low, int high) { return low + high * 256; }
int LX16A::clamp(int range_min, int range_max, int value) {
  return std::min(range_max, std::max(range_min, value));
}

// servo funcitons
uint8_t LX16A::get_servo_id(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_ID_READ);
  return response[0];
}

void LX16A::set_servo_id(uint8_t servo_id, uint8_t new_servo_id) {
  this->send_command(servo_id, SERVO_ID_WRITE, {new_servo_id});
}

// move functions
void LX16A::move(uint8_t servo_id, int position, int time) {
  int aux_position = this->clamp(0, 1000, position);
  int aux_time = this->clamp(0, 30000, time);

  this->send_command(servo_id, SERVO_MOVE_TIME_WRITE,
                     {this->lower_byte(aux_position),
                      this->higher_byte(aux_position),
                      this->lower_byte(aux_time), this->higher_byte(aux_time)});
}

void LX16A::move_prepare(uint8_t servo_id, int position, int time) {
  int aux_position = this->clamp(0, 1000, position);
  int aux_time = this->clamp(0, 30000, time);

  this->send_command(servo_id, SERVO_MOVE_TIME_WAIT_WRITE,
                     {this->lower_byte(aux_position),
                      this->higher_byte(aux_position),
                      this->lower_byte(aux_time), this->higher_byte(aux_time)});
}

void LX16A::move_start(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_MOVE_START, {});
}

void LX16A::move_stop(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_MOVE_STOP, {});
}

std::vector<uint8_t> LX16A::get_prepared_move(uint8_t servo_id) {
  std::vector<uint8_t> response =
      this->query(servo_id, SERVO_MOVE_TIME_WAIT_READ);

  return {this->word(response[0], response[1]),
          this->word(response[2], response[3])};
}

// position functions
int LX16A::get_position_offset(uint8_t servo_id) {
  std::vector<uint8_t> response =
      this->query(servo_id, SERVO_ANGLE_OFFSET_READ);
  int deviation = response[0];

  if (deviation > 127) {
    deviation -= 256;
  }

  return deviation;
}

std::vector<uint8_t> LX16A::get_position_limits(int8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_ANGLE_LIMIT_READ);

  return {this->word(response[0], response[1]),
          this->word(response[2], response[3])};
}

int LX16A::get_position(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_POS_READ);
  int position = this->word(response[0], response[1]);

  if (position > 32767)
    position -= 65536;

  return position;
}

void LX16A::set_position_offset(uint8_t servo_id, int deviation) {
  int aux_deviation = this->clamp(-125, 125, deviation);
  uint8_t aux_deviation_1 = 0;

  if (aux_deviation < 0)
    aux_deviation += 256;

  aux_deviation_1 = aux_deviation;

  this->send_command(servo_id, SERVO_ANGLE_OFFSET_ADJUST, {aux_deviation_1});
}

void LX16A::save_position_offset(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_ANGLE_OFFSET_WRITE, {});
}

void LX16A::set_position_limits(uint8_t servo_id, int min_position,
                                int max_position) {
  int aux_min_position = this->clamp(0, 1000, min_position);
  int aux_max_position = this->clamp(0, 1000, max_position);

  this->send_command(servo_id, SERVO_ANGLE_LIMIT_WRITE,
                     {this->lower_byte(aux_min_position),
                      this->higher_byte(aux_min_position),
                      this->lower_byte(aux_max_position),
                      this->higher_byte(aux_max_position)});
}

// voltage functions
std::vector<uint8_t> LX16A::get_voltage_limits(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_VIN_LIMIT_READ);

  return {this->word(response[0], response[1]),
          this->word(response[2], response[3])};
}

uint8_t LX16A::get_voltage(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_VIN_READ);

  return word(response[0], response[1]);
}

void LX16A::set_voltage_limits(uint8_t servo_id, int min_voltage,
                               int max_voltage) {
  int aux_min_voltage = this->clamp(4500, 12000, min_voltage);
  int aux_max_voltage = this->clamp(4500, 12000, max_voltage);

  this->send_command(
      servo_id, SERVO_VIN_LIMIT_WRITE,
      {this->lower_byte(aux_min_voltage), this->higher_byte(aux_min_voltage),
       this->lower_byte(aux_max_voltage), this->higher_byte(aux_max_voltage)});
}

// temperature functions
uint8_t LX16A::get_max_temperature_limit(uint8_t servo_id) {
  std::vector<uint8_t> response =
      this->query(servo_id, SERVO_TEMP_MAX_LIMIT_READ);
  return response[0];
}

uint8_t LX16A::get_temperature(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_TEMP_READ);
  return response[0];
}

void LX16A::set_max_temperature_limit(uint8_t servo_id, int max_temperature) {
  uint8_t aux_max_temperature = this->clamp(50, 100, max_temperature);
  this->send_command(servo_id, SERVO_TEMP_MAX_LIMIT_WRITE,
                     {aux_max_temperature});
}

// mode functions
uint8_t LX16A::get_mode(uint8_t servo_id) {
  std::vector<uint8_t> response =
      this->query(servo_id, SERVO_OR_MOTOR_MODE_READ);
  return response[0];
}

int LX16A::get_motor_speed(uint8_t servo_id) {
  std::vector<uint8_t> response =
      this->query(servo_id, SERVO_OR_MOTOR_MODE_READ);

  if (response[0] != 1)
    return 0;

  int speed = this->word(response[2], response[3]);

  if (speed > 32767)
    speed -= 65536;

  return speed;
}

void LX16A::set_servo_mode(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, {0, 0, 0, 0});
}

void LX16A::set_motor_mode(uint8_t servo_id, int speed) {
  int aux_speed = this->clamp(-1000, 1000, speed);

  if (aux_speed < 0)
    aux_speed += 65536;

  this->send_command(
      servo_id, SERVO_OR_MOTOR_MODE_WRITE,
      {1, 0, this->lower_byte(aux_speed), this->higher_byte(aux_speed)});
}

bool LX16A::is_motor_on(uint8_t servo_id) {
  std::vector<uint8_t> response =
      this->query(servo_id, SERVO_LOAD_OR_UNLOAD_READ);
  return response[0] == 1;
}

void LX16A::motor_on(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, {1});
}

void LX16A::motor_off(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, {0});
}

// LEDS functions
bool LX16A::is_led_on(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_LED_CTRL_READ);
  return response[0] == 0;
}

void LX16A::led_on(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_LED_CTRL_WRITE, {0});
}

void LX16A::led_off(uint8_t servo_id) {
  this->send_command(servo_id, SERVO_LED_CTRL_WRITE, {1});
}

uint8_t LX16A::get_led_errors(uint8_t servo_id) {
  std::vector<uint8_t> response = this->query(servo_id, SERVO_LED_ERROR_READ);
  return response[0];
}

void LX16A::set_led_errors(uint8_t servo_id, uint8_t error) {
  uint8_t aux_error = this->clamp(0, 7, error);
  this->send_command(servo_id, SERVO_LED_ERROR_WRITE, {aux_error});
}
