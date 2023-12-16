// MIT License

// Copyright (c) 2023  Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "lx16a/lx16a.hpp"
#include "lx16a/lx16a_consts.hpp"

namespace lx16a {

class MotorController {
private:
  std::unique_ptr<LX16A> lx16a;

public:
  MotorController(std::string serial_port, unsigned int baud_rate);

  void corner_to_position(std::vector<int> corner_ticks);
  void send_motor_duty(std::vector<int> drive_ticks);

  void kill_motors();

  int get_corner_position(uint8_t servo_id);
  int get_motor_speed(uint8_t servo_id);

  uint8_t get_mode(uint8_t servo_id);
  uint8_t get_status(uint8_t servo_id);

  void move(uint8_t servo_id, int position, int time);

  void led_turn_off(uint8_t servo_id);
  void led_turn_on(uint8_t servo_id);
};

} // namespace lx16a
#endif
