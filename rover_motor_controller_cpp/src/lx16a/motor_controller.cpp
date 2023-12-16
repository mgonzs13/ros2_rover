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

#include <memory>
#include <vector>

#include "lx16a/lx16a.hpp"
#include "lx16a/lx16a_consts.hpp"
#include "lx16a/motor_controller.hpp"

using namespace lx16a;

MotorController::MotorController(std::string serial_port,
                                 unsigned int baud_rate) {

  this->lx16a = std::make_unique<LX16A>(LX16A(serial_port, baud_rate));

  this->lx16a->set_motor_mode(MOTOR_LEFT_FRONT, 0);
  this->lx16a->set_motor_mode(MOTOR_LEFT_MIDDLE, 0);
  this->lx16a->set_motor_mode(MOTOR_LEFT_BACK, 0);
  this->lx16a->set_motor_mode(MOTOR_RIGHT_FRONT, 0);
  this->lx16a->set_motor_mode(MOTOR_RIGHT_MIDDLE, 0);
  this->lx16a->set_motor_mode(MOTOR_RIGHT_BACK, 0);
  this->lx16a->set_servo_mode(SERVO_LEFT_FRONT);
  this->lx16a->set_servo_mode(SERVO_RIGHT_FRONT);
  this->lx16a->set_servo_mode(SERVO_LEFT_BACK);
  this->lx16a->set_servo_mode(SERVO_RIGHT_BACK);
}

void MotorController::corner_to_position(std::vector<int> corner_ticks) {
  this->lx16a->move_prepare(SERVO_LEFT_FRONT, corner_ticks[0], 0);
  this->lx16a->move_prepare(SERVO_RIGHT_FRONT, corner_ticks[1], 0);
  this->lx16a->move_prepare(SERVO_LEFT_BACK, corner_ticks[2], 0);
  this->lx16a->move_prepare(SERVO_RIGHT_BACK, corner_ticks[3], 0);
  this->lx16a->move_start(SERVO_ID_ALL);
}

void MotorController::send_motor_duty(std::vector<int> drive_ticks) {
  this->lx16a->set_motor_mode(MOTOR_LEFT_FRONT, drive_ticks[0]);
  this->lx16a->set_motor_mode(MOTOR_LEFT_MIDDLE, drive_ticks[1]);
  this->lx16a->set_motor_mode(MOTOR_LEFT_BACK, drive_ticks[2]);
  this->lx16a->set_motor_mode(MOTOR_RIGHT_FRONT, drive_ticks[3]);
  this->lx16a->set_motor_mode(MOTOR_RIGHT_MIDDLE, drive_ticks[4]);
  this->lx16a->set_motor_mode(MOTOR_RIGHT_BACK, drive_ticks[5]);
}

void MotorController::kill_motors() {
  this->corner_to_position({500, 500, 500, 500});
  this->send_motor_duty({0, 0, 0, 0, 0, 0});
}

int MotorController::get_corner_position(uint8_t servo_id) {
  return this->lx16a->get_position(servo_id);
}

int MotorController::get_motor_speed(uint8_t servo_id) {
  return this->lx16a->get_motor_speed(servo_id);
}

uint8_t MotorController::get_mode(uint8_t servo_id) {
  return this->lx16a->get_mode(servo_id);
}

uint8_t MotorController::get_status(uint8_t servo_id) {
  return this->lx16a->is_motor_on(servo_id);
}

void MotorController::move(uint8_t servo_id, int position, int time) {
  this->lx16a->move(servo_id, position, time);
}

void MotorController::led_turn_off(uint8_t servo_id) {
  this->lx16a->led_off(servo_id);
}

void MotorController::led_turn_on(uint8_t servo_id) {
  this->lx16a->led_on(servo_id);
}
