
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
