
#ifndef LX16A_HPP
#define LX16A_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

  void move(uint8_t servo_id, int position, int time);
  void move_prepare(uint8_t servo_id, int position, int time);
  void move_start(uint8_t servo_id);
  void move_stop(uint8_t servo_id);

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
  void set_motor_mode(uint8_t servo_id, int speed);
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
