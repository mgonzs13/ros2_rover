

#ifndef SERVO_INFO_HPP
#define SERVO_INFO_HPP

#include <cstdint>
#include <vector>

namespace lx16a {

struct ServoInfo {
  uint8_t id;
  uint8_t cmd;
  std::vector<uint8_t> params;
};

} // namespace lx16a
#endif
