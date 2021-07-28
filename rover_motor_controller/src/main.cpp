#include "lx16a/motor_controller.hpp"

int main(int argc, char *argv[]) {

  lx16a::MotorController controller("/dev/ttyUSB0", 115200);
  controller.corner_to_position({300, 300, 300, 300});

  return 0;
}
