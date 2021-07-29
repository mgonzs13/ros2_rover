
#include <memory>

#include "motor_controller/vel_parser_node.hpp"

using namespace motor_controller;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelParserNode>());
  rclcpp::shutdown();
  return 0;
}
