
#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lx16a/motor_controller.hpp"
#include "rover_msgs/msg/motors_command.hpp"

namespace motor_controller {

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  void callback(const rover_msgs::msg::MotorsCommand::SharedPtr msg);
  void shutdown();

private:
  std::unique_ptr<lx16a::MotorController> motor_controller;
  rclcpp::Subscription<rover_msgs::msg::MotorsCommand>::SharedPtr
      subscription;
};

} // namespace motor_controller
#endif
