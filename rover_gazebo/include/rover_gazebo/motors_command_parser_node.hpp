
#ifndef MOTORS_COMMANDS_NODE_HPP
#define MOTORS_COMMANDS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rover_interfaces/msg/motors_command.hpp"

class MotorsCommandParserNode : public rclcpp::Node {
public:
  MotorsCommandParserNode();
  void callback(const rover_interfaces::msg::MotorsCommand::SharedPtr msg);
  int clamp(int range_min, int range_max, int value);
  float normalize(int range_min, int range_max, int value);

private:
  rclcpp::Subscription<rover_interfaces::msg::MotorsCommand>::SharedPtr
      subscription;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      velocity_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      position_publisher;
};

#endif