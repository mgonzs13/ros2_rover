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

#ifndef MOTORS_COMMANDS_NODE_HPP
#define MOTORS_COMMANDS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rover_msgs/msg/motors_command.hpp"

class MotorsCommandParserNode : public rclcpp::Node {
public:
  MotorsCommandParserNode();
  void callback(const rover_msgs::msg::MotorsCommand::SharedPtr msg);
  int clamp(int range_min, int range_max, int value);
  float normalize(int range_min, int range_max, int value);

private:
  rclcpp::Subscription<rover_msgs::msg::MotorsCommand>::SharedPtr subscription;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      velocity_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      position_publisher;
};

#endif