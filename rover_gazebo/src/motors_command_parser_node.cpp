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

#define BOOST_BIND_NO_PLACEHOLDERS

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rover_gazebo/motors_command_parser_node.hpp"
#include "rover_msgs/msg/motors_command.hpp"

using std::placeholders::_1;

MotorsCommandParserNode::MotorsCommandParserNode()
    : rclcpp::Node("motors_command_parser_node") {

  this->velocity_publisher =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "velocity_controller/commands", 10);

  this->position_publisher =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "position_controller/commands", 10);

  this->subscription =
      this->create_subscription<rover_msgs::msg::MotorsCommand>(
          "motors_command", 10,
          std::bind(&MotorsCommandParserNode::callback, this, _1));
}

void MotorsCommandParserNode::callback(
    const rover_msgs::msg::MotorsCommand::SharedPtr msg) {

  // RCLCPP_INFO(this->get_logger(),
  //             "Speed: %d %d %d %d %d %d, Sterring: %d, %d, %d, %d",
  //             msg->drive_motor[0], msg->drive_motor[1], msg->drive_motor[2],
  //             msg->drive_motor[3], msg->drive_motor[4], msg->drive_motor[5],
  //             msg->corner_motor[0], msg->corner_motor[1],
  //             msg->corner_motor[2], msg->corner_motor[3]);

  // create msgs
  auto corners_positions = std_msgs::msg::Float64MultiArray();
  auto wheel_velocities = std_msgs::msg::Float64MultiArray();

  float pi = atan(1) * 4;

  for (long unsigned int i = 0; i < msg->drive_motor.size(); i++) {

    float value =
        this->normalize(-1000, 1000,
                        this->clamp(-1000, 1000, msg->drive_motor[i])) *
        20;

    if (i > 2) {
      value *= -1;
    }

    wheel_velocities.data.push_back(value);
  }

  for (int position : msg->corner_motor) {

    corners_positions.data.push_back(
        this->normalize(250, 750, this->clamp(-250, 750, position)) * -pi / 2);
  }

  // publish
  this->position_publisher->publish(corners_positions);
  this->velocity_publisher->publish(wheel_velocities);
}

int MotorsCommandParserNode::clamp(int range_min, int range_max, int value) {
  return std::min(range_max, std::max(range_min, value));
}

float MotorsCommandParserNode::normalize(int range_min, int range_max,
                                         int value) {
  return 2 * ((float)value - range_min) / (range_max - range_min) - 1;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorsCommandParserNode>());
  rclcpp::shutdown();
  return 0;
}
