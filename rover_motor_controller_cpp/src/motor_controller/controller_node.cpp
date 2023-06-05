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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "lx16a/motor_controller.hpp"
#include "motor_controller/controller_node.hpp"
#include "rover_msgs/msg/motors_command.hpp"

using std::placeholders::_1;
using namespace motor_controller;

ControllerNode::ControllerNode() : rclcpp::Node("controller_node") {

  // declaring params
  this->declare_parameter<std::string>("motor_controller_device",
                                       "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);

  // getting params
  std::string motor_controller_device;
  this->get_parameter("motor_controller_device", motor_controller_device);

  int baud_rate;
  this->get_parameter("baud_rate", baud_rate);

  this->motor_controller = std::make_unique<lx16a::MotorController>(
      lx16a::MotorController(motor_controller_device, baud_rate));

  // sub
  this->subscription =
      this->create_subscription<rover_msgs::msg::MotorsCommand>(
          "motors_command", 10, std::bind(&ControllerNode::callback, this, _1));
}

void ControllerNode::callback(
    const rover_msgs::msg::MotorsCommand::SharedPtr msg) {
  this->motor_controller->corner_to_position(msg->corner_motor);
  this->motor_controller->send_motor_duty(msg->drive_motor);
}

void ControllerNode::shutdown() { this->motor_controller->kill_motors(); }