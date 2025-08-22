// MIT License

// Copyright (c) 2023 Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

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

  // Check if the device exists and is accessible
  struct stat device_stat;
  if (stat(motor_controller_device.c_str(), &device_stat) != 0) {
    std::string error_msg = "Motor controller device '" +
                            motor_controller_device +
                            "' does not exist. Please check if the device is "
                            "connected and the path is correct.";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Check if we have read/write permissions
  if (access(motor_controller_device.c_str(), R_OK | W_OK) != 0) {
    std::string error_msg =
        "No read/write permission for motor controller device '" +
        motor_controller_device +
        "'. Please check device permissions or add user to dialout group.";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Check if it's a character device (serial devices are character devices)
  if (!S_ISCHR(device_stat.st_mode)) {
    std::string error_msg =
        "Motor controller device '" + motor_controller_device +
        "' is not a character device. Expected a serial device.";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  RCLCPP_INFO(this->get_logger(),
              "Motor controller device '%s' found and accessible",
              motor_controller_device.c_str());

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