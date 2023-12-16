// MIT License

// Copyright (c) 2023  Miguel Ángel González Santamarta

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

#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_msgs/msg/motors_command.hpp"

#define MAX_RADIUS 255
#define MIN_RADIUS 55

namespace motor_controller {

class VelParserNode : public rclcpp::Node {
public:
  VelParserNode();
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  float normalize(float value, float old_min, float old_max, float new_min,
                  float new_max);
  float deg_to_tick(float deg, float e_min, float e_max);
  float radians_to_deg(float radians);

  std::vector<float> calculate_velocity(float velocity, float radius);
  std::vector<float> calculate_target_deg(float radius);
  std::vector<float> calculate_target_tick(std::vector<float> target_angles);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
  rclcpp::Publisher<rover_msgs::msg::MotorsCommand>::SharedPtr publisher;
  int speed_factor;
  float d1;
  float d2;
  float d3;
  float d4;
  int enc_min;
  int enc_max;
  float enc_mid;
  float linear_limit;
  float angular_limit;
  float angular_factor;
};

} // namespace motor_controller
#endif
