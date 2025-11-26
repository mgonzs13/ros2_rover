// MIT License

// Copyright (c) 2025 Miguel Ángel González Santamarta

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

#ifndef GROUND_TRUTH_REMAPPER_NODE_HPP
#define GROUND_TRUTH_REMAPPER_NODE_HPP

#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class GroundTruthRemapperNode : public rclcpp::Node {
public:
  GroundTruthRemapperNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q);
  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

  bool initial_pose_set_;
  double initial_x_;
  double initial_y_;
  double initial_z_;
  double initial_yaw_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

#endif // GROUND_TRUTH_REMAPPER_NODE_HPP
