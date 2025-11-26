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

#ifndef ODOMETRY_NODE_HPP
#define ODOMETRY_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdometryNode : public rclcpp::Node {
public:
  OdometryNode();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_odometry(const rclcpp::Time &current_time);
  static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);

  // Odometry integration methods
  void integrate_runge_kutta2(double linear, double angular);
  void integrate_exact(double linear, double angular);

  // Parameters
  double wheel_radius_;
  double wheel_separation_;
  bool publish_tf_;
  std::string odom_frame_;
  std::string base_frame_;

  // Odometry state
  double x_;
  double y_;
  double heading_;
  double linear_;
  double angular_;
  rclcpp::Time timestamp_;

  // Wheel position tracking
  double left_wheel_old_pos_;
  double right_wheel_old_pos_;
  bool initialized_;

  // Velocity smoothing (simple moving average)
  static constexpr size_t VELOCITY_WINDOW_SIZE = 10;
  std::vector<double> linear_velocity_samples_;
  std::vector<double> angular_velocity_samples_;

  // Helper methods
  double compute_rolling_mean(std::vector<double> &samples, double new_sample);

  // ROS Interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // ODOMETRY_NODE_HPP
