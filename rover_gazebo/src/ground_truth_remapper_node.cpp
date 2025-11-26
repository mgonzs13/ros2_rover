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

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_gazebo/ground_truth_remapper_node.hpp"

using std::placeholders::_1;

GroundTruthRemapperNode::GroundTruthRemapperNode()
    : rclcpp::Node("ground_truth_remapper"), initial_pose_set_(false),
      initial_x_(0.0), initial_y_(0.0), initial_z_(0.0), initial_yaw_(0.0) {

  this->pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odom_ground_truth", 10);
  this->sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_ground_truth_absolute", 10,
      std::bind(&GroundTruthRemapperNode::odom_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Ground truth remapper initialized");
  RCLCPP_INFO(this->get_logger(),
              "Waiting for first odometry message to set initial pose...");
}

void GroundTruthRemapperNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

  if (!this->initial_pose_set_) {
    this->initial_x_ = msg->pose.pose.position.x;
    this->initial_y_ = msg->pose.pose.position.y;
    this->initial_z_ = msg->pose.pose.position.z;
    this->initial_yaw_ = this->quaternion_to_yaw(msg->pose.pose.orientation);
    this->initial_pose_set_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Initial pose captured: x=%.3f, y=%.3f, z=%.3f, yaw=%.1f°",
                this->initial_x_, this->initial_y_, this->initial_z_,
                this->initial_yaw_ * 180.0 / M_PI);
  }

  nav_msgs::msg::Odometry relative;
  relative.header = msg->header;
  relative.child_frame_id = msg->child_frame_id;

  double dx = msg->pose.pose.position.x - this->initial_x_;
  double dy = msg->pose.pose.position.y - this->initial_y_;
  double dz = msg->pose.pose.position.z - this->initial_z_;

  double cos_yaw = std::cos(-this->initial_yaw_);
  double sin_yaw = std::sin(-this->initial_yaw_);
  relative.pose.pose.position.x = dx * cos_yaw - dy * sin_yaw;
  relative.pose.pose.position.y = dx * sin_yaw + dy * cos_yaw;
  relative.pose.pose.position.z = dz;

  double current_yaw = this->quaternion_to_yaw(msg->pose.pose.orientation);
  double relative_yaw = current_yaw - this->initial_yaw_;
  relative_yaw = std::atan2(std::sin(relative_yaw), std::cos(relative_yaw));
  relative.pose.pose.orientation = this->yaw_to_quaternion(relative_yaw);

  relative.pose.covariance = msg->pose.covariance;
  relative.twist = msg->twist;

  pub_->publish(relative);
}

double GroundTruthRemapperNode::quaternion_to_yaw(
    const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion
GroundTruthRemapperNode::yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthRemapperNode>());
  rclcpp::shutdown();
  return 0;
}
