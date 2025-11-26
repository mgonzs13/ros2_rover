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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "rover_gazebo/odometry_node.hpp"

using std::placeholders::_1;

OdometryNode::OdometryNode() : rclcpp::Node("odometry_node") {
  // Declare parameters
  this->wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.06);
  this->wheel_separation_ =
      this->declare_parameter<double>("wheel_separation", 0.5458);
  this->publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
  this->odom_frame_ =
      this->declare_parameter<std::string>("odom_frame", "odom");
  this->base_frame_ =
      this->declare_parameter<std::string>("base_frame", "base_link");

  // Initialize odometry state
  this->x_ = 0.0;
  this->y_ = 0.0;
  this->heading_ = 0.0;
  this->linear_ = 0.0;
  this->angular_ = 0.0;
  this->timestamp_ = this->now();

  // Initialize wheel tracking
  this->left_wheel_old_pos_ = 0.0;
  this->right_wheel_old_pos_ = 0.0;
  this->initialized_ = false;

  // Reserve space for velocity samples
  this->linear_velocity_samples_.reserve(VELOCITY_WINDOW_SIZE);
  this->angular_velocity_samples_.reserve(VELOCITY_WINDOW_SIZE);

  // Create publisher and subscriber
  this->odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

  if (this->publish_tf_) {
    this->tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  this->joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&OdometryNode::joint_state_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Odometry node initialized");
  RCLCPP_INFO(this->get_logger(),
              "Parameters: wheel_radius=%.3f, wheel_separation=%.3f",
              this->wheel_radius_, this->wheel_separation_);
  RCLCPP_INFO(this->get_logger(), "Frames: %s -> %s, publish_tf=%s",
              this->odom_frame_.c_str(), this->base_frame_.c_str(),
              this->publish_tf_ ? "true" : "false");
}

void OdometryNode::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {

  rclcpp::Time current_time = msg->header.stamp;

  // Find mid wheel positions
  double left_wheel_pos = 0.0;
  double right_wheel_pos = 0.0;
  bool found_left = false;
  bool found_right = false;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "mid_left_wheel_joint" && i < msg->position.size()) {
      left_wheel_pos = msg->position[i];
      found_left = true;
    } else if (msg->name[i] == "mid_right_wheel_joint" &&
               i < msg->position.size()) {
      right_wheel_pos = msg->position[i];
      found_right = true;
    }
  }

  if (!found_left || !found_right) {
    return;
  }

  // Initialize on first callback
  if (!this->initialized_) {
    // Store initial positions in linear distance units
    this->left_wheel_old_pos_ = left_wheel_pos * this->wheel_radius_;
    this->right_wheel_old_pos_ = right_wheel_pos * this->wheel_radius_;
    this->timestamp_ = current_time;
    this->initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Odometry initialized");
    return;
  }

  // Get current wheel joint positions (convert from radians to linear distance)
  double left_wheel_cur_pos = left_wheel_pos * this->wheel_radius_;
  double right_wheel_cur_pos = right_wheel_pos * this->wheel_radius_;

  // Estimate velocity of wheels using old and current position
  double left_wheel_est_vel = left_wheel_cur_pos - this->left_wheel_old_pos_;
  double right_wheel_est_vel = right_wheel_cur_pos - this->right_wheel_old_pos_;

  // Sanity check: if delta is too large, it might be a bad measurement
  constexpr double MAX_WHEEL_DELTA = 1.0; // 1 meter per update seems excessive
  if (std::fabs(left_wheel_est_vel) > MAX_WHEEL_DELTA ||
      std::fabs(right_wheel_est_vel) > MAX_WHEEL_DELTA) {
    RCLCPP_WARN(this->get_logger(),
                "Large wheel delta detected (L=%.4f, R=%.4f), skipping update",
                left_wheel_est_vel, right_wheel_est_vel);
    // Update positions but don't integrate this step
    this->left_wheel_old_pos_ = left_wheel_cur_pos;
    this->right_wheel_old_pos_ = right_wheel_cur_pos;
    return;
  }

  // Update old position with current
  this->left_wheel_old_pos_ = left_wheel_cur_pos;
  this->right_wheel_old_pos_ = right_wheel_cur_pos;

  // Compute linear and angular diff (differential drive from mid wheels)
  const double linear = (right_wheel_est_vel + left_wheel_est_vel) * 0.5;
  const double angular =
      (right_wheel_est_vel - left_wheel_est_vel) / this->wheel_separation_;

  // Debug output for rotation issues
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Wheels: L=%.4f R=%.4f | Vel: L=%.4f R=%.4f | Motion: lin=%.4f ang=%.4f",
      left_wheel_cur_pos, right_wheel_cur_pos, left_wheel_est_vel,
      right_wheel_est_vel, linear, angular);

  // Only integrate if there's meaningful motion (avoid integration of noise)
  constexpr double MIN_MOTION_THRESHOLD = 1e-6; // 1 micron
  if (std::fabs(linear) > MIN_MOTION_THRESHOLD ||
      std::fabs(angular) > MIN_MOTION_THRESHOLD) {
    // Integrate odometry
    this->integrate_exact(linear, angular);
  }

  // Calculate dt for velocity estimation
  const double dt = (current_time - this->timestamp_).seconds();

  // Estimate speeds with rolling average for smoothing
  if (dt >= 0.0001) {
    double instantaneous_linear = linear / dt;
    double instantaneous_angular = angular / dt;

    // If motion is negligible, force velocities to zero to prevent drift
    constexpr double MIN_VELOCITY_THRESHOLD = 0.001; // 1 mm/s or 0.001 rad/s
    if (std::fabs(instantaneous_linear) < MIN_VELOCITY_THRESHOLD &&
        std::fabs(instantaneous_angular) < MIN_VELOCITY_THRESHOLD) {
      this->linear_ = 0.0;
      this->angular_ = 0.0;
      // Clear velocity history when stopped
      this->linear_velocity_samples_.clear();
      this->angular_velocity_samples_.clear();
    } else {
      // Apply rolling mean for smooth velocity estimation
      this->linear_ = this->compute_rolling_mean(this->linear_velocity_samples_,
                                                 instantaneous_linear);
      this->angular_ = this->compute_rolling_mean(
          this->angular_velocity_samples_, instantaneous_angular);
    }
    this->timestamp_ = current_time;
  }

  // Publish odometry
  this->publish_odometry(current_time);
}

double OdometryNode::compute_rolling_mean(std::vector<double> &samples,
                                          double new_sample) {
  // Add new sample
  samples.push_back(new_sample);

  // Keep window size limited
  if (samples.size() > VELOCITY_WINDOW_SIZE) {
    samples.erase(samples.begin());
  }

  // Compute mean
  double sum = 0.0;
  for (const auto &sample : samples) {
    sum += sample;
  }

  return samples.empty() ? 0.0 : sum / samples.size();
}

void OdometryNode::integrate_runge_kutta2(double linear, double angular) {
  const double direction = this->heading_ + angular * 0.5;

  // Runge-Kutta 2nd order integration
  this->x_ += linear * std::cos(direction);
  this->y_ += linear * std::sin(direction);
  this->heading_ += angular;
}

void OdometryNode::integrate_exact(double linear, double angular) {
  if (std::fabs(angular) < 1e-6) {
    // Straight line motion - use Runge-Kutta
    this->integrate_runge_kutta2(linear, angular);
  } else {
    // Curved motion - exact integration
    const double heading_old = this->heading_;
    const double r = linear / angular;
    this->heading_ += angular;
    this->x_ += r * (std::sin(this->heading_) - std::sin(heading_old));
    this->y_ += -r * (std::cos(this->heading_) - std::cos(heading_old));
  }

  // Normalize heading to [-pi, pi]
  this->heading_ =
      std::atan2(std::sin(this->heading_), std::cos(this->heading_));
}

void OdometryNode::publish_odometry(const rclcpp::Time &current_time) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = this->odom_frame_;
  odom.child_frame_id = this->base_frame_;

  odom.pose.pose.position.x = this->x_;
  odom.pose.pose.position.y = this->y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = this->quaternion_from_yaw(this->heading_);

  // Pose covariance (36 elements, row-major about x, y, z, rotation about x, y,
  // z) We're only using x, y, and rotation about z (yaw)
  odom.pose.covariance[0] = 0.001; // x variance
  odom.pose.covariance[7] = 0.001; // y variance
  odom.pose.covariance[14] = 1e6;  // z variance (not used, set high)
  odom.pose.covariance[21] = 1e6;  // rotation about x (not used)
  odom.pose.covariance[28] = 1e6;  // rotation about y (not used)
  odom.pose.covariance[35] = 0.01; // yaw variance

  odom.twist.twist.linear.x = this->linear_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = this->angular_;

  // Twist covariance
  odom.twist.covariance[0] = 0.001; // linear x variance
  odom.twist.covariance[7] = 1e6;   // linear y variance (not used)
  odom.twist.covariance[14] = 1e6;  // linear z variance (not used)
  odom.twist.covariance[21] = 1e6;  // angular x variance (not used)
  odom.twist.covariance[28] = 1e6;  // angular y variance (not used)
  odom.twist.covariance[35] = 0.01; // angular z variance

  this->odom_pub_->publish(odom);

  if (this->publish_tf_ && this->tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = this->odom_frame_;
    t.child_frame_id = this->base_frame_;
    t.transform.translation.x = this->x_;
    t.transform.translation.y = this->y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = this->quaternion_from_yaw(this->heading_);
    this->tf_broadcaster_->sendTransform(t);
  }
}

geometry_msgs::msg::Quaternion OdometryNode::quaternion_from_yaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
