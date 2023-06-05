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

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "motor_controller/vel_parser_node.hpp"
#include "rover_msgs/msg/motors_command.hpp"

using std::placeholders::_1;
using namespace motor_controller;

VelParserNode::VelParserNode() : rclcpp::Node("vel_parser_node") {

  // declaring params
  this->declare_parameter<std::vector<double>>(
      "hardware_distances", std::vector<double>({23.0, 25.5, 28.5, 26.0}));

  this->declare_parameter<int>("enc_min", 250);
  this->declare_parameter<int>("enc_max", 750);

  // Speed [-100, +100] * 6 = [-600, +600]
  this->declare_parameter<int>("speed_factor", 10);

  // getting params
  std::vector<double> hardware_distances;
  this->get_parameter("hardware_distances", hardware_distances);

  this->get_parameter("enc_min", this->enc_min);
  this->get_parameter("enc_max", this->enc_max);
  this->get_parameter("speed_factor", this->speed_factor);

  this->d1 = hardware_distances[0];
  this->d2 = hardware_distances[1];
  this->d3 = hardware_distances[2];
  this->d4 = hardware_distances[3];

  // pubs and subs
  this->publisher = this->create_publisher<rover_msgs::msg::MotorsCommand>(
      "motors_command", 10);

  this->subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&VelParserNode::callback, this, _1));
}

void VelParserNode::callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

  auto motors_command = rover_msgs::msg::MotorsCommand();

  // normalize speed and steering
  float norm_speed = this->normalize(msg->linear.x, -1, 1, -100, 100);
  float norm_steering = this->normalize(msg->angular.z, -1, 1, -100, 100) * -1;

  // calculate new speeds and steerings
  std::vector<float> new_speeds =
      this->calculate_velocity(norm_speed, norm_steering);
  std::vector<float> new_ticks =
      this->calculate_target_tick(this->calculate_target_deg(norm_steering));

  // convert to int
  for (unsigned i = 0; i < new_speeds.size(); i++) {
    motors_command.drive_motor.push_back(int(new_speeds.at(i)) *
                                         this->speed_factor);
  }

  for (unsigned i = 0; i < new_ticks.size(); i++) {
    motors_command.corner_motor.push_back(int(new_ticks.at(i)));
  }

  // publish
  this->publisher->publish(motors_command);
}

float VelParserNode::normalize(float value, float old_min, float old_max,
                               float new_min, float new_max) {
  return (new_max - new_min) * ((value - old_min) / (old_max - old_min)) +
         new_min;
}

float VelParserNode::deg_to_tick(float deg, float e_min, float e_max) {
  float temp = (e_max + e_min) / 2 + ((e_max - e_min) / 90) * deg;

  if (temp < e_min)
    temp = e_min;
  else if (temp > e_max)
    temp = e_max;

  return temp;
}

float VelParserNode::radians_to_deg(float radians) {
  float pi = atan(1) * 4;
  return radians * 180.0 / pi;
}

std::vector<float> VelParserNode::calculate_velocity(float velocity,
                                                     float radius) {

  std::vector<float> new_velocity = {0, 0, 0, 0, 0, 0};
  float new_radius = 0;

  if (velocity == 0)
    return new_velocity;

  if (abs(radius) <= 5) {
    // No turning radius, all wheels same speed
    // Go ahead / Go back
    new_velocity = {velocity,  velocity,  velocity,
                    -velocity, -velocity, -velocity};

  } else {
    // Get radius in centimeters(MAX_RADIUS(255) to MIN_RADIUS(55))
    new_radius =
        MAX_RADIUS - (((MAX_RADIUS - MIN_RADIUS) * abs(radius)) / 100.0);

    float a = pow(this->d2, 2); // Back - D2
    float b = pow(this->d3, 2); // Front - D3

    float c = pow(new_radius + this->d1, 2); // Front / Back - Farthest
    float d = pow(new_radius - this->d1, 2); // Front / Back - Closest

    float e = new_radius - this->d4; // Center - Closest
    float f = new_radius + this->d4; // Center - Farthest

    float rx = 1;

    if (new_radius < 111) {
      // Front - Farthest wheel is the Farthest
      rx = sqrt(b + c);
    } else {
      // Center - Farthest wheel is the Farthest
      rx = f;
    }

    //  Get speed of each wheel
    float abs_v1 = abs(velocity) * sqrt(b + c) / rx;
    float abs_v2 = abs(velocity) * (f / rx);
    float abs_v3 = abs(velocity) * sqrt(a + c) / rx;
    float abs_v4 = abs(velocity) * sqrt(b + d) / rx;
    float abs_v5 = abs(velocity) * (e / rx);
    float abs_v6 = abs(velocity) * sqrt(a + d) / rx;

    if (velocity < 0) { //#Go back

      if (radius < 0) { // Turn Left
        new_velocity = {-abs_v4, -abs_v5, -abs_v6, abs_v1, abs_v2, abs_v3};

      } else { // Turn Right
        new_velocity = {-abs_v1, -abs_v2, -abs_v3, abs_v4, abs_v5, abs_v6};
      }

    } else { // Go ahead

      if (radius < 0) { // Turn Left
        new_velocity = {abs_v4, abs_v5, abs_v6, -abs_v1, -abs_v2, -abs_v3};

      } else { // Turn Right
        new_velocity = {abs_v1, abs_v2, abs_v3, -abs_v4, -abs_v5, -abs_v6};
      }
    }
  }

  // Set the speeds between the range[-max_speed, +max_speed]
  return new_velocity;
}

std::vector<float> VelParserNode::calculate_target_deg(float radius) {

  float new_radius = 0;
  std::vector<float> angles = {0, 0, 0, 0};

  // Scaled from MAX_RADIUS (255) to MIN_RADIUS (55) centimeters
  if (radius == 0) {
    new_radius = MAX_RADIUS;
  } else if (-100 <= radius && radius <= 100) {
    new_radius = MAX_RADIUS - abs(radius) * int(MAX_RADIUS / 100);
  } else {
    new_radius = MAX_RADIUS;
  }

  if (new_radius == MAX_RADIUS) {
    return angles;
  }

  // Turn Right - Turn Left
  // Front Left - Front Right
  float ang7 =
      this->radians_to_deg(atan(this->d3 / (abs(new_radius) + this->d1)));

  // Front Right - Front Left
  float ang8 =
      this->radians_to_deg(atan(this->d3 / (abs(new_radius) - this->d1)));

  // Back Left - Back Right
  float ang9 =
      this->radians_to_deg(atan(this->d2 / (abs(new_radius) + this->d1)));

  // Back Right - Back Left
  float ang10 =
      this->radians_to_deg(atan(this->d2 / (abs(new_radius) - this->d1)));

  if (radius < 0) { // Turn Left
    angles = {-ang8, -ang7, ang10, ang9};

  } else { // Turn Right
    angles = {ang7, ang8, -ang9, -ang10};
  }

  return angles;
}

std::vector<float>
VelParserNode::calculate_target_tick(std::vector<float> target_angles) {
  std::vector<float> tick;

  for (int i = 0; i < 4; i++) {
    tick.push_back(
        this->deg_to_tick(target_angles[i], this->enc_min, this->enc_max));
  }

  return tick;
}