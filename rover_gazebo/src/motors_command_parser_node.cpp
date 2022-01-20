
#define BOOST_BIND_NO_PLACEHOLDERS

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rover_gazebo/motors_command_parser_node.hpp"
#include "rover_interfaces/msg/motors_command.hpp"

using std::placeholders::_1;

MotorsCommandParserNode::MotorsCommandParserNode()
    : rclcpp::Node("motors_command_parser_node") {

  this->velocity_publisher =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "velocity_controller/commands", 10);

  this->position_publisher =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "joint_trajectory_controller/joint_trajectory", 10);

  this->subscription =
      this->create_subscription<rover_interfaces::msg::MotorsCommand>(
          "motors_command", 10,
          std::bind(&MotorsCommandParserNode::callback, this, _1));
}

void MotorsCommandParserNode::callback(
    const rover_interfaces::msg::MotorsCommand::SharedPtr msg) {

  // RCLCPP_INFO(this->get_logger(),
  //             "Speed: %d %d %d %d %d %d, Sterring: %d, %d, %d, %d",
  //             msg->drive_motor[0], msg->drive_motor[1], msg->drive_motor[2],
  //             msg->drive_motor[3], msg->drive_motor[4], msg->drive_motor[5],
  //             msg->corner_motor[0], msg->corner_motor[1],
  //             msg->corner_motor[2], msg->corner_motor[3]);

  // create msgs
  auto corners_positions = trajectory_msgs::msg::JointTrajectory();
  auto wheel_velocities = std_msgs::msg::Float64MultiArray();

  float pi = atan(1) * 4;

  for (long unsigned int i = 0; i < msg->drive_motor.size(); i++) {

    float value =
        this->normalize(-1000, 1000,
                        this->clamp(-1000, 1000, msg->drive_motor[i])) *
        10;

    if (i > 2) {
      value *= -1;
    }

    wheel_velocities.data.push_back(value);
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(1.0);

  for (int position : msg->corner_motor) {

    point.positions.push_back(
        this->normalize(250, 750, this->clamp(-250, 750, position)) * -pi / 2);
  }

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  points.push_back(point);

  corners_positions.joint_names = {
      "front_left_corner_joint", "front_right_corner_joint",
      "back_left_corner_joint", "back_right_corner_joint"};
  corners_positions.points = points;

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
