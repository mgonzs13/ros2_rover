
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
};

} // namespace motor_controller
#endif
