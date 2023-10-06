#include "rtabmap_to_octomap/OctomapToGridmap.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto OctomapToGridmap =
      std::make_shared<rtabmap_to_octomap::OctomapToGridmap>();
  rclcpp::spin(OctomapToGridmap);
  rclcpp::shutdown();
  return 0;
}