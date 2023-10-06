
#ifndef RTABMAP_TO_OCTOMAP__OCTOMAPTOGRIDMAP_HPP_
#define RTABMAP_TO_OCTOMAP__OCTOMAPTOGRIDMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace rtabmap_to_octomap {

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation
 * layer. The grid map is published and can be viewed in Rviz.
 */
class OctomapToGridmap : public rclcpp::Node {
public:
  using GetOctomapSrv = octomap_msgs::srv::GetOctomap;
  using OctomapMessage = octomap_msgs::msg::Octomap;

  OctomapToGridmap();
  virtual ~OctomapToGridmap();
  bool read_parameters();
  void convert_and_publish(const OctomapMessage::SharedPtr octomap);
  void octomap_cb(const OctomapMessage::SharedPtr msg);

private:
  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;

  //! Octomap publisher.
  rclcpp::Publisher<OctomapMessage>::SharedPtr octomap_publisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Name of the grid map topic.
  std::string octomapServiceTopic_;

  //! Octomap sub
  rclcpp::Subscription<OctomapMessage>::SharedPtr octo_sub_;

  //! Bounding box of octomap to convert.
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;
};

} // namespace rtabmap_to_octomap
#endif // rtabmap_to_octomap__OCTOMAPTOGRIDMAP_HPP_