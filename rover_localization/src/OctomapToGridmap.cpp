#include "rtabmap_to_octomap/OctomapToGridmap.hpp"

#include <grid_map_octomap/GridMapOctomapConverter.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;

namespace rtabmap_to_octomap {

OctomapToGridmap::OctomapToGridmap()
    : Node("octomap_to_gridmap_demo"),
      map_(grid_map::GridMap({"elevation", "occupancy"})) {

  read_parameters();

  this->map_.setBasicLayers({"elevation", "occupancy"});

  this->grid_map_publisher_ =
      this->create_publisher<grid_map_msgs::msg::GridMap>(
          "grid_map", rclcpp::QoS(1).transient_local());
  this->octomap_publisher_ = this->create_publisher<OctomapMessage>(
      "octomap", rclcpp::QoS(1).transient_local());

  this->octo_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_full", 1, std::bind(&OctomapToGridmap::octomap_cb, this, _1));
}

OctomapToGridmap::~OctomapToGridmap() {}

void OctomapToGridmap::octomap_cb(const OctomapMessage::SharedPtr msg) {
  this->convert_and_publish(msg);
}

bool OctomapToGridmap::read_parameters() {
  this->declare_parameter("octomap_service_topic",
                          std::string("/octomap_binary"));
  this->declare_parameter("min_x", NAN);
  this->declare_parameter("max_x", NAN);
  this->declare_parameter("min_y", NAN);
  this->declare_parameter("max_y", NAN);
  this->declare_parameter("min_z", NAN);
  this->declare_parameter("max_z", NAN);

  this->get_parameter("octomap_service_topic", this->octomapServiceTopic_);
  this->get_parameter("min_x", this->minX_);
  this->get_parameter("max_x", this->maxX_);
  this->get_parameter("min_y", this->minY_);
  this->get_parameter("max_y", this->maxY_);
  this->get_parameter("min_z", this->minZ_);
  this->get_parameter("max_z", this->maxZ_);
  return true;
}

void OctomapToGridmap::convert_and_publish(
    const OctomapMessage::SharedPtr msg) {

  octomap::OcTree *octomap = nullptr;
  octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap = dynamic_cast<octomap::OcTree *>(tree);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call convert Octomap.");
    return;
  }

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
  octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
  if (!std::isnan(this->minX_)) {
    min_bound(0) = this->minX_;
  }
  if (!std::isnan(maxX_)) {
    max_bound(0) = this->maxX_;
  }
  if (!std::isnan(minY_)) {
    min_bound(1) = this->minY_;
  }
  if (!std::isnan(maxY_)) {
    max_bound(1) = this->maxY_;
  }
  if (!std::isnan(minZ_)) {
    min_bound(2) = this->minZ_;
  }
  if (!std::isnan(maxZ_)) {
    max_bound(2) = this->maxZ_;
  }

  bool res = grid_map::GridMapOctomapConverter::fromOctomap(
      *octomap, "elevation", this->map_, &min_bound, &max_bound);
  if (!res) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call convert Octomap elevation layer.");
    return;
  }

  this->map_.setFrameId(msg->header.frame_id);

  // Publish as grid map.
  auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(this->map_);

  grid_map_msg->basic_layers = {};

  if (grid_map_msg->data[0].data.size() == 0) {
    return;
  }

  for (size_t i = 0; i < grid_map_msg->data[0].data.size(); i++) {
    if (!std::isnan(grid_map_msg->data[0].data[i])) {
      grid_map_msg->data[1].data[i] = 100.0;
    } else {
      grid_map_msg->data[0].data[i] = min_bound(2);
      grid_map_msg->data[1].data[i] = 0.0;
    }
  }

  this->grid_map_publisher_->publish(std::move(grid_map_msg));

  // Also publish as an octomap msg for visualization
  OctomapMessage octomap_msg;
  octomap_msgs::fullMapToMsg(*octomap, octomap_msg);
  octomap_msg.header.frame_id = map_.getFrameId();

  std::unique_ptr<OctomapMessage> octomap_msg_ptr(
      new OctomapMessage(octomap_msg));
  this->octomap_publisher_->publish(std::move(octomap_msg_ptr));
}

} // namespace rtabmap_to_octomap