// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware_intersection_priority/autoware_intersection_priority_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware::intersection_priority
{

IntersectionPriorityNode::IntersectionPriorityNode(const rclcpp::NodeOptions & options)
: Node("autoware_intersection_priority", options)
{
  map_subscription_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local().reliable(),
    std::bind(&IntersectionPriorityNode::on_map, this, std::placeholders::_1));
}


void IntersectionPriorityNode::on_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  // Never retain stale geometry after a replacement map, even if decoding fails.
  priority_polygons_.clear();
  auto map = std::make_shared<lanelet::LaneletMap>();
  try {
    lanelet::utils::conversion::fromBinMsg(*msg, map);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to decode vector map: %s", error.what());
    return;
  }

  for (const auto & area : map->areaLayer) {
    if (area.attributeOr("subtype", std::string{}) != "intersection_priority") {
      continue;
    }
    PriorityPolygon polygon{area.id(), area.outerBoundPolygon().basicPolygon(), {}};
    for (const auto & inner_boundary : area.innerBoundPolygons()) {
      polygon.inner_boundaries.push_back(inner_boundary.basicPolygon());
    }
    priority_polygons_.push_back(std::move(polygon));
  }

  for (const auto & polygon : map->polygonLayer) {
    if (polygon.attributeOr("subtype", std::string{}) != "intersection_priority") {
      continue;
    }
    priority_polygons_.push_back({polygon.id(), polygon.basicPolygon(), {}});
  }

  if (priority_polygons_.empty()) {
    RCLCPP_WARN(get_logger(), "No polygon with subtype=intersection_priority found in vector map.");
    return;
  }

  for (const auto & polygon : priority_polygons_) {
    RCLCPP_INFO(
      get_logger(), "Found intersection_priority polygon: Lanelet2 ID=%s, polygon points=%zu",
      std::to_string(polygon.id).c_str(), polygon.outer_boundary.size());
  }
}

}  // namespace autoware::intersection_priority

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::intersection_priority::IntersectionPriorityNode>());
  rclcpp::shutdown();
  return 0;
}
