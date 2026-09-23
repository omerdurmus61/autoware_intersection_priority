// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef AUTOWARE_INTERSECTION_PRIORITY__AUTOWARE_INTERSECTION_PRIORITY_NODE_HPP_
#define AUTOWARE_INTERSECTION_PRIORITY__AUTOWARE_INTERSECTION_PRIORITY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_core/primitives/Polygon.h>

#include <vector>

namespace autoware::intersection_priority
{

class IntersectionPriorityNode : public rclcpp::Node
{
public:
  explicit IntersectionPriorityNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct PriorityPolygon
  {
    lanelet::Id id;
    lanelet::BasicPolygon3d outer_boundary;
    std::vector<lanelet::BasicPolygon3d> inner_boundaries;
  };

  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_subscription_;
  // Coordinates are stored in the vector map's frame, preserving Area holes.
  std::vector<PriorityPolygon> priority_polygons_;
};

}  // namespace autoware::intersection_priority

#endif  // AUTOWARE_INTERSECTION_PRIORITY__AUTOWARE_INTERSECTION_PRIORITY_NODE_HPP_
