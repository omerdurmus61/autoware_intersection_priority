// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef AUTOWARE_INTERSECTION_PRIORITY__AUTOWARE_INTERSECTION_PRIORITY_NODE_HPP_
#define AUTOWARE_INTERSECTION_PRIORITY__AUTOWARE_INTERSECTION_PRIORITY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <lanelet2_core/primitives/Polygon.h>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::intersection_priority
{

class IntersectionPriorityNode : public rclcpp::Node
{
public:
  explicit IntersectionPriorityNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct MapPolygon
  {
    lanelet::Id id;
    lanelet::BasicPolygon3d outer_boundary;
    std::vector<lanelet::BasicPolygon3d> inner_boundaries;
  };

  struct IntersectionPolygons
  {
    std::vector<MapPolygon> priority_polygons;
    std::vector<MapPolygon> conflict_polygons;
  };

  enum class ZoneState { OUTSIDE, PRIORITY, CONFLICT };

  enum class ParticipantType { EGO, TRACKED_OBJECT };

  struct ArrivalEntry
  {
    std::string identifier;
    ParticipantType type;
    std::string intersection_id;
    rclcpp::Time arrival_time;
    ZoneState state;
  };

  void update_arrival_queue(
    const std::string & identifier, ParticipantType type, const std::string & intersection_id,
    ZoneState previous_state, ZoneState new_state, const rclcpp::Time & observation_time);
  void log_arrival_queue(const std::string & intersection_id) const;

  struct ObjectZoneState
  {
    ZoneState state{ZoneState::OUTSIDE};
    // First observed PRIORITY entry in message time; retained until the map changes.
    std::optional<rclcpp::Time> first_priority_entry_time;
  };

  struct EgoZoneState
  {
    ZoneState state{ZoneState::OUTSIDE};
    // First PRIORITY entry during the current visit, using the odometry timestamp.
    std::optional<rclcpp::Time> priority_entry_time;
  };

  static ZoneState zone_state(
    const IntersectionPolygons & group, const lanelet::BasicPoint2d & point);
  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  static const char * state_name(ZoneState state);
  void update_zone_state(
    const std::string & uuid, const std::string & intersection_id,
    const IntersectionPolygons & group, const lanelet::BasicPoint2d & point,
    const rclcpp::Time & observation_time);

  void on_objects(const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg);
  static bool contains(const MapPolygon & polygon, const lanelet::BasicPoint2d & point);

  bool is_allowed_object(const autoware_perception_msgs::msg::TrackedObject & object) const;
  void remove_tracked_object(const std::string & uuid, const char * event);
  void remove_stale_objects(const rclcpp::Time & receive_time);
  void update_approval_conditions(const rclcpp::Time & receive_time);
  void publish_virtual_traffic_light();
  void set_approval(bool granted);

  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);

  std::string intersection_id_;
  std::string virtual_traffic_light_id_;
  std::string virtual_traffic_light_type_;
  double tracked_object_timeout_sec_;
  double conflict_clear_duration_sec_;
  double tracked_objects_freshness_sec_;
  bool allow_unknown_objects_;
  // Receipt-time health checks must keep advancing even if ROS/simulation time pauses.
  rclcpp::Clock receive_clock_{RCL_STEADY_TIME};
  std::map<std::string, rclcpp::Time> object_last_seen_times_;
  std::optional<rclcpp::Time> last_tracked_objects_time_;
  std::optional<rclcpp::Time> approval_conditions_since_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_subscription_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
    objects_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr
    virtual_traffic_light_publisher_;
  rclcpp::TimerBase::SharedPtr virtual_traffic_light_timer_;
  bool approval_granted_{false};
  // Coordinates are stored in the vector map's frame, preserving Area holes.
  std::map<std::string, IntersectionPolygons> intersections_;
  // Preserve geometry from older maps without assigning it to a made-up intersection.
  IntersectionPolygons ungrouped_polygons_;
  // Key: (object UUID, intersection_id). Empty intersection_id denotes ungrouped geometry.
  // Missing observations do not imply OUTSIDE; timeout cleanup removes stale history.
  std::map<std::pair<std::string, std::string>, ObjectZoneState> object_zone_states_;
  std::map<std::string, EgoZoneState> ego_zone_states_;
  // Entry time is stored in the corresponding ego_zone_states_ record.
  std::optional<std::string> active_intersection_id_;
  // Ordered by observed arrival timestamp only; equal timestamps retain insertion order.
  std::map<std::string, std::vector<ArrivalEntry>> arrival_queues_;
};

}  // namespace autoware::intersection_priority

#endif  // AUTOWARE_INTERSECTION_PRIORITY__AUTOWARE_INTERSECTION_PRIORITY_NODE_HPP_
