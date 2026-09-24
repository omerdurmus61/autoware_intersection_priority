// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware_intersection_priority/autoware_intersection_priority_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::intersection_priority
{
IntersectionPriorityNode::IntersectionPriorityNode(const rclcpp::NodeOptions & options)
: Node("autoware_intersection_priority", options)
{
  intersection_id_ = declare_parameter<std::string>("intersection_id", "1");
  virtual_traffic_light_id_ = declare_parameter<std::string>("virtual_traffic_light_id", "2012980");
  virtual_traffic_light_type_ =
    declare_parameter<std::string>("virtual_traffic_light_type", "virtual");
  const auto publish_rate =
    declare_parameter<double>("virtual_traffic_light_publish_rate_hz", 10.0);
  const auto map_topic = declare_parameter<std::string>("vector_map_topic", "/map/vector_map");
  const auto objects_topic = declare_parameter<std::string>(
    "tracked_objects_topic", "/perception/object_recognition/tracking/objects");
  const auto odometry_topic =
    declare_parameter<std::string>("odometry_topic", "/localization/kinematic_state");
  const auto vtl_topic = declare_parameter<std::string>(
    "virtual_traffic_light_state_topic", "/awapi/tmp/virtual_traffic_light_states");
  tracked_object_timeout_sec_ = declare_parameter<double>("tracked_object_timeout_sec", 1.0);
  conflict_clear_duration_sec_ = declare_parameter<double>("conflict_clear_duration_sec", 0.5);
  tracked_objects_freshness_sec_ = declare_parameter<double>("tracked_objects_freshness_sec", 0.5);
  allow_unknown_objects_ = declare_parameter<bool>("allow_unknown_objects", true);
  if (
    !std::isfinite(publish_rate) || publish_rate <= 0.0 ||
    !std::isfinite(tracked_object_timeout_sec_) || tracked_object_timeout_sec_ <= 0.0 ||
    !std::isfinite(tracked_objects_freshness_sec_) || tracked_objects_freshness_sec_ <= 0.0 ||
    !std::isfinite(conflict_clear_duration_sec_) || conflict_clear_duration_sec_ < 0.0) {
    throw std::invalid_argument(
      "Publish rate, object timeout and freshness must be finite and positive; "
      "conflict clear duration must be finite and nonnegative.");
  }
  if (
    intersection_id_.empty() || virtual_traffic_light_id_.empty() ||
    virtual_traffic_light_type_.empty()) {
    throw std::invalid_argument(
      "Intersection ID and virtual traffic light ID/type must not be empty.");
  }
  const double period_sec = 1.0 / publish_rate;
  if (
    period_sec < 1e-9 ||
    period_sec >= std::chrono::duration<double>{std::chrono::nanoseconds::max()}.count()) {
    throw std::invalid_argument("Virtual traffic light publish period is outside the timer range.");
  }
  const auto publish_period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>{period_sec});
  map_subscription_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    map_topic, rclcpp::QoS{1}.transient_local().reliable(),
    std::bind(&IntersectionPriorityNode::on_map, this, std::placeholders::_1));
  objects_subscription_ = create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    objects_topic, rclcpp::QoS{1},
    std::bind(&IntersectionPriorityNode::on_objects, this, std::placeholders::_1));
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic, rclcpp::QoS{1},
    std::bind(&IntersectionPriorityNode::on_odometry, this, std::placeholders::_1));
  virtual_traffic_light_publisher_ =
    create_publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
      vtl_topic, rclcpp::QoS{10});
  virtual_traffic_light_timer_ = create_wall_timer(
    publish_period, std::bind(&IntersectionPriorityNode::publish_virtual_traffic_light, this));
}

bool IntersectionPriorityNode::contains(
  const MapPolygon & polygon, const lanelet::BasicPoint2d & point)
{
  if (polygon.outer_boundary.size() < 3) {
    return false;
  }
  // Test the strict interior in XY only; holes (including their edges) are excluded.
  if (!lanelet::geometry::within(point, lanelet::utils::to2D(polygon.outer_boundary))) {
    return false;
  }
  return std::none_of(
    polygon.inner_boundaries.begin(), polygon.inner_boundaries.end(), [&point](const auto & hole) {
      return hole.size() >= 3 &&
             lanelet::geometry::distance2d(point, lanelet::utils::to2D(hole)) == 0.0;
    });
}

const char * IntersectionPriorityNode::state_name(const ZoneState state)
{
  switch (state) {
    case ZoneState::PRIORITY:
      return "PRIORITY";
    case ZoneState::CONFLICT:
      return "CONFLICT";
    case ZoneState::OUTSIDE:
    default:
      return "OUTSIDE";
  }
}

IntersectionPriorityNode::ZoneState IntersectionPriorityNode::zone_state(
  const IntersectionPolygons & group, const lanelet::BasicPoint2d & point)
{
  const auto inside_any = [&point](const auto & polygons) {
    return std::any_of(polygons.begin(), polygons.end(), [&point](const auto & polygon) {
      return contains(polygon, point);
    });
  };
  ZoneState new_state = ZoneState::OUTSIDE;
  // Conflict wins even when the polygon boundaries overlap.
  if (inside_any(group.conflict_polygons)) {
    new_state = ZoneState::CONFLICT;
  } else if (inside_any(group.priority_polygons)) {
    new_state = ZoneState::PRIORITY;
  }
  return new_state;
}

void IntersectionPriorityNode::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (msg->header.frame_id != "map") {
    approval_conditions_since_.reset();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Skipping ego odometry in frame '%s'; expected 'map'.",
      msg->header.frame_id.c_str());
    return;
  }
  const auto & position = msg->pose.pose.position;
  const auto speed = msg->twist.twist.linear.x;
  if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
    approval_conditions_since_.reset();
    return;
  }
  const lanelet::BasicPoint2d point{position.x, position.y};
  for (const auto & [intersection_id, group] : intersections_) {
    const auto new_state = zone_state(group, point);
    auto & ego = ego_zone_states_[intersection_id];
    if (ego.state == new_state) {
      continue;
    }
    if (new_state == ZoneState::PRIORITY) {
      if (!ego.priority_entry_time) {
        ego.priority_entry_time = rclcpp::Time{msg->header.stamp};
      }
      active_intersection_id_ = intersection_id;
    } else if (new_state == ZoneState::OUTSIDE) {
      if (intersection_id == intersection_id_) {
        set_approval(false);
      }
      ego.priority_entry_time.reset();
      if (active_intersection_id_ == intersection_id) {
        active_intersection_id_.reset();
      }
    }
    const char * event = new_state == ZoneState::CONFLICT   ? "EGO_ENTERED_CONFLICT"
                         : new_state == ZoneState::PRIORITY ? "EGO_ENTERED_PRIORITY"
                                                            : "EGO_LEFT_INTERSECTION";
    RCLCPP_INFO(
      get_logger(),
      "%s: intersection_id=%s, previous_state=%s, new_state=%s, "
      "x=%.3f, y=%.3f, speed=%.3f",
      event, intersection_id.c_str(), state_name(ego.state), state_name(new_state), position.x,
      position.y, speed);
    update_arrival_queue(
      "EGO", ParticipantType::EGO, intersection_id, ego.state, new_state,
      rclcpp::Time{msg->header.stamp});
    ego.state = new_state;
  }
  update_approval_conditions(receive_clock_.now());
}

void IntersectionPriorityNode::update_zone_state(
  const std::string & uuid, const std::string & intersection_id, const IntersectionPolygons & group,
  const lanelet::BasicPoint2d & point, const rclcpp::Time & observation_time)
{
  const auto new_state = zone_state(group, point);

  const auto key = std::make_pair(uuid, intersection_id);
  // An unseen object starts OUTSIDE; do not allocate history until it enters a zone.
  if (new_state == ZoneState::OUTSIDE && object_zone_states_.count(key) == 0) {
    return;
  }
  auto & tracked = object_zone_states_[key];
  if (tracked.state == new_state) {
    return;
  }
  if (new_state == ZoneState::PRIORITY && !tracked.first_priority_entry_time) {
    tracked.first_priority_entry_time = observation_time;
  }
  const char * event = new_state == ZoneState::CONFLICT   ? "ENTERED_CONFLICT"
                       : new_state == ZoneState::PRIORITY ? "ENTERED_PRIORITY"
                                                          : "LEFT_INTERSECTION";
  RCLCPP_INFO(
    get_logger(),
    "%s: object UUID=%s, intersection_id=%s, previous_state=%s, new_state=%s, "
    "x=%.3f, y=%.3f",
    event, uuid.c_str(), intersection_id.empty() ? "<missing>" : intersection_id.c_str(),
    state_name(tracked.state), state_name(new_state), point.x(), point.y());
  update_arrival_queue(
    uuid, ParticipantType::TRACKED_OBJECT, intersection_id, tracked.state, new_state,
    observation_time);
  tracked.state = new_state;
}

void IntersectionPriorityNode::update_arrival_queue(
  const std::string & identifier, const ParticipantType type, const std::string & intersection_id,
  const ZoneState previous_state, const ZoneState new_state, const rclcpp::Time & observation_time)
{
  // Untagged geometry cannot be assigned to an intersection queue.
  if (intersection_id.empty() || previous_state == new_state) {
    return;
  }
  auto & queue = arrival_queues_[intersection_id];
  const auto entry = std::find_if(queue.begin(), queue.end(), [&](const auto & candidate) {
    return candidate.type == type && candidate.identifier == identifier;
  });
  const bool remove_entry = new_state == ZoneState::OUTSIDE &&
                            (type == ParticipantType::EGO || previous_state == ZoneState::CONFLICT);
  if (remove_entry) {
    if (entry == queue.end()) {
      return;
    }
    queue.erase(entry);
  } else if (entry != queue.end()) {
    if (entry->state == new_state) {
      return;
    }
    // PRIORITY -> OUTSIDE can be boundary noise: retain order and arrival timestamp.
    entry->state = new_state;
  } else if (previous_state == ZoneState::OUTSIDE && new_state == ZoneState::PRIORITY) {
    queue.push_back({identifier, type, intersection_id, observation_time, new_state});
    // A late-delivered observation may have an earlier arrival timestamp.
    // Equal timestamps stay in insertion order; no right-of-way tie rule is applied.
    std::stable_sort(queue.begin(), queue.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.arrival_time < rhs.arrival_time;
    });
  } else {
    // Direct conflict entry does not establish a PRIORITY arrival time.
    return;
  }
  log_arrival_queue(intersection_id);
}

void IntersectionPriorityNode::log_arrival_queue(const std::string & intersection_id) const
{
  const auto & queue = arrival_queues_.at(intersection_id);
  std::ostringstream order;
  std::optional<std::size_t> ego_position;
  order << '[';
  for (std::size_t i = 0; i < queue.size(); ++i) {
    if (i != 0) {
      order << ", ";
    }
    order << i + 1 << ": ";
    if (queue[i].type == ParticipantType::EGO) {
      order << "EGO";
      ego_position = i + 1;
    } else {
      order << "object UUID=" << queue[i].identifier;
    }
  }
  order << ']';
  RCLCPP_INFO(
    get_logger(), "Intersection %s queue: %s", intersection_id.c_str(), order.str().c_str());
  if (ego_position) {
    RCLCPP_INFO(
      get_logger(), "EGO_QUEUE_POSITION: intersection_id=%s, position=%zu, queue_size=%zu",
      intersection_id.c_str(), *ego_position, queue.size());
  }
}

bool IntersectionPriorityNode::is_allowed_object(
  const autoware_perception_msgs::msg::TrackedObject & object) const
{
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  auto label = Classification::UNKNOWN;
  float probability = -std::numeric_limits<float>::infinity();
  for (const auto & classification : object.classification) {
    if (std::isfinite(classification.probability) && classification.probability > probability) {
      label = classification.label;
      probability = classification.probability;
    }
  }
  if (label == Classification::PEDESTRIAN || label == Classification::BICYCLE) {
    return false;
  }
  // Missing/invalid classifications are UNKNOWN; do not require a vehicle-specific label.
  return label != Classification::UNKNOWN || allow_unknown_objects_;
}

void IntersectionPriorityNode::remove_tracked_object(const std::string & uuid, const char * event)
{
  for (auto it = object_zone_states_.begin(); it != object_zone_states_.end();) {
    if (it->first.first != uuid) {
      ++it;
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "%s: UUID=%s, intersection_id=%s, previous_state=%s", event, uuid.c_str(),
      it->first.second.empty() ? "<missing>" : it->first.second.c_str(),
      state_name(it->second.state));
    it = object_zone_states_.erase(it);
  }
  for (auto & [intersection_id, queue] : arrival_queues_) {
    const auto end = std::remove_if(queue.begin(), queue.end(), [&](const auto & entry) {
      return entry.type == ParticipantType::TRACKED_OBJECT && entry.identifier == uuid;
    });
    if (end != queue.end()) {
      queue.erase(end, queue.end());
      log_arrival_queue(intersection_id);
    }
  }
}

void IntersectionPriorityNode::remove_stale_objects(const rclcpp::Time & receive_time)
{
  for (auto it = object_last_seen_times_.begin(); it != object_last_seen_times_.end();) {
    if ((receive_time - it->second).seconds() < tracked_object_timeout_sec_) {
      ++it;
      continue;
    }
    remove_tracked_object(it->first, "TRACKED_OBJECT_TIMEOUT");
    it = object_last_seen_times_.erase(it);
  }
}

void IntersectionPriorityNode::on_objects(
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg)
{
  if (msg->header.frame_id != "map") {
    last_tracked_objects_time_.reset();
    approval_conditions_since_.reset();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Skipping tracked objects in frame '%s'; expected 'map'.",
      msg->header.frame_id.c_str());
    return;
  }
  const auto receive_time = receive_clock_.now();
  // Detect gaps before refreshing the timestamp, including gaps between timer ticks.
  update_approval_conditions(receive_time);
  last_tracked_objects_time_ = receive_time;
  bool valid_positions = true;
  for (const auto & object : msg->objects) {
    std::ostringstream uuid_stream;
    uuid_stream << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < object.object_id.uuid.size(); ++i) {
      if (i == 4 || i == 6 || i == 8 || i == 10) {
        uuid_stream << '-';
      }
      uuid_stream << std::setw(2) << static_cast<unsigned int>(object.object_id.uuid[i]);
    }
    const auto uuid = uuid_stream.str();
    object_last_seen_times_.insert_or_assign(uuid, receive_time);
    if (!is_allowed_object(object)) {
      remove_tracked_object(uuid, "TRACKED_OBJECT_FILTERED");
      continue;
    }
    const auto & position = object.kinematics.pose_with_covariance.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
      valid_positions = false;
      continue;
    }
    const lanelet::BasicPoint2d point{position.x, position.y};
    const rclcpp::Time observation_time{msg->header.stamp};
    for (const auto & [intersection_id, group] : intersections_) {
      update_zone_state(uuid, intersection_id, group, point, observation_time);
    }
    update_zone_state(uuid, "", ungrouped_polygons_, point, observation_time);
  }
  if (!valid_positions) {
    last_tracked_objects_time_.reset();
  }
  update_approval_conditions(receive_time);
}

void IntersectionPriorityNode::set_approval(const bool granted)
{
  if (approval_granted_ == granted) {
    return;
  }
  approval_granted_ = granted;
  approval_conditions_since_.reset();
  RCLCPP_INFO(
    get_logger(), "%s: intersection_id=%s, virtual_traffic_light_id=%s",
    granted ? "RIGHT_OF_WAY_GRANTED" : "RIGHT_OF_WAY_REVOKED", intersection_id_.c_str(),
    virtual_traffic_light_id_.c_str());
}

void IntersectionPriorityNode::update_approval_conditions(const rclcpp::Time & receive_time)
{
  if (approval_granted_) {
    return;
  }
  const auto queue = arrival_queues_.find(intersection_id_);
  const bool ego_first = queue != arrival_queues_.end() && !queue->second.empty() &&
                         queue->second.front().type == ParticipantType::EGO;
  const auto ego = ego_zone_states_.find(intersection_id_);
  const bool ego_in_priority =
    ego != ego_zone_states_.end() && ego->second.state == ZoneState::PRIORITY;
  // Include objects that entered CONFLICT directly, without joining the queue.
  const bool conflict_occupied =
    std::any_of(object_zone_states_.begin(), object_zone_states_.end(), [this](const auto & entry) {
      return entry.first.second == intersection_id_ && entry.second.state == ZoneState::CONFLICT;
    });
  const bool fresh =
    last_tracked_objects_time_ && (receive_time - *last_tracked_objects_time_).seconds() >= 0.0 &&
    (receive_time - *last_tracked_objects_time_).seconds() <= tracked_objects_freshness_sec_;
  if (!ego_first || !ego_in_priority || conflict_occupied || !fresh) {
    approval_conditions_since_.reset();
  } else if (!approval_conditions_since_) {
    approval_conditions_since_ = receive_time;
  }
}

void IntersectionPriorityNode::publish_virtual_traffic_light()
{
  const auto receive_time = receive_clock_.now();
  // Cleanup remains active even after approval has been latched.
  remove_stale_objects(receive_time);
  update_approval_conditions(receive_time);
  if (
    !approval_granted_ && approval_conditions_since_ &&
    (receive_time - *approval_conditions_since_).seconds() >= conflict_clear_duration_sec_) {
    set_approval(true);
  }

  const auto stamp = now();
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray message;
  message.stamp = stamp;
  tier4_v2x_msgs::msg::VirtualTrafficLightState state;
  state.stamp = stamp;
  state.type = virtual_traffic_light_type_;
  state.id = virtual_traffic_light_id_;
  state.approval = approval_granted_;
  state.is_finalized = false;
  message.states.push_back(state);
  virtual_traffic_light_publisher_->publish(message);
}

void IntersectionPriorityNode::on_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  // Preserve exit detection for an ongoing grant across map updates. Map/queue changes
  // must not revoke approval while ego is crossing; only its observed exit does.
  std::optional<EgoZoneState> granted_ego_state;
  const auto ego = ego_zone_states_.find(intersection_id_);
  if (approval_granted_ && ego != ego_zone_states_.end()) {
    granted_ego_state = ego->second;
  }
  // Never retain stale geometry after a replacement map, even if decoding fails.
  for (auto & [intersection_id, queue] : arrival_queues_) {
    if (!queue.empty()) {
      queue.clear();
      log_arrival_queue(intersection_id);
    }
  }
  arrival_queues_.clear();
  ego_zone_states_.clear();
  active_intersection_id_.reset();
  if (granted_ego_state) {
    ego_zone_states_[intersection_id_] = *granted_ego_state;
    active_intersection_id_ = intersection_id_;
  }
  object_zone_states_.clear();
  object_last_seen_times_.clear();
  last_tracked_objects_time_.reset();
  approval_conditions_since_.reset();
  intersections_.clear();
  ungrouped_polygons_ = {};
  auto map = std::make_shared<lanelet::LaneletMap>();
  try {
    lanelet::utils::conversion::fromBinMsg(*msg, map);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to decode vector map: %s", error.what());
    return;
  }

  const auto store_polygon = [this](const auto & primitive, MapPolygon polygon) {
    const auto subtype = primitive.attributeOr("subtype", std::string{});
    const auto intersection_id = primitive.attributeOr("intersection_id", std::string{});
    if (intersection_id.empty()) {
      RCLCPP_WARN(
        get_logger(), "Polygon Lanelet2 ID=%s has no intersection_id; storing it ungrouped.",
        std::to_string(polygon.id).c_str());
    }
    auto & group = intersection_id.empty() ? ungrouped_polygons_ : intersections_[intersection_id];
    auto & polygons =
      subtype == "intersection_priority" ? group.priority_polygons : group.conflict_polygons;
    RCLCPP_INFO(
      get_logger(), "Found %s polygon: Lanelet2 ID=%s, polygon points=%zu, intersection ID=%s",
      subtype.c_str(), std::to_string(polygon.id).c_str(), polygon.outer_boundary.size(),
      intersection_id.empty() ? "<missing>" : intersection_id.c_str());
    polygons.push_back(std::move(polygon));
  };
  const auto is_intersection_polygon = [](const auto & primitive) {
    const auto subtype = primitive.attributeOr("subtype", std::string{});
    return subtype == "intersection_priority" || subtype == "intersection_conflict";
  };

  for (const auto & area : map->areaLayer) {
    if (!is_intersection_polygon(area)) {
      continue;
    }
    MapPolygon polygon{area.id(), area.outerBoundPolygon().basicPolygon(), {}};
    for (const auto & inner_boundary : area.innerBoundPolygons()) {
      polygon.inner_boundaries.push_back(inner_boundary.basicPolygon());
    }
    store_polygon(area, std::move(polygon));
  }

  for (const auto & polygon : map->polygonLayer) {
    if (!is_intersection_polygon(polygon)) {
      continue;
    }
    store_polygon(polygon, {polygon.id(), polygon.basicPolygon(), {}});
  }

  bool has_priority_polygon = !ungrouped_polygons_.priority_polygons.empty();
  for (const auto & [intersection_id, group] : intersections_) {
    has_priority_polygon = has_priority_polygon || !group.priority_polygons.empty();
    RCLCPP_DEBUG(
      get_logger(),
      "Intersection ID=%s: priority polygons=%zu, conflict polygon found=%s (count=%zu)",
      intersection_id.c_str(), group.priority_polygons.size(),
      group.conflict_polygons.empty() ? "false" : "true", group.conflict_polygons.size());
    const auto log_polygons = [this, &intersection_id](
                                const auto & polygons, const char * subtype) {
      for (const auto & polygon : polygons) {
        RCLCPP_DEBUG(
          get_logger(), "Intersection ID=%s: %s Lanelet2 ID=%s, polygon points=%zu",
          intersection_id.c_str(), subtype, std::to_string(polygon.id).c_str(),
          polygon.outer_boundary.size());
      }
    };
    log_polygons(group.priority_polygons, "intersection_priority");
    log_polygons(group.conflict_polygons, "intersection_conflict");
    if (group.conflict_polygons.size() > 1) {
      RCLCPP_WARN(
        get_logger(), "Intersection ID=%s has %zu conflict polygons; expected one, retaining all.",
        intersection_id.c_str(), group.conflict_polygons.size());
    }
  }
  if (!has_priority_polygon) {
    RCLCPP_WARN(get_logger(), "No polygon with subtype=intersection_priority found in vector map.");
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
