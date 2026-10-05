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
  const auto intersection_ids =
    declare_parameter<std::vector<std::string>>("intersection_ids", std::vector<std::string>{});
  const auto add_light = [this](const std::string & intersection_id, const std::string & light_id) {
    if (intersection_id.empty() || light_id.empty()) {
      throw std::invalid_argument(
        "Intersection ID and virtual traffic light ID must not be empty.");
    }
    if (
      std::any_of(
        virtual_traffic_lights_.begin(), virtual_traffic_lights_.end(),
        [&light_id](const auto & entry) { return entry.second.id == light_id; })) {
      throw std::invalid_argument("Duplicate virtual traffic light ID: " + light_id);
    }
    VirtualTrafficLightControl control;
    control.id = light_id;
    virtual_traffic_lights_.emplace(intersection_id, std::move(control));
  };
  if (intersection_ids.empty()) {
    // Keep existing single-intersection parameter files and bare ros2 run working.
    const auto intersection_id = declare_parameter<std::string>("intersection_id", "1");
    const auto light_id = declare_parameter<std::string>("virtual_traffic_light_id", "2012980");
    add_light(intersection_id, light_id);
  } else {
    for (const auto & intersection_id : intersection_ids) {
      if (intersection_id.empty() || virtual_traffic_lights_.count(intersection_id) != 0) {
        throw std::invalid_argument("Empty or duplicate intersection ID: " + intersection_id);
      }
      const auto light_id = declare_parameter<std::string>(
        "intersections." + intersection_id + ".virtual_traffic_light_id", "");
      add_light(intersection_id, light_id);
    }
  }
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
    "virtual_traffic_light_state_topic", "/vtl/intersection_states");
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
  if (virtual_traffic_light_type_.empty()) {
    throw std::invalid_argument("Virtual traffic light type must not be empty.");
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
  for (const auto & [intersection_id, control] : virtual_traffic_lights_) {
    RCLCPP_INFO(
      get_logger(),
      "CONFIGURATION: intersection_id=%s, virtual_traffic_light_id=%s, "
      "virtual_traffic_light_type=%s, virtual_traffic_light_publish_rate_hz=%.3f",
      intersection_id.c_str(), control.id.c_str(), virtual_traffic_light_type_.c_str(),
      publish_rate);
  }
  RCLCPP_INFO(
    get_logger(),
    "CONFIGURATION: tracked_object_timeout_sec=%.3f, "
    "tracked_objects_freshness_sec=%.3f, conflict_clear_duration_sec=%.3f, "
    "allow_unknown_objects=%s, health_check_clock=steady",
    tracked_object_timeout_sec_, tracked_objects_freshness_sec_, conflict_clear_duration_sec_,
    allow_unknown_objects_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "TOPICS: vector_map=%s, tracked_objects=%s, odometry=%s, virtual_traffic_light=%s",
    map_subscription_->get_topic_name(), objects_subscription_->get_topic_name(),
    odometry_subscription_->get_topic_name(), virtual_traffic_light_publisher_->get_topic_name());
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
    reset_pending_approvals();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Skipping ego odometry in frame '%s'; expected 'map'.",
      msg->header.frame_id.c_str());
    return;
  }
  const auto & position = msg->pose.pose.position;
  const auto speed = msg->twist.twist.linear.x;
  if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
    reset_pending_approvals();
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
    } else if (new_state == ZoneState::OUTSIDE) {
      set_approval(intersection_id, false);
      ego.priority_entry_time.reset();
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
  const auto seen = object_last_seen_times_.find(uuid);
  const auto age =
    seen == object_last_seen_times_.end() ? -1.0 : (receive_clock_.now() - seen->second).seconds();
  for (auto it = object_zone_states_.begin(); it != object_zone_states_.end();) {
    if (it->first.first != uuid) {
      ++it;
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "%s: UUID=%s, intersection_id=%s, previous_state=%s, last_seen_age=%.3fs",
      event, uuid.c_str(), it->first.second.empty() ? "<missing>" : it->first.second.c_str(),
      state_name(it->second.state), age);
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
  last_objects_message_time_ = receive_clock_.now();
  if (msg->header.frame_id != "map") {
    object_data_status_ = "INVALID_FRAME";
    last_tracked_objects_time_.reset();
    reset_pending_approvals();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Skipping tracked objects in frame '%s'; expected 'map'.",
      msg->header.frame_id.c_str());
    return;
  }
  const auto receive_time = receive_clock_.now();
  // Detect gaps before refreshing the timestamp, including gaps between timer ticks.
  update_approval_conditions(receive_time);
  last_tracked_objects_time_ = receive_time;
  object_data_status_ = "VALID";
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
    object_data_status_ = "INVALID_POSITION";
    last_tracked_objects_time_.reset();
  }
  update_approval_conditions(receive_time);
}

void IntersectionPriorityNode::set_approval(const std::string & intersection_id, const bool granted)
{
  const auto light = virtual_traffic_lights_.find(intersection_id);
  if (light == virtual_traffic_lights_.end() || light->second.approval_granted == granted) {
    return;
  }
  auto & control = light->second;
  control.approval_granted = granted;
  control.last_waiting_key.clear();
  control.last_waiting_log_time.reset();
  control.approval_conditions_since.reset();
  RCLCPP_INFO(
    get_logger(), "%s: intersection_id=%s, virtual_traffic_light_id=%s",
    granted ? "RIGHT_OF_WAY_GRANTED" : "RIGHT_OF_WAY_REVOKED", intersection_id.c_str(),
    control.id.c_str());
}

void IntersectionPriorityNode::reset_pending_approvals()
{
  for (auto & entry : virtual_traffic_lights_) {
    entry.second.approval_conditions_since.reset();
  }
}

void IntersectionPriorityNode::update_approval_conditions(const rclcpp::Time & receive_time)
{
  const bool fresh =
    last_tracked_objects_time_ && (receive_time - *last_tracked_objects_time_).seconds() >= 0.0 &&
    (receive_time - *last_tracked_objects_time_).seconds() <= tracked_objects_freshness_sec_;
  for (auto & [intersection_id, control] : virtual_traffic_lights_) {
    if (control.approval_granted) {
      continue;
    }
    const auto queue = arrival_queues_.find(intersection_id);
    const bool ego_first = queue != arrival_queues_.end() && !queue->second.empty() &&
                           queue->second.front().type == ParticipantType::EGO;
    const auto ego = ego_zone_states_.find(intersection_id);
    const bool ego_in_priority =
      ego != ego_zone_states_.end() && ego->second.state == ZoneState::PRIORITY;
    // A missing conflict polygon must not be interpreted as a clear conflict zone.
    const auto group = intersections_.find(intersection_id);
    const bool has_polygons = group != intersections_.end() &&
                              !group->second.priority_polygons.empty() &&
                              !group->second.conflict_polygons.empty();
    // Include direct CONFLICT entries, but only in this intersection.
    const bool conflict_occupied = std::any_of(
      object_zone_states_.begin(), object_zone_states_.end(),
      [&intersection_id](const auto & entry) {
        return entry.first.second == intersection_id && entry.second.state == ZoneState::CONFLICT;
      });
    if (!has_polygons || !ego_first || !ego_in_priority || conflict_occupied || !fresh) {
      control.approval_conditions_since.reset();
    } else if (!control.approval_conditions_since) {
      control.approval_conditions_since = receive_time;
    }
  }
}

void IntersectionPriorityNode::log_waiting_status(
  const std::string & intersection_id, VirtualTrafficLightControl & control,
  const rclcpp::Time & receive_time)
{
  const auto ego = ego_zone_states_.find(intersection_id);
  if (
    control.approval_granted || ego == ego_zone_states_.end() ||
    ego->second.state == ZoneState::OUTSIDE) {
    control.last_waiting_key.clear();
    control.last_waiting_log_time.reset();
    return;
  }

  const auto queue = arrival_queues_.find(intersection_id);
  std::size_t ego_position = 0;
  const std::size_t queue_size = queue == arrival_queues_.end() ? 0 : queue->second.size();
  std::string queue_head;
  std::string queue_head_state;
  if (queue != arrival_queues_.end()) {
    for (std::size_t i = 0; i < queue->second.size(); ++i) {
      if (queue->second[i].type == ParticipantType::EGO) {
        ego_position = i + 1;
      }
    }
    if (!queue->second.empty()) {
      queue_head = queue->second.front().identifier;
      queue_head_state = state_name(queue->second.front().state);
    }
  }
  std::vector<std::string> blockers;
  for (const auto & [key, tracked] : object_zone_states_) {
    if (key.second == intersection_id && tracked.state == ZoneState::CONFLICT) {
      blockers.push_back(key.first);
    }
  }
  const double data_age =
    last_tracked_objects_time_ ? (receive_time - *last_tracked_objects_time_).seconds() : -1.0;
  const bool fresh = data_age >= 0.0 && data_age <= tracked_objects_freshness_sec_;
  const auto data_status =
    object_data_status_ == "VALID" && !fresh ? std::string{"STALE"} : object_data_status_;
  const char * reason = "CLEAR_DURATION_PENDING";
  const auto group = intersections_.find(intersection_id);
  if (
    group == intersections_.end() || group->second.priority_polygons.empty() ||
    group->second.conflict_polygons.empty()) {
    reason = "MAP_POLYGONS_MISSING";
  } else if (ego->second.state != ZoneState::PRIORITY) {
    reason = "EGO_NOT_IN_PRIORITY";
  } else if (ego_position == 0) {
    reason = "EGO_NOT_IN_QUEUE";
  } else if (!fresh) {
    reason = "OBJECT_DATA_STALE";
  } else if (!blockers.empty()) {
    reason = "CONFLICT_OCCUPIED";
  } else if (ego_position != 1) {
    reason = "EGO_NOT_FIRST";
  }

  // Ages are deliberately excluded from this key to avoid per-frame INFO logs.
  std::ostringstream key;
  key << reason << '|' << state_name(ego->second.state) << '|' << ego_position << '|' << queue_size
      << '|' << queue_head << '|' << queue_head_state << '|' << data_status;
  for (const auto & uuid : blockers) {
    key << '|' << uuid;
  }
  const bool changed = key.str() != control.last_waiting_key;
  if (
    !changed && control.last_waiting_log_time &&
    (receive_time - *control.last_waiting_log_time).seconds() < 1.0) {
    return;
  }
  control.last_waiting_key = key.str();
  control.last_waiting_log_time = receive_time;
  const auto emit = [this, changed](const std::string & message) {
    if (changed) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
    }
  };
  const auto age_text = [](const std::optional<rclcpp::Time> & time, const rclcpp::Time & now) {
    if (!time) {
      return std::string{"unavailable"};
    }
    std::ostringstream value;
    value << std::fixed << std::setprecision(3) << (now - *time).seconds() << 's';
    return value.str();
  };
  const auto head_seen = object_last_seen_times_.find(queue_head);
  const std::optional<rclcpp::Time> head_last_seen =
    head_seen == object_last_seen_times_.end() ? std::nullopt
                                               : std::optional<rclcpp::Time>{head_seen->second};
  std::ostringstream summary;
  summary << "WAITING: intersection_id=" << intersection_id << ", reason=" << reason
          << ", ego_state=" << state_name(ego->second.state)
          << ", ego_queue_position=" << ego_position << ", queue_size=" << queue_size
          << ", queue_head=" << (queue_head.empty() ? "none" : queue_head)
          << ", queue_head_state=" << (queue_head_state.empty() ? "none" : queue_head_state)
          << ", queue_head_last_seen_age=" << age_text(head_last_seen, receive_time)
          << ", object_data_status=" << data_status
          << ", data_age=" << age_text(last_tracked_objects_time_, receive_time)
          << ", last_message_age=" << age_text(last_objects_message_time_, receive_time)
          << ", freshness_limit=" << tracked_objects_freshness_sec_ << 's'
          << ", conflict_objects=" << blockers.size()
          << ", clear_elapsed=" << age_text(control.approval_conditions_since, receive_time)
          << ", clear_required=" << conflict_clear_duration_sec_ << 's';
  emit(summary.str());
  for (const auto & uuid : blockers) {
    const auto seen = object_last_seen_times_.find(uuid);
    const std::optional<rclcpp::Time> last_seen = seen == object_last_seen_times_.end()
                                                    ? std::nullopt
                                                    : std::optional<rclcpp::Time>{seen->second};
    std::ostringstream blocker;
    blocker << "WAITING_BLOCKER: intersection_id=" << intersection_id
            << ", reason=CONFLICT_OCCUPIED, UUID=" << uuid
            << ", last_seen_age=" << age_text(last_seen, receive_time)
            << ", timeout=" << tracked_object_timeout_sec_ << 's';
    emit(blocker.str());
  }
}

void IntersectionPriorityNode::publish_virtual_traffic_light()
{
  const auto receive_time = receive_clock_.now();
  // Cleanup remains active even after approval has been latched.
  remove_stale_objects(receive_time);
  update_approval_conditions(receive_time);
  const auto stamp = now();
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray message;
  message.stamp = stamp;
  message.states.reserve(virtual_traffic_lights_.size());
  for (auto & [intersection_id, control] : virtual_traffic_lights_) {
    if (
      !control.approval_granted && control.approval_conditions_since &&
      (receive_time - *control.approval_conditions_since).seconds() >=
        conflict_clear_duration_sec_) {
      set_approval(intersection_id, true);
    }
    log_waiting_status(intersection_id, control, receive_time);

    tier4_v2x_msgs::msg::VirtualTrafficLightState state;
    state.stamp = stamp;
    state.type = virtual_traffic_light_type_;
    state.id = control.id;
    state.approval = control.approval_granted;
    state.is_finalized = false;
    message.states.push_back(state);
  }
  virtual_traffic_light_publisher_->publish(message);
}

void IntersectionPriorityNode::on_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  // Preserve exit detection for an ongoing grant across map updates. Map/queue changes
  // must not revoke approval while ego is crossing; only its observed exit does.
  std::map<std::string, EgoZoneState> granted_ego_states;
  for (auto & [intersection_id, control] : virtual_traffic_lights_) {
    const auto ego = ego_zone_states_.find(intersection_id);
    if (control.approval_granted && ego != ego_zone_states_.end()) {
      granted_ego_states.emplace(intersection_id, ego->second);
    }
    control.last_waiting_key.clear();
    control.last_waiting_log_time.reset();
  }
  // Never retain stale geometry after a replacement map, even if decoding fails.
  for (auto & [intersection_id, queue] : arrival_queues_) {
    if (!queue.empty()) {
      queue.clear();
      log_arrival_queue(intersection_id);
    }
  }
  arrival_queues_.clear();
  ego_zone_states_ = std::move(granted_ego_states);
  object_zone_states_.clear();
  object_last_seen_times_.clear();
  last_objects_message_time_.reset();
  object_data_status_ = "NOT_RECEIVED";
  last_tracked_objects_time_.reset();
  reset_pending_approvals();
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
  for (const auto & [intersection_id, control] : virtual_traffic_lights_) {
    const auto group = intersections_.find(intersection_id);
    if (
      group == intersections_.end() || group->second.priority_polygons.empty() ||
      group->second.conflict_polygons.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Intersection ID=%s, virtual_traffic_light_id=%s: missing priority or conflict polygons; "
        "new approval is disabled for this intersection.",
        intersection_id.c_str(), control.id.c_str());
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
