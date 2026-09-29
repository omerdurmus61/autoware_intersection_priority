// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <fstream>
#include <memory>
#include <string>

// Write a serialized Lanelet2 fixture without publishing to a live ROS domain.
int main(int argc, char ** argv)
{
  if (argc != 3) {
    return 1;
  }
  const bool overlap = std::string{argv[2]} == "overlap";
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id next_id = 1;
  const auto add = [&](
                     const std::string & intersection, const char * subtype, double x0, double x1) {
    lanelet::Points3d points;
    points.emplace_back(next_id++, x0, 0.0, 0.0);
    points.emplace_back(next_id++, x1, 0.0, 0.0);
    points.emplace_back(next_id++, x1, 10.0, 0.0);
    points.emplace_back(next_id++, x0, 10.0, 0.0);
    lanelet::Polygon3d polygon(next_id++, points);
    polygon.attributes()["subtype"] = subtype;
    polygon.attributes()["intersection_id"] = intersection;
    map->add(polygon);
  };
  add("1", "intersection_priority", 0.0, 10.0);
  add("1", "intersection_conflict", 8.0, 18.0);
  add("2", "intersection_priority", overlap ? 0.0 : 30.0, overlap ? 20.0 : 40.0);
  add("2", "intersection_conflict", overlap ? 18.0 : 38.0, overlap ? 28.0 : 48.0);
  // Deliberately incomplete intersection to exercise the missing-geometry guard.
  add("3", "intersection_priority", 60.0, 70.0);
  autoware_map_msgs::msg::LaneletMapBin message;
  lanelet::utils::conversion::toBinMsg(map, &message);
  std::ofstream output(argv[1], std::ios::binary);
  output.write(reinterpret_cast<const char *>(message.data.data()), message.data.size());
  return output ? 0 : 1;
}
