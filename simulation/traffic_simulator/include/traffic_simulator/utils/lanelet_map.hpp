// Copyright 2024 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/utils/lanelet_core/lanelet_map.hpp>
#include <traffic_simulator/utils/lanelet_core/lanelet_map_core.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace traffic_simulator
{
inline namespace lanelet_map
{
using Point = geometry_msgs::msg::Point;

template <typename... Ts>
inline auto activate(Ts &&... xs)
{
  return lanelet_core::LaneletMapCore::activate(std::forward<decltype(xs)>(xs)...);
}

auto laneletLength(const lanelet::Id lanelet_id) -> double;

auto nearbyLaneletIds(
  const Pose & pose, const double distance_threshold, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids;

auto borderlinePoses() -> std::vector<Pose>;

auto visualizationMarker() -> visualization_msgs::msg::MarkerArray;
}  // namespace lanelet_map
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_
