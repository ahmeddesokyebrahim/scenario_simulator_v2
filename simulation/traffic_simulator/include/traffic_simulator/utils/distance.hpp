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

#ifndef TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_

#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/utils/lanelet/distance.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{
inline namespace distance
{
// Lateral
auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change) -> std::optional<double>;

auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance, bool allow_lane_change) -> std::optional<double>;

// Longitudinal
auto longitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change)
  -> std::optional<double>;

// BoundingBox
auto boundingBoxDistance(
  const geometry_msgs::msg::Pose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box) -> std::optional<double>;

auto boundingBoxLaneLateralDistance(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool allow_lane_change)
  -> std::optional<double>;

auto boundingBoxLaneLongitudinalDistance(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool include_adjacent_lanelet,
  bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>;

auto splineDistanceToBoundingBox(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedLaneletPose & pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, double width_extension_right = 0.0,
  double width_extension_left = 0.0, double length_extension_front = 0.0,
  double length_extension_rear = 0.0) -> std::optional<double>;

// Bounds
auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id) -> double;

auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids)
  -> double;

auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id) -> double;

auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids)
  -> double;

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id) -> double;

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids)
  -> double;

// Other objects
auto distanceToTrafficLightStopLine(
  const math::geometry::CatmullRomSplineInterface & spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto distanceToCrosswalk(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_crosswalk_id) -> std::optional<double>;

auto distanceToCrosswalk(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedLaneletPose & pose) -> std::optional<double>;

auto distanceToStopLine(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_stop_line_id) -> std::optional<double>;

template <typename... Ts>
auto distanceToStopLine(Ts &&... xs)
{
  return lanelet2::distance::getDistanceToStopLine(std::forward<decltype(xs)>(xs)...);
}

auto distanceToYieldStop(
  const CanonicalizedLaneletPose & reference_pose, const lanelet::Ids & following_lanelets,
  const std::vector<CanonicalizedLaneletPose> & other_poses) -> std::optional<double>;

auto distanceToNearestConflictingPose(
  const lanelet::Ids & following_lanelets, const math::geometry::CatmullRomSplineInterface & spline,
  const std::vector<CanonicalizedEntityStatus> & other_statuses) -> std::optional<double>;
}  // namespace distance
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_
