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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANE_CHANGE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANE_CHANGE_HPP_

#include <lanelet2_core/geometry/Lanelet.h>

#include <geometry/spline/hermite_curve.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace lane_change
{
using Pose = geometry_msgs::msg::Pose;
using Vector3 = geometry_msgs::msg::Vector3;
using Curve = math::geometry::HermiteCurve;

using Parameter = traffic_simulator::lane_change::Parameter;
using Direction = traffic_simulator::lane_change::Direction;
using Constraint = traffic_simulator::lane_change::Constraint;
using TrajectoryShape = traffic_simulator::lane_change::TrajectoryShape;

auto canChangeLane(const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id) -> bool;

auto laneChangeableLaneletId(const lanelet::Id lanelet_id, const Direction & direction)
  -> std::optional<lanelet::Id>;

auto laneChangeableLaneletId(
  const lanelet::Id lanelet_id, const Direction & direction, const std::uint8_t shift)
  -> std::optional<lanelet::Id>;

// Trajectory
auto laneChangeTrajectory(const LaneletPose & from_pose, const Parameter & lane_change_parameter)
  -> std::optional<std::pair<Curve, double>>;

auto laneChangeTrajectory(
  const Pose & from_pose, const Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold) -> std::optional<std::pair<Curve, double>>;

auto laneChangeTrajectory(
  const Pose & from_pose, const LaneletPose & to_pose, const TrajectoryShape & trajectory_shape,
  const double tangent_vector_size) -> Curve;
}  // namespace lane_change
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANE_CHANGE_HPP_
