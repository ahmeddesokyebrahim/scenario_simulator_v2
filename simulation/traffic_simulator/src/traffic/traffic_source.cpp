// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

constexpr std::size_t spawnable_lanes_limit = 1e3;
constexpr double lanelet_sampling_step = 0.1;
constexpr int max_randomization_attempts = 1e4;

namespace traffic_simulator
{
namespace traffic
{
TrafficSource::TrafficSource(
  const double radius, const double rate, const double speed,
  const geometry_msgs::msg::Point & position,
  const std::vector<std::pair<std::variant<VehicleParams, PedestrianParams>, double>> & params,
  const std::optional<int> random_seed,
  const std::function<void(
    const std::string &, const geometry_msgs::msg::Pose &, const VehicleParams &, const double)> &
    vehicle_spawn_function,
  const std::function<void(
    const std::string &, const geometry_msgs::msg::Pose &, const PedestrianParams &,
    const double)> & pedestrian_spawn_function,
  const Configuration & configuration, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils)
: radius_(radius),
  rate_(rate),
  speed_(speed),
  position_(position),
  source_id_(next_source_id_++),
  vehicle_spawn_function_(vehicle_spawn_function),
  pedestrian_spawn_function_(pedestrian_spawn_function),
  hdmap_utils_(hdmap_utils),
  spawnable_lanelets_(hdmap_utils->getNearbyLaneletIds(
    position, radius, false, static_cast<std::size_t>(spawnable_lanes_limit))),
  id_distribution_(0, spawnable_lanelets_.size() - 1),
  angle_distribution_(0.0, M_PI * 2.0),
  radius_distribution_(0.0, radius_),
  config_(configuration)
{
  std::transform(
    params.begin(), params.end(), std::back_inserter(params_),
    [](const std::pair<std::variant<VehicleParams, PedestrianParams>, double> & pair) {
      return pair.first;
    });

  std::vector<double> weights;
  std::transform(
    params.begin(), params.end(), std::back_inserter(weights),
    [](const std::pair<std::variant<VehicleParams, PedestrianParams>, double> & pair) {
      return pair.second;
    });
  params_distribution_ = std::discrete_distribution<>(weights.begin(), weights.end());

  if (spawnable_lanelets_.empty()) {
    THROW_SIMULATION_ERROR("TrafficSource ", source_id_, " has no spawnable lanelets.");
  }
  for (const auto & id : spawnable_lanelets_) {
    s_distributions_.emplace(
      id, std::uniform_real_distribution<double>(getSmallestSValue(id), getBiggestSValue(id)));
  }
  if (random_seed) {
    engine_.seed(random_seed.value());
  }
}

auto TrafficSource::getRandomPose(const bool random_orientation) -> geometry_msgs::msg::Pose
{
  const double angle = angle_distribution_(engine_);
  const double radius = radius_distribution_(engine_);

  geometry_msgs::msg::Pose pose;
  /// @todo add orientation from TrafficSource orientation when it will have orientation
  pose.position = position_;
  pose.position.x += radius * std::cos(angle);
  pose.position.y += radius * std::sin(angle);

  if (random_orientation) {
    pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(
      traffic_simulator::helper::constructRPY(0.0, 0.0, angle_distribution_(engine_)));
  }

  return pose;
}

auto TrafficSource::getSmallestSValue(const lanelet::Id id) -> double
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = id;
  for (pose.s = 0.0; pose.s < hdmap_utils_->getLaneletLength(id); pose.s += lanelet_sampling_step) {
    if (convertToPoseInArea(pose)) {
      return pose.s;
    }
  }
  return 0.0;
}

auto TrafficSource::getBiggestSValue(const lanelet::Id id) -> double
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = id;
  for (pose.s = hdmap_utils_->getLaneletLength(id); pose.s > 0.0; pose.s -= lanelet_sampling_step) {
    if (convertToPoseInArea(pose)) {
      return pose.s;
    }
  }
  return hdmap_utils_->getLaneletLength(id);
}

auto TrafficSource::convertToPoseInArea(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::optional<geometry_msgs::msg::Pose>
{
  const auto map_pose = hdmap_utils_->toMapPose(lanelet_pose).pose;
  const double distance2D =
    std::hypot(map_pose.position.x - position_.x, map_pose.position.y - position_.y);
  if (distance2D <= radius_) {
    return map_pose;
  }
  return std::nullopt;
}

auto TrafficSource::getNewEntityName() -> std::string
{
  return "traffic_source_" + std::to_string(source_id_) + "_entity_" + std::to_string(entity_id_++);
}

void TrafficSource::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  if (current_time - last_spawn_time_ < 1.0 / rate_) {
    return;
  }
  last_spawn_time_ = current_time;

  randomParams();
  if (isPedestrian(*current_params_)) {
    pedestrian_spawn_function_(
      getNewEntityName(), getValidRandomPose(), std::get<PedestrianParams>(*current_params_),
      speed_);
  } else {
    vehicle_spawn_function_(
      getNewEntityName(), getValidRandomPose(), std::get<VehicleParams>(*current_params_), speed_);
  }
}

auto TrafficSource::randomParams() -> void
{
  const auto current_params_idx = params_distribution_(engine_);
  current_params_ = std::next(params_.begin(), current_params_idx);
}

auto TrafficSource::isPedestrian(const std::variant<VehicleParams, PedestrianParams> & params)
  -> bool
{
  return std::holds_alternative<PedestrianParams>(params);
}

auto TrafficSource::isPoseValid(const geometry_msgs::msg::Pose & pose) -> bool
{
  if (config_.allow_spawn_outside_lane) {
    return true;
  }
  const auto lanelet_pose = hdmap_utils_->toLaneletPose(pose, isPedestrian(*current_params_));
  if (lanelet_pose) {
    if (!config_.require_footprint_fitting) {
      return true;
    }
    /// @todo enable footprint fitting
  }
  return false;
}

auto TrafficSource::getValidRandomPose() -> geometry_msgs::msg::Pose
{
  for (int tries = 0; tries < max_randomization_attempts; ++tries) {
    const auto candidate_pose = getRandomPose();
    if (isPoseValid(candidate_pose)) {
      return candidate_pose;
    }
  }
  THROW_SIMULATION_ERROR("TrafficSource failed to get valid random pose.");
}

auto TrafficSource::getRandomLaneletId() -> lanelet::Id
{
  return spawnable_lanelets_.at(id_distribution_(engine_));
}

auto TrafficSource::getRandomSValue(const lanelet::Id id) -> double
{
  if (s_distributions_.find(id) == s_distributions_.end()) {
    THROW_SIMULATION_ERROR(
      "TrafficSource::getRandomSValue failed to find random distribution for lanelet_id ", id,
      " this should not happen. Please contact the developer.");
  }
  return s_distributions_.at(id)(engine_);
}
}  // namespace traffic
}  // namespace traffic_simulator
