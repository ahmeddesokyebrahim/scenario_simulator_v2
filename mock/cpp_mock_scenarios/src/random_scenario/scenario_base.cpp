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

#include <quaternion_operation/quaternion_operation.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <common_parameters.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <geometry/distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>

#include "./random_util.hpp"

// headers in STL
#include <cstdlib>
#include <ctime>
#include <memory>
#include <random>
#include <string>
#include <vector>

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "random_scenario", "/dir", "lanelet2_map.osm", __FILE__, false, option),
    param_listener_(std::make_shared<common::ParamListener>(get_node_parameters_interface()))
  {
    start();
  }

private:
  std::shared_ptr<common::ParamListener> param_listener_;
  common::Params params_;
  double lane_change_position = 0.0;
  bool lane_change_requested = false;

  const size_t MAX_SPAWN_NUMBER = 10;

  const double TH_DESPAWN_DISTANCE = 220.0;
  const double TH_SPAWN_DISTANCE = 200.0;
  const double TH_CREAR_DISTANCE = 30.0;
  const std::string PEDESTRIAN_PREFIX = "pedestrian_";
  const std::string PARKED_VEHICLE_PREFIX = "parked_vehicle_";
  const std::string MOVING_VEHICLE_PREFIX = "moving_vehicle_";

  StateManager<std::string> traffic_light_{{"red", "amber", "green"}, {10.0, 3.0, 10.0}};
  StateManager<std::string> pedestrian_{{"go", "stop"}, {15.0, 15.0}};

  void removeFarEntity(const lanelet::Id & id, const std::string & prefix)
  {
    for (const auto & name : api_.getEntityNames()) {
      if (name.find(prefix + std::to_string(id)) == std::string::npos) {
        continue;
      }

      if (api_.reachPosition("ego", name, TH_DESPAWN_DISTANCE)) {
        continue;
      }

      RCLCPP_DEBUG_STREAM(get_logger(), "Despawn: " << name << std::endl);
      api_.despawn(name);
    }
  }

  bool isTooClose(const lanelet::Id & id)
  {
    return api_.reachPosition(
      "ego", api_.canonicalize(constructLaneletPose(id, 0.0)), TH_CREAR_DISTANCE);
  }

  bool isInSpawnRange(const lanelet::Id & id)
  {
    return api_.reachPosition(
      "ego", api_.canonicalize(constructLaneletPose(id, 0.0)), TH_SPAWN_DISTANCE);
  }

  void updateMovingVehicle(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id,
    const size_t entity_max_num, const double min_v = 3.0, const double max_v = 18.0)
  {
    removeFarEntity(spawn_lane_id, MOVING_VEHICLE_PREFIX);

    if (!isInSpawnRange(spawn_lane_id)) {
      return;
    }

    if (isTooClose(spawn_lane_id)) {
      return;
    }

    const auto & p = params_.random_parameters.road_parking_vehicle;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);
    for (size_t npc_id = 0; npc_id < entity_max_num; npc_id++) {
      const auto lon_offset =
        static_cast<double>(npc_id) / entity_max_num * api_.getLaneletLength(spawn_lane_id) +
        normal_dist(engine_);
      spawnNPCVehicle(
        spawn_lane_id, spawn_lane_id, MOVING_VEHICLE_PREFIX, npc_id,
        std::clamp(lon_offset, 0.0, api_.getLaneletLength(spawn_lane_id)), DIRECTION::CENTER,
        randomDouble(min_v, max_v));
    }
  }

  void updateParkedVehicle(
    const lanelet::Id & spawn_lane_id, const size_t entity_max_num, const DIRECTION direction)
  {
    removeFarEntity(spawn_lane_id, PARKED_VEHICLE_PREFIX);

    if (!isInSpawnRange(spawn_lane_id)) {
      return;
    }

    if (isTooClose(spawn_lane_id)) {
      return;
    }

    const auto & p = params_.random_parameters.road_parking_vehicle;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);
    for (size_t npc_id = 0; npc_id < entity_max_num; npc_id++) {
      const auto lon_offset =
        static_cast<double>(npc_id) / entity_max_num * api_.getLaneletLength(spawn_lane_id) +
        normal_dist(engine_);
      spawnNPCVehicle(
        spawn_lane_id, spawn_lane_id, PARKED_VEHICLE_PREFIX, npc_id,
        std::clamp(lon_offset, 0.0, api_.getLaneletLength(spawn_lane_id)), direction, 0.0);
    }
  }

  void updatePedestrian(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id,
    const size_t entity_max_num, const DIRECTION direction, const double min_v = 0.0,
    const double max_v = 2.0)
  {
    removeFarEntity(spawn_lane_id, PEDESTRIAN_PREFIX);

    if (!isInSpawnRange(spawn_lane_id)) {
      return;
    }

    if (isTooClose(spawn_lane_id)) {
      return;
    }

    const auto & p = params_.random_parameters.crossing_pedestrian;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);
    for (size_t npc_id = 0; npc_id < entity_max_num; ++npc_id) {
      const auto lon_offset =
        static_cast<double>(npc_id) / entity_max_num * api_.getLaneletLength(spawn_lane_id) +
        normal_dist(engine_);
      spawnNPCPedestrian(
        spawn_lane_id, goal_lane_id, npc_id,
        std::clamp(lon_offset, 0.0, api_.getLaneletLength(spawn_lane_id)), direction,
        randomDouble(min_v, max_v));
    }
  }

  void spawnNPCVehicle(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id, const std::string & prefix,
    const size_t npc_id, const double lon_offset, const DIRECTION direction, const double speed)
  {
    const std::string entity_name = prefix + std::to_string(spawn_lane_id) + "_" +
                                    std::to_string(goal_lane_id) + "_" + std::to_string(npc_id);

    if (!api_.entityExists(entity_name)) {
      const auto lanelet_pose =
        constructLaneletPose(spawn_lane_id, lon_offset, get_random_lateral_offset(direction));
      const auto vehicle_param = getVehicleParameters(get_random_entity_subtype());
      api_.spawn(entity_name, api_.canonicalize(lanelet_pose), vehicle_param);
      RCLCPP_DEBUG_STREAM(
        get_logger(), "Spawn: " << entity_name << " Speed: " << speed << std::endl);
    }

    api_.requestSpeedChange(entity_name, speed, true);
    api_.setLinearVelocity(entity_name, speed);
    api_.setVelocityLimit(entity_name, speed);

    if (spawn_lane_id == goal_lane_id) {
      return;
    }

    constexpr double reach_tolerance = 2.0;
    if (api_.reachPosition(
          entity_name,
          api_.canonicalize(
            constructLaneletPose(goal_lane_id, api_.getLaneletLength(goal_lane_id))),
          reach_tolerance)) {
      api_.despawn(entity_name);
    }
  }

  void spawnNPCPedestrian(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id, const size_t npc_id,
    const double lon_offset, const DIRECTION direction, const double speed)
  {
    const std::string entity_name = PEDESTRIAN_PREFIX + std::to_string(spawn_lane_id) + "_" +
                                    std::to_string(goal_lane_id) + "_" + std::to_string(npc_id);

    if (!api_.entityExists(entity_name)) {
      const auto lanelet_pose =
        constructLaneletPose(spawn_lane_id, lon_offset, get_random_lateral_offset(direction));
      api_.spawn(entity_name, api_.canonicalize(lanelet_pose), getPedestrianParameters());
      RCLCPP_DEBUG_STREAM(
        get_logger(), "Spawn: " << entity_name << " Speed: " << speed << std::endl);
      if (pedestrian_.getCurrentState() == "go") {
        api_.requestSpeedChange(entity_name, speed, true);
        api_.setLinearVelocity(entity_name, speed);
        api_.setVelocityLimit(entity_name, speed);
      } else {
        api_.setLinearVelocity(entity_name, 0.0);
        api_.setVelocityLimit(entity_name, speed);
      }
    } else {
      if (
        pedestrian_.getCurrentState() == "go" &&
        std::fabs(api_.getEntityStatus(entity_name).getTwist().linear.x) < 0.01) {
        api_.requestSpeedChange(entity_name, speed, true);
        api_.setLinearVelocity(entity_name, speed);
        api_.setVelocityLimit(entity_name, speed);
      }
      constexpr double reach_tolerance = 5.0;
      if (api_.reachPosition(
            "ego", api_.canonicalize(constructLaneletPose(goal_lane_id, 5.0)), reach_tolerance)) {
        api_.despawn(entity_name);
      }
    }
  }

  void spawnAndChangeLane(
    const std::string & entity_name, const LaneletPose & spawn_pose,
    const lanelet::Id & lane_change_id, const Direction & lane_change_direction)
  {
    const auto & p = params_.random_parameters.lane_following_vehicle;
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
      std::uniform_real_distribution<> lane_change_position_distribution(
        0.0, api_.getLaneletLength(lane_change_id));
      lane_change_position = lane_change_position_distribution(engine_);
      lane_change_requested = false;
    }
    const auto lanelet_pose = api_.getLaneletPose("ego");

    /// Checking the ego entity overs the lane change position.
    if (
      lanelet_pose && static_cast<LaneletPose>(lanelet_pose.value()).lanelet_id == lane_change_id &&
      std::abs(static_cast<LaneletPose>(lanelet_pose.value()).s) >= lane_change_position) {
      api_.requestLaneChange(entity_name, lane_change_direction);
      lane_change_requested = true;
    }
  }

  void spawnAndDespawnRelativeFromEgoInRange(
    const lanelet::Id & trigger_lane_id, const double trigger_lane_s, const double trigger_range,
    const double rel_x, const double rel_y)
  {
    const auto trigger_position =
      api_.canonicalize(constructLaneletPose(trigger_lane_id, trigger_lane_s));
    const auto entity_name = "spawn_nearby_ego";
    if (
      api_.reachPosition("ego", trigger_position, trigger_range) &&
      !api_.entityExists(entity_name)) {
      api_.spawn(
        entity_name, api_.getMapPoseFromRelativePose("ego", createPose(rel_x, rel_y)),
        getVehicleParameters(),
        traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    }
    if (
      !api_.reachPosition("ego", trigger_position, trigger_range) &&
      api_.entityExists(entity_name)) {
      api_.despawn(entity_name);
    }
  }

  // Set color for the traffic_right_id. Set opposite color (green <-> red) to the opposite_traffic_right_id
  void updateRandomTrafficLightColor(
    const std::vector<lanelet::Id> & traffic_light_ids,
    const std::vector<lanelet::Id> & opposite_phase_ids, const std::string & tl_color)
  {
    const auto setTlColor = [&](const auto & ids, const std::string & color) {
      for (const auto id : ids) {
        for (traffic_simulator::TrafficLight & traffic_light :
             api_.getConventionalTrafficLights(id)) {
          traffic_light.clear();
          traffic_light.set(color + " solidOn circle");
        };
      }
    };

    setTlColor(traffic_light_ids, tl_color);
    setTlColor(opposite_phase_ids, getOppositeTlColor(tl_color));
  }

  void onUpdate() override
  {
    if (route_.empty()) {
      RCLCPP_DEBUG(get_logger(), "route is empty.");
      return;
    }

    const auto current_state = api_.asFieldOperatorApplication("ego").getAutowareStateName();
    if (current_state == "ARRIVED_GOAL") {
      reach_goal_ = true;
    }

    if (reach_goal_ && current_state == "WAITING_FOR_ROUTE") {
      RCLCPP_INFO(get_logger(), "\n\nReach current goal. Set next route.\n\n");
      updateRoute();
      reach_goal_ = false;
    }

    if (processForEgoStuck()) {
      return;
    }

    const auto convert = [](const std::string & direction) {
      if (direction == "right") {
        return DIRECTION::RIGHT;
      }
      if (direction == "left") {
        return DIRECTION::LEFT;
      }
      if (direction == "very_right") {
        return DIRECTION::VERY_RIGHT;
      }
      if (direction == "very_left") {
        return DIRECTION::VERY_LEFT;
      }
      return DIRECTION::CENTER;
    };

    for (const auto & [name, data] : params_.npc.npc_names_map) {
      if (data.npc_type == "parked_vehicle") {
        updateParkedVehicle(
          data.spawn_lane_id, randomInt(data.min_num, data.max_num), convert(data.direction));
      }

      if (data.npc_type == "moving_vehicle") {
        updateMovingVehicle(
          data.spawn_lane_id, data.goal_lane_id, randomInt(data.min_num, data.max_num),
          data.min_speed, data.max_speed);
      }

      if (data.npc_type == "pedestrian") {
        updatePedestrian(
          data.spawn_lane_id, data.goal_lane_id, randomInt(data.min_num, data.max_num),
          convert(data.direction));
      }
    }

    // traffic light
    for (const auto & [name, data] : params_.traffic_light.traffic_light_names_map) {
      const auto state = traffic_light_.getCurrentState();
      updateRandomTrafficLightColor(data.traffic_light_ids, data.opposite_phase_ids, state);
    }
  }

  void onInitialize() override
  {
    srand(time(0));  // Initialize random seed

    params_ = param_listener_->get_params();

    for (const auto & [name, route] : params_.route.route_names_map) {
      if (route.init_start_lane) {
        route_.emplace(
          route.start_lane_id, !route.fix_start_pose, route.route_ids, !route.fix_goal_pose);
      } else {
        route_.emplace(std::nullopt, !route.fix_start_pose, route.route_ids, !route.fix_goal_pose);
      }
    }

    const auto [opt_start_lane_id, is_random_start_pose, route_lane_ids, is_random_goal_pose] =
      getNewRoute();
    const auto spawn_pose = api_.canonicalize(constructLaneletPose(opt_start_lane_id.value(), 5.0));
    const auto goal_poses = [&](const std::vector<lanelet::Id> lane_ids) {
      std::vector<traffic_simulator::CanonicalizedLaneletPose> poses;
      for (const auto id : lane_ids) {
        poses.push_back(api_.canonicalize(constructLaneletPose(id, 5.0)));
      }
      return poses;
    }(route_lane_ids);
    spawnEgoEntity(spawn_pose, goal_poses, getVehicleParameters());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<RandomScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
