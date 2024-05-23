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
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <random001_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
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
      "random_bs", std::string(getenv("HOME")) + "/workspace/bs_stable", "lanelet2_map.osm",
      __FILE__, false, option),
    param_listener_(std::make_shared<random001::ParamListener>(get_node_parameters_interface())),
    capture_cli_(this->create_client<std_srvs::srv::Trigger>(
      "/debug/capture/screen_shot", rmw_qos_profile_default)),
    engine_(seed_gen_())
  {
    start();
  }

private:
  std::shared_ptr<random001::ParamListener> param_listener_;
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_cli_;
  random001::Params params_;
  std::random_device seed_gen_;
  std::mt19937 engine_;
  double lane_change_position = 0.0;
  bool lane_change_requested = false;

  const size_t MAX_SPAWN_NUMBER = 10;
  bool has_cleared_npc_ = false;
  bool has_respawned_ego_ = false;

  lanelet::Id spawn_start_lane_id_{95};
  lanelet::Id spawn_goal_lane_id_{45};
  bool reach_goal_{false};

  std::queue<std::pair<std::optional<lanelet::Id>, std::vector<lanelet::Id>>> route_;

  // std::vector<lanelet::Id> goal_no1_candidate_ids_ = {45};
  // std::vector<lanelet::Id> goal_no2_candidate_ids_ = {95};

  // lanelet::Id start_lane_id_ = 95;
  // std::vector<lanelet::Id> route_to_destination_ids_ = {30, 37, 45};
  // lanelet::Id destination_lane_id_ = 45;
  // std::vector<lanelet::Id> route_to_start_lane_ids_ = {7, 95};

  // MyStopWatch<> sw_ego_stuck_;

  StateManager<std::string> tl_state_manager_{{"red", "amber", "green"}, {10.0, 3.0, 10.0}};
  StateManager<std::string> crosswalk_pedestrian_state_manager_{{"go", "stop"}, {15.0, 15.0}};

  auto getNewRoute() -> std::pair<std::optional<lanelet::Id>, std::vector<lanelet::Id>>
  {
    const auto [opt_start_lane_id, route_lane_ids] = route_.front();
    route_.pop();
    route_.emplace(opt_start_lane_id, route_lane_ids);

    return {opt_start_lane_id, route_lane_ids};
  }

  void updateRoute()
  {
    const auto [opt_start_lane_id, route_lane_ids] = getNewRoute();
    if (opt_start_lane_id.has_value() && !route_lane_ids.empty()) {
      respawn(opt_start_lane_id.value(), route_lane_ids.back());
      return;
    }

    std::vector<traffic_simulator::CanonicalizedLaneletPose> new_lane_poses;
    for (const auto & id : route_lane_ids) {
      new_lane_poses.push_back(api_.canonicalize(constructLaneletPose(id, 5.0)));
    }
    api_.requestClearRoute("ego");
    api_.requestAssignRoute("ego", new_lane_poses);
  }

  // If ego is far from lane_id, remove all entities.
  // Return if the ego is close to the lane_id.
  bool removeFarNPCsAndCheckIsInTriggerDistance(
    const std::string & entity_name_prefix, const lanelet::Id & lane_id)
  {
    const auto removeEntities = [&]() {
      for (size_t i = 0; i < MAX_SPAWN_NUMBER; i++) {
        const std::string name = entity_name_prefix + "_" + std::to_string(i);
        if (api_.entityExists(name)) {
          api_.despawn(name);
        }
      }
    };

    constexpr auto untrigger_distance = 220.0;  // must be longer than trigger_distance
    constexpr auto trigger_distance = 200.0;    // must be shorter than untrigger_distance
    constexpr auto too_close_for_trigger_distance =
      50.0;  // must be shorter than untrigger_distance
    const auto target_lane = api_.canonicalize(constructLaneletPose(lane_id, 0.0));

    const bool already_exist = api_.entityExists(entity_name_prefix + "_0");

    // Remove entities if the lane is far from ego.
    if (already_exist) {
      if (!api_.reachPosition("ego", target_lane, untrigger_distance)) {
        removeEntities();
      }
      return false;  // no need to spawn vehicles
    }

    // No need to spawn since the ego is too close to the lane.
    if (api_.reachPosition("ego", target_lane, too_close_for_trigger_distance)) {
      return false;  // no need to spawn vehicles
    }

    // No need to spawn since the ego is far from the lane.
    if (!api_.reachPosition("ego", target_lane, trigger_distance)) {
      return false;  // no need to spawn vehicles
    }

    return true;  // need to spawn vehicles
  }

  void spawnAndCrossPedestrian(
    const int entity_num_max, const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id)
  {
    const std::string entity_name_prefix =
      "pedestrian_" + std::to_string(spawn_lane_id) + "_" + std::to_string(goal_lane_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lane_id)) {
      return;
    }

    const auto & p = params_.random_parameters.crossing_pedestrian;
    std::normal_distribution<> offset_distribution(0.0, p.offset_variance);
    std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0, offset_distribution(engine_));
    const auto goal_pose = constructLaneletPose(goal_lane_id, 5.0);

    // api_.requestWalkStraight();

    for (int entity_index = 0; entity_index < entity_num_max; ++entity_index) {
      const std::string entity_name = entity_name_prefix + "_" + std::to_string(entity_index);
      constexpr double reach_tolerance = 5.0;

      if (!api_.entityExists(entity_name)) {
        std::normal_distribution<> offset_distribution(0.0, p.offset_variance);
        std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
        api_.spawn(entity_name, api_.canonicalize(spawn_pose), getPedestrianParameters());
        if (crosswalk_pedestrian_state_manager_.getCurrentState() == "go") {
          const auto speed = speed_distribution(engine_);
          api_.requestSpeedChange(entity_name, speed, true);
          api_.setLinearVelocity(entity_name, speed);
        } else {
          api_.setLinearVelocity(entity_name, 0.0);
        }
      } else {
        if (
          crosswalk_pedestrian_state_manager_.getCurrentState() == "go" &&
          std::fabs(api_.getEntityStatus(entity_name).getTwist().linear.x) < 0.01) {
          const auto speed = speed_distribution(engine_);
          api_.requestSpeedChange(entity_name, speed, true);
          api_.setLinearVelocity(entity_name, speed);
        }
        if (api_.reachPosition("ego", api_.canonicalize(goal_pose), reach_tolerance)) {
          api_.despawn(entity_name);
        }
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

  void spawnAndMoveToGoal(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id, const double min_v = 3.0,
    const double max_v = 18.0)
  {
    const std::string entity_name_prefix =
      "vehicle_move_to_goal_" + std::to_string(spawn_lane_id) + "_" + std::to_string(goal_lane_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lane_id)) {
      return;
    }

    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0);
    const auto goal_pose = constructLaneletPose(goal_lane_id, 0.0);

    const auto entity_name = entity_name_prefix;
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(min_v, max_v);
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
    }

    constexpr double reach_tolerance = 2.0;
    if (api_.reachPosition(entity_name, api_.canonicalize(goal_pose), reach_tolerance)) {
      api_.despawn(entity_name);
    }
  }

  void spawnRoadParkingVehicles(
    const lanelet::Id & spawn_lanelet_id, const size_t number_of_vehicles,
    const DIRECTION direction)
  {
    const std::string entity_name_prefix = "road_parking_" + std::to_string(spawn_lanelet_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lanelet_id)) {
      return;
    }

    const auto & p = params_.random_parameters.road_parking_vehicle;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);

    const auto object_type = [&]() {
      std::uniform_real_distribution<> dis(0.0, 1.0);
      double probability = dis(engine_);
      if (probability < 0.50) {
        return "car";  // 50% for "car"
      } else if (probability < 0.70) {
        return "truck";  // Additional 20% for "truck", total 70%
      } else if (probability < 0.90) {
        return "bus";  // Additional 20% for "bus", total 90%
      } else {
        return "trailer";  // Remaining 10% for "trailer"
      }
    }();

    const auto spawn_road_parking_vehicle = [&](const auto & entity_index, const auto offset) {
      const std::string entity_name = entity_name_prefix + "_" + std::to_string(entity_index);
      const auto space = static_cast<double>(entity_index) / number_of_vehicles;
      const auto spawn_position =
        space * api_.getLaneletLength(spawn_lanelet_id) + normal_dist(engine_);
      const auto spawn_pose = constructLaneletPose(spawn_lanelet_id, spawn_position, offset, 0, 0);
      const auto vehicle_param = getVehicleParameters(get_entity_subtype(object_type));
      if (!api_.entityExists(entity_name)) {
        api_.spawn(entity_name, api_.canonicalize(spawn_pose), vehicle_param);
      }
      api_.requestSpeedChange(entity_name, 0, true);
    };

    const auto [min_offset, max_offset] = [&]() -> std::pair<double, double> {
      if (direction == DIRECTION::CENTER) {
        return {-0.5, 0.5};
      } else if (direction == DIRECTION::LEFT) {
        return {3.0, 1.5};
      } else if (direction == DIRECTION::RIGHT) {
        return {-1.5, -3.0};
      } else if (direction == DIRECTION::VERY_LEFT) {
        return {3.0, 1.5};
      } else if (direction == DIRECTION::VERY_RIGHT) {
        return {-1.5, -3.0};
      }
      return {-0.1, 0.1};
    }();

    std::uniform_real_distribution<> dist(min_offset, max_offset);
    for (size_t i = 0; i < number_of_vehicles; i++) {
      spawn_road_parking_vehicle(i, dist(engine_));
    }
  }

  /// Despawn parking entity before replacing parking entity.
  void despawnRoadParkingVehicles()
  {
    for (int i = 0; i < params_.random_parameters.road_parking_vehicle.number_of_vehicle; i++) {
      api_.despawn("road_parking_" + std::to_string(i));
    }
  }

  void despawnCrossingPedestrians()
  {
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      std::string entity_name = "pedestrian" + std::to_string(i);
      if (api_.entityExists(entity_name)) {
        api_.despawn(entity_name);
      }
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
    const std::vector<int> & traffic_right_ids, const std::vector<int> & opposite_traffic_right_ids,
    const std::string & tl_color)
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

    setTlColor(traffic_right_ids, tl_color);
    setTlColor(opposite_traffic_right_ids, getOppositeTlColor(tl_color));
  }

  template <typename T>
  void callServiceWithoutResponse(const typename rclcpp::Client<T>::SharedPtr client)
  {
    auto req = std::make_shared<typename T::Request>();

    // RCLCPP_DEBUG(raw_node_->get_logger(), "client request");

    if (!client->service_is_ready()) {
      // RCLCPP_DEBUG(raw_node_->get_logger(), "client is unavailable");
      return;
    }

    client->async_send_request(req, [this](typename rclcpp::Client<T>::SharedFuture result) {
      RCLCPP_DEBUG(rclcpp::get_logger(__func__), "Status: %s", result.get()->message.c_str());
    });
  }

  bool processForEgoStuck()
  {
    const auto stuck_time = api_.getStandStillDuration("ego");

    RCLCPP_DEBUG(get_logger(), "stuck time: %f[s]", stuck_time);

    constexpr auto STUCK_TIME_THRESHOLD = 5.0;
    if (stuck_time > STUCK_TIME_THRESHOLD && !has_cleared_npc_) {
      RCLCPP_ERROR(get_logger(), "\n\nEgo is in stuck. Remove all NPC.\n\n");
      auto entities = api_.getEntityNames();
      for (const auto & e : entities) {
        if (e != api_.getEgoName()) {
          api_.despawn(e);
        }
      }
      has_cleared_npc_ = true;
    }

    constexpr auto RESPAWN_TIME_THRESHOLD = 10.0;
    if (stuck_time > RESPAWN_TIME_THRESHOLD && !has_respawned_ego_) {
      RCLCPP_ERROR(get_logger(), "\n\nEgo is in stuck. Respawn ego vehicle.\n\n");
      respawn(spawn_start_lane_id_, spawn_goal_lane_id_);
      has_respawned_ego_ = true;
    }

    if (std::abs(api_.getCurrentTwist("ego").linear.x) > 0.1) {
      has_cleared_npc_ = false;
      has_respawned_ego_ = false;
    }

    return has_cleared_npc_ || has_respawned_ego_;
  }

  void respawn(const lanelet::Id & start_lane_id, const lanelet::Id & goal_lane_id)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped ego_pose;
    ego_pose.header.frame_id = "map";
    ego_pose.pose.pose =
      api_.toMapPose(api_.canonicalize(constructLaneletPose(start_lane_id, 5.0)));

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose = api_.toMapPose(api_.canonicalize(constructLaneletPose(goal_lane_id, 5.0)));

    api_.respawn("ego", ego_pose, goal_pose);
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

    constexpr double MIN_VEL = 5.0;
    constexpr double MAX_VEL = 20.0;

    // // === 以下、egoが特定のレーンに近づいたらNPCを生成する ===

    // // 静止物体を生成。位置はランダム、数もランダム。
    spawnRoadParkingVehicles(57, randomInt(1, 1), DIRECTION::LEFT);  // unstable
    spawnRoadParkingVehicles(37, randomInt(0, 2), DIRECTION::LEFT);  // unstable
    spawnRoadParkingVehicles(8022, randomInt(0, 2), DIRECTION::VERY_RIGHT);
    spawnRoadParkingVehicles(39, randomInt(0, 2), DIRECTION::VERY_LEFT);  // stuck多し

    // // 目的レーンまで動く移動物体を生成
    spawnAndMoveToGoal(97, 62, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(8017, 57, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(350, 1506, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1482, 38, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1483, 38, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1484, 39, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1501, 40, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(32, 38, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(33, 39, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(34, 40, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1314, 41, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(94, 41, MIN_VEL, MAX_VEL);

    // spawnAndMoveToGoal(175378, 174994, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1263, 106, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1265, 178001, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1153, 94, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(178233, 179475, MIN_VEL, MAX_VEL);

    // spawnAndMoveToGoal(74, 84, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(75, 83, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(75, 178573, MIN_VEL, MAX_VEL);
    // spawnAndMoveToGoal(1483, 1500, MIN_VEL, MAX_VEL);

    // // 信号機の情報を変更
    // updateRandomTrafficLightColor({10584}, {10589}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10324, 190343}, {10316, 10322}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10352}, {10356, 10359}, tl_state_manager_.getCurrentState());
    // // updateRandomTrafficLightColor({179285, 10284}, {10293, 10283}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10269, 10276}, {10263, 10277}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10247, 10261}, {10249}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10237, 10236}, {10238}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10546, 10549}, {10551}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10562, 10564}, {10556}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10575, 10569}, {10571, 10581}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10610, 10598}, {10604}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({10342}, {10343}, tl_state_manager_.getCurrentState());

    // 横断歩道歩行者をspawn（速度は毎回ランダム、人数はspawnで抽選、数秒ごとにspawnを止める）
    // spawnAndCrossPedestrian(3, 1561, 1561);
    // spawnAndCrossPedestrian(3, 1621, 1621);
    // spawnAndCrossPedestrian(2, 1606, 1606);
    // spawnAndCrossPedestrian(1, 1600, 1600);
    // spawnAndCrossPedestrian(2, 1599, 1599);
    // spawnAndCrossPedestrian(1, 1591, 1591);
    // spawnAndCrossPedestrian(1, 1586, 1586);
    // spawnAndCrossPedestrian(1, 1584, 1584);
    // spawnAndCrossPedestrian(2, 1625, 1625);
    // spawnAndCrossPedestrian(2, 1627, 1627);
    // spawnAndCrossPedestrian(2, 1628, 1628);
    // spawnAndCrossPedestrian(10, 1567, 1567);

    // NPCを出力させてLCする
    // if (api_.isInLanelet("ego", 34684, 0.1)) {
    //   spawnAndChangeLane(
    //     "lane_following_0", constructLaneletPose(34513, 0.0), 34684, Direction::RIGHT);
    // }

    // 自己位置の相対位置にnpcを配置
    // spawnAndDespawnRelativeFromEgoInRange(34621, 10.0, 20.0, 10.0, -5.0);
  }

  void onInitialize() override
  {
    // api_.setVerbose(true);

    srand(time(0));  // Initialize random seed

    params_ = param_listener_->get_params();

    std::vector<lanelet::Id> route1 = {30, 37, 45};
    route_.emplace(spawn_start_lane_id_, route1);

    std::vector<lanelet::Id> route2 = {7, 95};
    route_.emplace(std::nullopt, route2);

    const auto [opt_start_lane_id, route_lane_ids] = getNewRoute();
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
