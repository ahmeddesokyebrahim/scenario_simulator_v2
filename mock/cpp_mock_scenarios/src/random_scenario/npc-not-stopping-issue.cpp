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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <random001_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>

#include <quaternion_operation/quaternion_operation.h>
#include "./random_util.hpp"

// headers in STL
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", /* ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map" */
      "/home/pzyskowski/projects/tier/deliverable/current/WP12/780/map",
      "lanelet2_map.osm", __FILE__, false, option),
    param_listener_(std::make_shared<random001::ParamListener>(get_node_parameters_interface())),
    engine_(seed_gen_())
  {
    start();
  }

private:
  std::shared_ptr<random001::ParamListener> param_listener_;
  random001::Params params_;
  std::random_device seed_gen_;
  std::mt19937 engine_;
  double lane_change_position = 0.0;
  bool lane_change_requested = false;

  const size_t MAX_SPAWN_NUMBER = 10;
  bool ego_is_in_stuck_ = false;
  bool driving_to_destination_ = true;

  // 300087:テレポート駅路肩レーン
  // 179398:未来館駐車場走行レーン
  // 179443:未来館駐車場路肩レーン
  // 190785:未来館駐車場路肩レーン
  // 178481:未来館と走行レーンの間のレーン
  // 1290:未来館左折退場後右側走行レーン
  lanelet::Id start_lane_id_ = 190336;
  std::vector<lanelet::Id> route_to_destination_ids_ = { 40, 179443};
  lanelet::Id destination_lane_id_ = 179443;
  std::vector<lanelet::Id> route_to_start_lane_ids_ = {190029 ,300087};



  MyStopWatch<> sw_ego_stuck_;

  StateManager<std::string> tl_state_manager_{{"red", "amber", "green"}, {10.0, 3.0, 10.0}};
  StateManager<std::string> crosswalk_pedestrian_state_manager_{{"go", "stop"}, {15.0, 15.0}};

  void updateRoute(const std::vector<lanelet::Id> & new_goal_lane_ids)
  {
    std::vector<traffic_simulator::CanonicalizedLaneletPose> new_lane_poses;
    for (const auto & id : new_goal_lane_ids) {
      new_lane_poses.push_back(api_.canonicalize(constructLaneletPose(id, 0, 0, 0, 0, 0)));
    }
    api_.requestAssignRoute("ego", new_lane_poses);
  }

  // If ego is far from lane_id, remove all entities.
  // Return if the ego is close to the lane_id.
  bool removeFarNPCsAndCheckIsInTriggerDistance(
    const std::string & entity_name_prefix, const lanelet::Id & lane_id)
  {
    const auto removeEntities = [&]() {
      for (int i = 0; i < MAX_SPAWN_NUMBER; i++) {
        const std::string name = entity_name_prefix + "_" + std::to_string(i);
        if (api_.entityExists(name)) {
          api_.despawn(name);
        }
      }
    };

    constexpr auto untrigger_distance = 220.0;  // must be longer than trigger_distance
    constexpr auto trigger_distance = 200.0;  // must be shorter than untrigger_distance
    constexpr auto too_close_for_trigger_distance = 50.0;  // must be shorter than untrigger_distance
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

  void onUpdate() override
  {
    constexpr double reach_tolerance = 3.0;
    const auto stuck_time = api_.getStandStillDuration("ego");
    const bool ego_is_in_stop = stuck_time > 5.0;
    if (
      ego_is_in_stop && driving_to_destination_ &&
      api_.reachPosition(
        "ego", api_.canonicalize(constructLaneletPose(destination_lane_id_, 0.0)),
        reach_tolerance)) {
      updateRoute(route_to_start_lane_ids_);
      driving_to_destination_ = false;
    } else if (
      ego_is_in_stop && !driving_to_destination_ &&
      api_.reachPosition(
        "ego", api_.canonicalize(constructLaneletPose(start_lane_id_, 0.0)), reach_tolerance)) {
      updateRoute(route_to_destination_ids_);
      driving_to_destination_ = true;
    }

    spawnAndMoveToGoal(1482, 38, 15.0, 15.0);
//    spawnAndMoveToGoal(1483, 38, 15.0, 15.0);
//    spawnAndMoveToGoal(1484, 39, 15.0, 15.0);
  }

  void onInitialize() override
  {
//    api_.setVerbose(true);

    srand(time(0));  // Initialize random seed

    params_ = param_listener_->get_params();

    // 初期値とゴールを設定。初期値は一度しか指定できない。ゴールは何度も指定できるが、Rvizから指定した方が早そう。
    // 路肩からの発進時に同じノード名で複数ノードが起動されてしまいautowareが発信できなくなってしまうエラーがあるため、duplicated_node_checkerを無効化する必要あり。
    const auto spawn_pose = api_.canonicalize(constructLaneletPose(start_lane_id_, 0, 0, 0, 0, 0));
    const auto goal_poses = [&](const std::vector<lanelet::Id> lane_ids) {
      std::vector<traffic_simulator::CanonicalizedLaneletPose> poses;
      for (const auto id : lane_ids) {
        poses.push_back(api_.canonicalize(constructLaneletPose(id, 20, 0, 0, 0, 0)));
      }
      return poses;
    }(route_to_destination_ids_);  // 最後がゴール、その前は並び順でcheck point
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
