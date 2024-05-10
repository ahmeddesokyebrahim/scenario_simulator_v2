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

enum class TrafficLightsPhase { EGO_GREEN, VEHICLE_GREEN, ALL_GREEN, UNKNOWN };

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", "/home/dmoszynski/robotecai/autoware/sources/autoware_maps/odaiba_beta",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

  lanelet::Id ego_start_lane_id = 176193;
  lanelet::Id ego_goal_lane_id = 1527;
  std::vector<lanelet::Id> ego_traffic_lights{2585, 2587, 2589};

  lanelet::Id vehicle_start_lane_id = 1486;
  lanelet::Id vehicle_goal_lane_id = 1524;
  lanelet::Id vehicle_traffic_light_change_lane_id = 1499;
  std::vector<lanelet::Id> vehicle_traffic_lights{2593, 2595, 2597};

  TrafficLightsPhase traffic_lights_phase = TrafficLightsPhase::UNKNOWN;
  double reach_tolerance = 2.0;

private:
  void setTrafficLight(const lanelet::Id id, const std::string color)
  {
    for (traffic_simulator::TrafficLight & traffic_light : api_.getConventionalTrafficLights(id)) {
      traffic_light.clear();
      traffic_light.set(color + " solidOn circle");
    }
  }

  void setTrafficLightsPhase(const TrafficLightsPhase phase)
  {
    std::string ego_traffic_lights_color;
    std::string vehicle_traffic_lights_color;
    switch (phase) {
      case TrafficLightsPhase::UNKNOWN:
        return;
        break;
      case TrafficLightsPhase::EGO_GREEN:
        ego_traffic_lights_color = "green";
        vehicle_traffic_lights_color = "red";
        break;
      case TrafficLightsPhase::VEHICLE_GREEN:
        ego_traffic_lights_color = "red";
        vehicle_traffic_lights_color = "green";
        break;
      case TrafficLightsPhase::ALL_GREEN:
        ego_traffic_lights_color = "green";
        vehicle_traffic_lights_color = "green";
        break;
    }

    for (auto const & traffic_light : ego_traffic_lights)
      setTrafficLight(traffic_light, ego_traffic_lights_color);
    for (auto const & traffic_light : vehicle_traffic_lights)
      setTrafficLight(traffic_light, vehicle_traffic_lights_color);
  }

  void updateTrafficLights()
  {
    const bool vehicle_green_phase_trigger = api_.reachPosition(
      "vehicle", api_.canonicalize(constructLaneletPose(vehicle_start_lane_id, 0.0)),
      reach_tolerance);

    const bool ego_green_phase_trigger = api_.reachPosition(
      "vehicle0",
      api_.canonicalize(constructLaneletPose(vehicle_traffic_light_change_lane_id, 0.0)),
      reach_tolerance);

    if (vehicle_green_phase_trigger && traffic_lights_phase != TrafficLightsPhase::VEHICLE_GREEN) {
      setTrafficLightsPhase(TrafficLightsPhase::VEHICLE_GREEN);
    } else if (ego_green_phase_trigger && traffic_lights_phase != TrafficLightsPhase::EGO_GREEN) {
      setTrafficLightsPhase(TrafficLightsPhase::EGO_GREEN);
    }
  }

  void spawnAndMoveToGoal(
    std::string entity_name, const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id,
    const double speed)
  {
    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0);
    const auto goal_pose = constructLaneletPose(goal_lane_id, 0.0);

    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
    }

    if (api_.reachPosition(entity_name, api_.canonicalize(goal_pose), reach_tolerance)) {
      api_.despawn(entity_name);
    }
  }

  void onUpdate() override
  {
    std::cout << std::endl << "<<<<<<<<<<< UPDATE  <<<<<<<<<<< " << std::endl;
    // setTrafficLightsPhase(TrafficLightsPhase::ALL_GREEN);
    spawnAndMoveToGoal("ego", ego_start_lane_id, ego_goal_lane_id, 10.0);
    // updateTrafficLights();

    spawnAndMoveToGoal("vehicle0", vehicle_start_lane_id, vehicle_goal_lane_id, 5.0);
    spawnAndMoveToGoal("vehicle1", 1487, 1500, 10.0);
    spawnAndMoveToGoal("vehicle2", 1488, 1501, 15.0);
  }

  void onInitialize() override {}
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
