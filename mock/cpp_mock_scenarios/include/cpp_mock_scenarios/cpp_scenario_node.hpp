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

#ifndef CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_
#define CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_

#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_junit/junit5.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <vector>

namespace cpp_mock_scenarios
{
using traffic_simulator::helper::constructLaneletPose;

enum class Result { SUCCESS = 0, FAILURE = 1 };

class CppScenarioNode : public rclcpp::Node
{
public:
  explicit CppScenarioNode(
    const std::string & node_name, const std::string & map_path,
    const std::string & lanelet2_map_file, const std::string & scenario_filename,
    const bool verbose, const rclcpp::NodeOptions & option);
  void start();
  void stop(Result result, const std::string & description = "");
  void expectThrow() { exception_expect_ = true; }
  void expectNoThrow() { exception_expect_ = false; }
  template <typename T>
  auto equals(const T v1, const T v2, const T tolerance = std::numeric_limits<T>::epsilon()) const
    -> bool
  {
    if (std::abs(v2 - v1) <= tolerance) {
      return true;
    }
    return false;
  }
  void spawnEgoEntity(
    const traffic_simulator::CanonicalizedLaneletPose & spawn_lanelet_pose,
    const std::vector<traffic_simulator::CanonicalizedLaneletPose> & goal_lanelet_pose,
    const traffic_simulator_msgs::msg::VehicleParameters & parameters);

  auto getNewRoute() -> std::pair<std::optional<lanelet::Id>, std::vector<lanelet::Id>>;
  void updateRoute();
  void respawn(const lanelet::Id & start_lane_id, const lanelet::Id & goal_lane_id);
  bool processForEgoStuck();
  template <typename T>
  void callServiceWithoutResponse(const typename rclcpp::Client<T>::SharedPtr client);

protected:
  traffic_simulator::API api_;
  common::junit::JUnit5 junit_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_cli_;

  std::queue<std::pair<std::optional<lanelet::Id>, std::vector<lanelet::Id>>> route_;
  lanelet::Id spawn_start_lane_id_;
  lanelet::Id spawn_goal_lane_id_;

  bool has_cleared_npc_{false};
  bool has_respawned_ego_{false};
  bool reach_goal_{false};

private:
  std::string scenario_filename_;
  bool exception_expect_;
  std::string junit_path_;
  void update();
  virtual void onUpdate() = 0;
  virtual void onInitialize() = 0;
  rclcpp::TimerBase::SharedPtr update_timer_;
  double timeout_;
  auto configure(
    const std::string & map_path, const std::string & lanelet2_map_file,
    const std::string & scenario_filename, const bool verbose) -> traffic_simulator::Configuration
  {
    auto configuration = traffic_simulator::Configuration(map_path);
    {
      configuration.lanelet2_map_file = lanelet2_map_file;
      // configuration.lanelet2_map_file = "lanelet2_map_with_private_road_and_walkway_ele_fix.osm";
      configuration.scenario_path = scenario_filename;
      configuration.verbose = verbose;
    }
    checkConfiguration(configuration);
    return configuration;
  }

  void checkConfiguration(const traffic_simulator::Configuration & configuration);
};

}  // namespace cpp_mock_scenarios

#endif  // CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_
