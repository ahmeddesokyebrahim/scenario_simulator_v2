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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

namespace cpp_mock_scenarios
{
class CancelRequestScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit CancelRequestScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "cancel_request", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  bool canceled = false;
  void onUpdate() override
  {
    if (api_.reachPosition(
          "ego",
          traffic_simulator::helper::constructCanonicalizedLaneletPose(
            34513, 30, 0, api_.getHdmapUtils()),
          3.0)) {
      api_.cancelRequest("ego");
      canceled = true;
    }
    if (api_.isInLanelet("ego", 34507, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructLaneletPose(34513, 0, 0, 0, 0, 0),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 7);
    api_.requestSpeedChange("ego", 7, true);
    const geometry_msgs::msg::Pose goal_pose = traffic_simulator::pose::toMapPose(
      traffic_simulator::helper::constructLaneletPose(34408, 0, 0, 0, 0, 0), api_.getHdmapUtils());
    api_.requestAcquirePosition("ego", goal_pose);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::CancelRequestScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
