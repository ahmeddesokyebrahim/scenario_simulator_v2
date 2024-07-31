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
class DecelerateAndFollowScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit DecelerateAndFollowScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "decelerate_and_follow",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    double ego_accel = api_.getEntity("ego")->getCurrentAccel().linear.x;
    double ego_twist = api_.getEntity("ego")->getCurrentTwist().linear.x;
    // double npc_accel = static_cast<EntityStatus>(api_.getEntity("npc")->getStatus()).action_status.accel.linear.x;
    double npc_twist = api_.getEntity("npc")->getCurrentTwist().linear.x;
    ;
    // LCOV_EXCL_START
    if (ego_twist > (npc_twist + 1) && ego_accel > 0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (api_.checkCollision("ego", "npc")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP
    if (api_.getCurrentTime() >= 10) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }
  void onInitialize() override
  {
    auto ego_entity = api_.spawn(
      "ego",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34741, 0.0, 0.0, api_.getHdmapUtils()),
      getVehicleParameters());
    ego_entity->setLinearVelocity(15);

    auto npc_entity = api_.spawn(
      "npc",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34741, 15.0, 0.0, api_.getHdmapUtils()),
      getVehicleParameters());
    npc_entity->setLinearVelocity(10);
    npc_entity->requestSpeedChange(10, true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::DecelerateAndFollowScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
