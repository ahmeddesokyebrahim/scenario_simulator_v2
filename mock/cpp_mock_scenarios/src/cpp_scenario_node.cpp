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

#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <iostream>

namespace cpp_mock_scenarios
{
CppScenarioNode::CppScenarioNode(
  const std::string & node_name, const std::string & map_path,
  const std::string & lanelet2_map_file, const std::string & scenario_filename, const bool verbose,
  const rclcpp::NodeOptions & option)
: Node(node_name, option),
  api_(
    this,
    configure(
      this->declare_parameter<std::string>("home_dir", std::string(getenv("HOME"))) +
        this->declare_parameter<std::string>("map_dir", map_path),
      lanelet2_map_file, scenario_filename, verbose),
    1.0, 20),
  capture_cli_(this->create_client<std_srvs::srv::Trigger>(
    "/debug/capture/screen_shot", rmw_qos_profile_default)),
  scenario_filename_(scenario_filename),
  init_lane_id_(this->declare_parameter<int>("respawn.init_lane_id")),
  goal_lane_id_(this->declare_parameter<int>("respawn.goal_lane_id")),
  exception_expect_(false),
  engine_(seed_gen_())
{
  declare_parameter<std::string>("junit_path", "/tmp");
  get_parameter<std::string>("junit_path", junit_path_);
  declare_parameter<double>("timeout", 10.0);
  get_parameter<double>("timeout", timeout_);

  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose::setConsiderPoseByRoadSlope([&]() {
    if (not has_parameter("consider_pose_by_road_slope")) {
      declare_parameter("consider_pose_by_road_slope", false);
    }
    return get_parameter("consider_pose_by_road_slope").as_bool();
  }());
}

void CppScenarioNode::update()
{
  onUpdate();
  try {
    api_.updateFrame();
    if (api_.getCurrentTime() >= timeout_) {
      stop(Result::FAILURE);
    }
  } catch (const common::scenario_simulator_exception::Error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what());
    if (exception_expect_) {
      stop(Result::SUCCESS);
    } else {
      stop(Result::FAILURE);
    }
  }
}

void CppScenarioNode::start()
{
  onInitialize();
  api_.startNpcLogic();
  using namespace std::chrono_literals;
  update_timer_ = this->create_wall_timer(50ms, std::bind(&CppScenarioNode::update, this));
}

void CppScenarioNode::stop(Result result, const std::string & description)
{
  junit_.testsuite("cpp_mock_scenario");
  switch (result) {
    case Result::SUCCESS: {
      common::junit::Pass pass_case;
      junit_.testsuite("cpp_mock_scenario").testcase(scenario_filename_).pass.push_back(pass_case);
      std::cout << "cpp_scenario:success" << std::endl;
      break;
    }
    case Result::FAILURE: {
      common::junit::Failure failure_case("result", "failure");
      failure_case.message = description;
      junit_.testsuite("cpp_mock_scenario")
        .testcase(scenario_filename_)
        .failure.push_back(failure_case);
      std::cerr << "cpp_scenario:failure" << std::endl;
      break;
    }
  }
  // junit_.testsuite("cpp_mock_scenario").testcase(scenario_filename_).time = api_.getCurrentTime();
  junit_.write_to(junit_path_.c_str(), "  ");
  update_timer_->cancel();
  rclcpp::shutdown();
  std::exit(0);
}

void CppScenarioNode::spawnEgoEntity(
  const traffic_simulator::CanonicalizedLaneletPose & spawn_lanelet_pose,
  const std::vector<traffic_simulator::CanonicalizedLaneletPose> & goal_lanelet_poses,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
{
  api_.updateFrame();
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 20.0));
  api_.spawn("ego", spawn_lanelet_pose, parameters, traffic_simulator::VehicleBehavior::autoware());
  api_.asFieldOperatorApplication("ego").declare_parameter<bool>("allow_goal_modification", true);
  api_.attachLidarSensor("ego", 0.0);

  api_.attachDetectionSensor("ego", 200.0, true, 0.0, 0, 0.0, 0.0);

  api_.attachOccupancyGridSensor([this] {
    simulation_api_schema::OccupancyGridSensorConfiguration configuration;
    // clang-format off
      configuration.set_architecture_type(api_.getROS2Parameter<std::string>("architecture_type", "awf/universe/20240605"));
      configuration.set_entity("ego");
      configuration.set_filter_by_range(true);
      configuration.set_height(200);
      configuration.set_range(300);
      configuration.set_resolution(0.5);
      configuration.set_update_duration(0.1);
      configuration.set_width(200);
    // clang-format on
    return configuration;
  }());
  api_.requestAssignRoute("ego", goal_lanelet_poses);

  using namespace std::chrono_literals;
  while (!api_.asFieldOperatorApplication("ego").engaged()) {
    if (api_.asFieldOperatorApplication("ego").engageable()) {
      api_.asFieldOperatorApplication("ego").engage();
    }
    api_.updateFrame();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 20.0));
  }
}

void CppScenarioNode::checkConfiguration(const traffic_simulator::Configuration & configuration)
{
  try {
    configuration.getLanelet2MapFile();
    configuration.getPointCloudMapFile();
  } catch (const common::SimulationError &) {
    stop(Result::FAILURE);
  }
}

auto CppScenarioNode::getNewRoute()
  -> std::tuple<std::optional<lanelet::Id>, bool, std::vector<lanelet::Id>, bool>
{
  const auto [opt_start_lane_id, is_random_start_pose, route_lane_ids, is_random_goal_pose] =
    route_.front();
  route_.pop();
  route_.emplace(opt_start_lane_id, is_random_start_pose, route_lane_ids, is_random_goal_pose);

  return {opt_start_lane_id, is_random_start_pose, route_lane_ids, is_random_goal_pose};
}

void CppScenarioNode::updateRoute()
{
  const auto [opt_start_lane_id, is_random_start_pose, route_lane_ids, is_random_goal_pose] =
    getNewRoute();
  if (opt_start_lane_id.has_value() && !route_lane_ids.empty()) {
    respawn(
      opt_start_lane_id.value(), is_random_start_pose, route_lane_ids.back(), is_random_goal_pose);
    return;
  }

  const auto lon_offset = [&, this](const bool is_random, const auto & lane_id) {
    const auto lane_length = api_.getLaneletLength(lane_id);
    std::uniform_real_distribution<> dist(3.0, api_.getLaneletLength(lane_id) - 3.0);
    return is_random ? dist(engine_) : 3.0;
  };

  std::vector<traffic_simulator::CanonicalizedLaneletPose> new_lane_poses;
  for (const auto & id : route_lane_ids) {
    new_lane_poses.push_back(
      api_.canonicalize(constructLaneletPose(id, lon_offset(is_random_goal_pose, id))));
  }
  api_.requestClearRoute("ego");
  api_.requestAssignRoute("ego", new_lane_poses);
}

void CppScenarioNode::respawn(
  const lanelet::Id & start_lane_id, const bool is_random_start_pose,
  const lanelet::Id & goal_lane_id, const bool is_random_goal_pose)
{
  const auto lon_offset = [&, this](const bool is_random, const auto & lane_id) {
    const auto lane_length = api_.getLaneletLength(lane_id);
    std::uniform_real_distribution<> dist(3.0, api_.getLaneletLength(lane_id) - 3.0);
    return is_random ? dist(engine_) : 3.0;
  };

  geometry_msgs::msg::PoseWithCovarianceStamped ego_pose;
  ego_pose.header.frame_id = "map";
  ego_pose.pose.pose = api_.toMapPose(api_.canonicalize(
    constructLaneletPose(start_lane_id, lon_offset(is_random_start_pose, start_lane_id))));

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose = api_.toMapPose(api_.canonicalize(
    constructLaneletPose(goal_lane_id, lon_offset(is_random_goal_pose, goal_lane_id))));

  api_.respawn("ego", ego_pose, goal_pose);
}

bool CppScenarioNode::processForEgoStuck()
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
    callServiceWithoutResponse<std_srvs::srv::Trigger>(capture_cli_);
    has_cleared_npc_ = true;
  }

  constexpr auto RESPAWN_TIME_THRESHOLD = 10.0;
  if (stuck_time > RESPAWN_TIME_THRESHOLD && !has_respawned_ego_) {
    RCLCPP_ERROR(get_logger(), "\n\nEgo is in stuck. Respawn ego vehicle.\n\n");
    respawn(init_lane_id_, false, goal_lane_id_, false);
    has_respawned_ego_ = true;
    callServiceWithoutResponse<std_srvs::srv::Trigger>(capture_cli_);
  }

  if (std::abs(api_.getCurrentTwist("ego").linear.x) > 0.1) {
    has_cleared_npc_ = false;
    has_respawned_ego_ = false;
  }

  return has_cleared_npc_ || has_respawned_ego_;
}

template <typename T>
void CppScenarioNode::callServiceWithoutResponse(const typename rclcpp::Client<T>::SharedPtr client)
{
  auto req = std::make_shared<typename T::Request>();

  if (!client->service_is_ready()) {
    return;
  }

  client->async_send_request(req, [this](typename rclcpp::Client<T>::SharedFuture result) {
    RCLCPP_DEBUG(rclcpp::get_logger(__func__), "Status: %s", result.get()->message.c_str());
  });
}
}  // namespace cpp_mock_scenarios
