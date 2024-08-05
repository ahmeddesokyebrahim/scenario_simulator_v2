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

#ifndef TRAFFIC_SIMULATOR__API__API_HPP_
#define TRAFFIC_SIMULATOR__API__API_HPP_

#include <simulation_api_schema.pb.h>

#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <std_msgs/msg/float64.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>
#include <traffic_simulator/traffic/traffic_controller.hpp>
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>

namespace traffic_simulator
{
struct VehicleBehavior : public entity::VehicleEntity::BuiltinBehavior
{
  static auto autoware() noexcept -> const std::string &
  {
    static const std::string name = "Autoware";
    return name;
  }
};

struct PedestrianBehavior : public entity::PedestrianEntity::BuiltinBehavior
{
};

class API
{
public:
  template <typename NodeT, typename AllocatorT = std::allocator<void>, typename... Ts>
  explicit API(
    NodeT && node, const Configuration & configuration, const double realtime_factor,
    const double frame_rate)
  : configuration_(configuration),
    node_parameters_(
      rclcpp::node_interfaces::get_node_parameters_interface(std::forward<NodeT>(node))),
    clock_pub_(rclcpp::create_publisher<rosgraph_msgs::msg::Clock>(
      node, "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    debug_marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "debug_marker", rclcpp::QoS(100), rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    clock_(getROS2Parameter<bool>("use_sim_time", true), realtime_factor, frame_rate),
    zeromq_client_(
      simulation_interface::protocol, configuration.simulator_host,
      getROS2Parameter<int>("port", 5555)),
    entity_manager_ptr_(
      std::make_shared<entity::EntityManager>(node, configuration, node_parameters_)),
    traffic_controller_ptr_(std::make_shared<traffic::TrafficController>(
      entity_manager_ptr_->getHdmapUtils(),
      [this]() { return entity_manager_ptr_->getEntityNames(); },
      [this](const auto & entity_name) { return getEntity(entity_name)->getMapPose(); },
      [this](const auto & name) { return despawn(name); }, configuration.auto_sink)),
    traffic_lights_ptr_(std::make_shared<TrafficLights>(
      node, entity_manager_ptr_->getHdmapUtils(),
      getParameter<std::string>(node_parameters_, "architecture_type", "awf/universe"))),
    real_time_factor_subscriber_(rclcpp::create_subscription<std_msgs::msg::Float64>(
      node, "/real_time_factor", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const std_msgs::msg::Float64 & message) {
        return setSimulationStepTime(message.data);
      }))
  {
    entity_manager_ptr_->setVerbose(configuration_.verbose);
    entity_manager_ptr_->setTrafficLights(traffic_lights_ptr_);
    if (not init()) {
      throw common::SimulationError("Failed to initialize simulator by InitializeRequest");
    }
  }

  // global
  template <typename ParameterT, typename... Ts>
  auto getROS2Parameter(Ts &&... xs) const -> decltype(auto)
  {
    return getParameter<ParameterT>(node_parameters_, std::forward<Ts>(xs)...);
  }

  auto init() -> bool;

  auto setVerbose(const bool verbose) -> void;

  auto setSimulationStepTime(const double step_time) -> bool;

  auto startNpcLogic() -> void;

  auto isNpcLogicStarted() const -> bool;

  auto getCurrentTime() const noexcept -> double;

  auto closeZMQConnection() -> void;

  // update
  auto updateFrame() -> bool;

  // entities, ego - spawn
  template <typename PoseType, typename ParamsType>
  auto spawn(
    const std::string & name, const PoseType & pose, const ParamsType & parameters,
    const std::string & behavior = "", const std::string & model3d = "")
    -> std::shared_ptr<entity::EntityBase>
  {
    using VehicleParameters = traffic_simulator_msgs::msg::VehicleParameters;
    using PedestrianParameters = traffic_simulator_msgs::msg::PedestrianParameters;
    using MiscObjectParameters = traffic_simulator_msgs::msg::MiscObjectParameters;

    auto register_to_entity_manager = [&]() {
      if constexpr (std::is_same<ParamsType, VehicleParameters>::value) {
        if (behavior == VehicleBehavior::autoware()) {
          return entity_manager_ptr_->spawnEntity<entity::EgoEntity>(
            name, pose, parameters, getCurrentTime(), configuration_, node_parameters_);
        } else {
          return entity_manager_ptr_->spawnEntity<entity::VehicleEntity>(
            name, pose, parameters, getCurrentTime(),
            behavior.empty() ? VehicleBehavior::defaultBehavior() : behavior);
        }
      } else if constexpr (std::is_same<ParamsType, PedestrianParameters>::value) {
        return entity_manager_ptr_->spawnEntity<entity::PedestrianEntity>(
          name, pose, parameters, getCurrentTime(),
          behavior.empty() ? PedestrianBehavior::defaultBehavior() : behavior);
      } else if constexpr (std::is_same<ParamsType, MiscObjectParameters>::value) {
        return entity_manager_ptr_->spawnEntity<entity::MiscObjectEntity>(
          name, pose, parameters, getCurrentTime());
      } else {
        THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " has an undefined type.");
      }
    };

    auto prepare_and_send_request = [&](const auto & entity, auto & reqest) -> bool {
      simulation_interface::toProto(parameters, *reqest.mutable_parameters());
      reqest.mutable_parameters()->set_name(name);
      reqest.set_asset_key(model3d);
      simulation_interface::toProto(entity->getMapPose(), *reqest.mutable_pose());
      return zeromq_client_.call(reqest).result().success();
    };

    auto register_to_environment_simulator = [&](const auto & entity) -> bool {
      if (configuration_.standalone_mode) {
        return true;
      } else {
        if constexpr (std::is_same<ParamsType, VehicleParameters>::value) {
          simulation_api_schema::SpawnVehicleEntityRequest reqest;
          reqest.set_is_ego(behavior == VehicleBehavior::autoware());
          /// @todo Should be filled from function API
          reqest.set_initial_speed(0.0);
          return prepare_and_send_request(entity, reqest);
        } else if constexpr (std::is_same<ParamsType, PedestrianParameters>::value) {
          simulation_api_schema::SpawnPedestrianEntityRequest reqest;
          return prepare_and_send_request(entity, reqest);
        } else if constexpr (std::is_same<ParamsType, MiscObjectParameters>::value) {
          simulation_api_schema::SpawnMiscObjectEntityRequest reqest;
          return prepare_and_send_request(entity, reqest);
        } else {
          return false;
        }
      }
    };

    const auto entity = register_to_entity_manager();
    if (entity && register_to_environment_simulator(entity)) {
      return entity;
    } else {
      THROW_SEMANTIC_ERROR("Spawn entity ", std::quoted(name), " resulted in failure.");
    }
  }

  // sensors - attach
  auto attachPseudoTrafficLightDetector(
    const simulation_api_schema::PseudoTrafficLightDetectorConfiguration &) -> bool;

  auto attachLidarSensor(const simulation_api_schema::LidarConfiguration &) -> bool;

  auto attachLidarSensor(
    const std::string &, const double lidar_sensor_delay,
    const helper::LidarType = helper::LidarType::VLP16) -> bool;

  auto attachDetectionSensor(const simulation_api_schema::DetectionSensorConfiguration &) -> bool;

  auto attachDetectionSensor(
    const std::string &, double detection_sensor_range, bool detect_all_objects_in_range,
    double pos_noise_stddev, int random_seed, double probability_of_lost,
    double object_recognition_delay) -> bool;

  auto attachOccupancyGridSensor(const simulation_api_schema::OccupancyGridSensorConfiguration &)
    -> bool;

  // ego - checks, getters
  auto isAnyEgoSpawned() const -> bool;

  auto getEgoName() const -> const std::string &;

  auto getEgoEntity() const -> std::shared_ptr<entity::EgoEntity>;

  auto getEgoEntity(const std::string & name) const -> std::shared_ptr<entity::EgoEntity>;

  // entities - checks, getters
  auto isEntitySpawned(const std::string & name) const -> bool;

  auto getEntityNames() const -> std::vector<std::string>;

  auto getEntity(const std::string & name) const -> std::shared_ptr<entity::EntityBase>;

  // entities - respawn, despawn, reset
  auto resetBehaviorPlugin(const std::string & name, const std::string & behavior_plugin_name)
    -> void;

  auto respawn(
    const std::string & name, const geometry_msgs::msg::PoseWithCovarianceStamped & new_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) -> void;

  auto despawn(const std::string & name) -> bool;

  auto despawnEntities() -> bool;

  // entities - features
  auto checkCollision(
    const std::string & first_entity_name, const std::string & second_entity_name) const -> bool;

  auto laneletRelativeYaw(const std::string & entity_name, const LaneletPose & lanelet_pose) const
    -> std::optional<double>;

  auto timeHeadway(const std::string & from_entity_name, const std::string & to_entity_name)
    -> std::optional<double>;

  auto boundingBoxDistance(const std::string & from_entity_name, const std::string & to_entity_name)
    -> std::optional<double>;

  auto relativePose(const std::string & from_entity_name, const std::string & to_entity_name)
    -> std::optional<geometry_msgs::msg::Pose>;

  auto relativePose(
    const std::string & from_entity_name, const geometry_msgs::msg::Pose & to_map_pose)
    -> std::optional<geometry_msgs::msg::Pose>;

  auto relativePose(
    const geometry_msgs::msg::Pose & from_map_pose, const std::string & to_entity_name)
    -> std::optional<geometry_msgs::msg::Pose>;

  auto boundingBoxRelativePose(
    const std::string & from_entity_name, const geometry_msgs::msg::Pose & to_map_pose)
    -> std::optional<geometry_msgs::msg::Pose>;

  auto boundingBoxRelativePose(
    const std::string & from_entity_name, const std::string & to_entity_name)
    -> std::optional<geometry_msgs::msg::Pose>;

  auto relativeLaneletPose(
    const std::string & from_entity_name, const std::string & to_entity_name,
    const bool allow_lane_change) -> std::optional<CanonicalizedLaneletPose>;

  auto relativeLaneletPose(
    const std::string & from_entity_name, const LaneletPose & to_lanelet_pose,
    const bool allow_lane_change) -> std::optional<CanonicalizedLaneletPose>;

  auto relativeLaneletPose(
    const LaneletPose & from_lanelet_pose, const std::string & to_entity_name,
    const bool allow_lane_change) -> std::optional<CanonicalizedLaneletPose>;

  auto boundingBoxRelativeLaneletPose(
    const std::string & from_entity_name, const std::string & to_entity_name,
    const bool allow_lane_change) -> std::optional<CanonicalizedLaneletPose>;

  auto boundingBoxRelativeLaneletPose(
    const std::string & from_entity_name, const LaneletPose & to_lanelet_pose,
    const bool allow_lane_change) -> std::optional<CanonicalizedLaneletPose>;

  // traffics, lanelet
  auto getHdmapUtils() const -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

  auto getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>;

  auto getConventionalTrafficLights() const -> std::shared_ptr<ConventionalTrafficLights>;

  /**
   * @brief Add a traffic source to the simulation
   * @param radius The radius defining the area on which entities will be spawned
   * @param rate The rate at which entities will be spawned [Hz]
   * @param speed The speed of the spawned entities
   * @param position The center of the area on which entities will be spawned (includes orientation)
   * @param distribution The parameters of the spawned entities with their respective weights for random distribution
   *                     For each entity there are 4 parameters in a tuple:
   *                     - VehicleParameters or PedestrianParameters - parameters of entity
   *                     - std::string - name of behavior to be used when spawning
   *                     - std::string - name of 3D model to be used when spawning
   *                     - double - weight of entity for random distribution
   * @param allow_spawn_outside_lane Whether entities can be spawned outside the lane
   * @param require_footprint_fitting Whether entities are required to fit inside lanelet polygon when spawned
   *                                  (allow_spawn_outside_lane has higher priority)
   * @param random_orientation Whether entities should have their orientation randomized before lane matching
   * @param random_seed [Optional] The seed for the random number generator
   */
  auto addTrafficSource(
    const double radius, const double rate, const double speed,
    const geometry_msgs::msg::Pose & position,
    const traffic::TrafficSource::Distribution & distribution,
    const bool allow_spawn_outside_lane = false, const bool require_footprint_fitting = false,
    const bool random_orientation = false, std::optional<int> random_seed = std::nullopt) -> void;

  auto getV2ITrafficLights() { return traffic_lights_ptr_->getV2ITrafficLights(); }

  auto getConventionalTrafficLights()
  {
    return traffic_lights_ptr_->getConventionalTrafficLights();
  }

private:
  auto updateTimeInSim() -> bool;

  auto updateEntitiesStatusInSim() -> bool;

  auto updateTrafficLightsInSim() -> bool;

  const Configuration configuration_;

  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  SimulationClock clock_;

  zeromq::MultiClient zeromq_client_;

  const std::shared_ptr<entity::EntityManager> entity_manager_ptr_;

  const std::shared_ptr<traffic::TrafficController> traffic_controller_ptr_;

  const std::shared_ptr<TrafficLights> traffic_lights_ptr_;

  const rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr real_time_factor_subscriber_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__API__API_HPP_
