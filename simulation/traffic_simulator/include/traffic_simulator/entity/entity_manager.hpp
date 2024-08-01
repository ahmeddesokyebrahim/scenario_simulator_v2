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

#ifndef TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/misc_object_entity.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator_msgs/msg/entity_status_with_trajectory_array.hpp>

namespace traffic_simulator
{
namespace entity
{
class LaneletMarkerQoS : public rclcpp::QoS
{
public:
  explicit LaneletMarkerQoS(std::size_t depth = 1) : rclcpp::QoS(depth) { transient_local(); }
};

class EntityMarkerQoS : public rclcpp::QoS
{
public:
  explicit EntityMarkerQoS(std::size_t depth = 100) : rclcpp::QoS(depth) {}
};

class EntityManager
{
  using EntityStatusWithTrajectoryArray =
    traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(
    NodeT && node, const Configuration & configuration,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
  : configuration(configuration),
    clock_ptr_(node->get_clock()),
    node_topics_interface(rclcpp::node_interfaces::get_node_topics_interface(node)),
    node_parameters_(node_parameters),
    broadcaster_(node),
    base_link_broadcaster_(node),
    entity_status_array_pub_ptr_(rclcpp::create_publisher<EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    lanelet_marker_pub_ptr_(rclcpp::create_publisher<MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    npc_logic_started_(false),
    traffic_lights_ptr_(nullptr),
    hdmap_utils_ptr_(std::make_shared<hdmap_utils::HdMapUtils>(
      configuration.lanelet2_map_path(), getOrigin(node_parameters))),
    markers_raw_(hdmap_utils_ptr_->generateMarker())
  {
    updateHdmapMarker();
  }

  ~EntityManager() = default;

  // global
  /**
     This function is necessary because the TrafficLights object is created after the EntityManager, 
     so it can be assigned during the call of the EntityManager constructor. 
     TrafficLights cannot be created before the EntityManager due to the dependency on HdMapUtils.
   */
  auto setTrafficLights(const std::shared_ptr<TrafficLights> & traffic_lights_ptr) -> void;

  auto setVerbose(const bool verbose) -> void;

  auto startNpcLogic(const double current_time) -> void;

  auto isNpcLogicStarted() const -> bool;

  auto makeDebugMarker() const -> visualization_msgs::msg::MarkerArray;

  // update
  auto update(const double current_time, const double step_time) -> void;

  auto updateNpcLogic(const std::string & name, const double current_time, const double step_time)
    -> const CanonicalizedEntityStatus &;

  auto updateHdmapMarker() -> void;

  auto broadcastEntityTransform() -> void;

  auto broadcastTransform(
    const geometry_msgs::msg::PoseStamped & pose, const bool static_transform = true) -> void;

  // entities, ego - spawn
  template <typename EntityType, typename PoseType, typename ParametersType, typename... Ts>
  auto spawnEntity(
    const std::string & name, const PoseType & pose, const ParametersType & parameters,
    const double current_time, Ts &&... xs) -> std::shared_ptr<entity::EntityBase>
  {
    auto makeEntityStatus = [&]() -> CanonicalizedEntityStatus {
      EntityStatus entity_status;

      if constexpr (std::is_same_v<std::decay_t<EntityType>, EgoEntity>) {
        if (isAnyEgoSpawned()) {
          THROW_SEMANTIC_ERROR("Multiple egos in the simulation are unsupported yet.");
        } else {
          entity_status.type.type = traffic_simulator_msgs::msg::EntityType::EGO;
        }
      } else if constexpr (std::is_same_v<std::decay_t<EntityType>, VehicleEntity>) {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
      } else if constexpr (std::is_same_v<std::decay_t<EntityType>, PedestrianEntity>) {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
      } else {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
      }

      entity_status.subtype = parameters.subtype;
      entity_status.time = current_time;
      entity_status.name = name;
      entity_status.bounding_box = parameters.bounding_box;
      entity_status.action_status = traffic_simulator_msgs::msg::ActionStatus();
      entity_status.action_status.current_action = "waiting for initialize";

      const auto include_crosswalk = [](const auto & entity_type) {
        return (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == entity_type.type) ||
               (traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == entity_type.type);
      }(entity_status.type);

      const auto matching_distance = [](const auto & parameters) {
        if constexpr (std::is_same_v<
                        std::decay_t<ParametersType>,
                        traffic_simulator_msgs::msg::VehicleParameters>) {
          return std::max(
                   parameters.axles.front_axle.track_width,
                   parameters.axles.rear_axle.track_width) *
                   0.5 +
                 1.0;
        } else {
          return parameters.bounding_box.dimensions.y * 0.5 + 1.0;
        }
      }(parameters);

      if constexpr (std::is_same_v<std::decay_t<PoseType>, LaneletPose>) {
        entity_status.pose = pose::toMapPose(pose, hdmap_utils_ptr_);
        // here bounding_box and matching_distance are not used to adjust LaneletPose
        // it is just rewritten, assuming that in the scenario is right, alternatively:
        // toCanonicalizedLaneletPose(entity_status.pose, parameters.bounding_box,
        // {pose.lanelet_id}, include_crosswalk, matching_distance, hdmap_utils_ptr_);
        return CanonicalizedEntityStatus(entity_status, pose::canonicalize(pose, hdmap_utils_ptr_));
      } else if constexpr (std::is_same_v<std::decay_t<PoseType>, CanonicalizedLaneletPose>) {
        entity_status.pose = pose::toMapPose(pose);
        return CanonicalizedEntityStatus(entity_status, pose);
      } else if constexpr (std::is_same_v<std::decay_t<PoseType>, geometry_msgs::msg::Pose>) {
        const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
          pose, parameters.bounding_box, include_crosswalk, matching_distance, hdmap_utils_ptr_);
        return CanonicalizedEntityStatus(entity_status, canonicalized_lanelet_pose);
      }
    };

    if (const auto [iter, success] = entities_.emplace(
          name, std::make_unique<EntityType>(
                  name, makeEntityStatus(), hdmap_utils_ptr_, parameters,
                  std::forward<decltype(xs)>(xs)...));
        success) {
      // FIXME: this ignores V2I traffic lights
      iter->second->setTrafficLights(traffic_lights_ptr_->getConventionalTrafficLights());
      return iter->second;
    } else {
      THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " is already exists.");
    }
  }

  // ego - checks, getters
  auto getNumberOfEgo() const -> std::size_t;

  auto isAnyEgoSpawned() const -> bool;

  auto getEgoName() const -> const std::string &;

  auto getEgoEntity() const -> std::shared_ptr<entity::EgoEntity>;

  auto getEgoEntity(const std::string & name) const -> std::shared_ptr<entity::EgoEntity>;

  // entities - checks, getters
  auto isEntitySpawned(const std::string & name) -> bool;

  auto getEntityNames() const -> const std::vector<std::string>;

  auto getEntity(const std::string & name) const -> std::shared_ptr<entity::EntityBase>;

  auto getEntityOrNullptr(const std::string & name) const -> std::shared_ptr<entity::EntityBase>;

  // entities - respawn, despawn, reset
  /**
   * @brief Reset behavior plugin of the target entity.
   * The internal behavior is to take over the various parameters and save them, then respawn the Entity and set the parameters.
   * @param name The name of the target entity.
   * @param behavior_plugin_name The name of the behavior plugin you want to set.
   * @sa entity::PedestrianEntity::BuiltinBehavior
   * @sa entity::VehicleEntity::BuiltinBehavior
   */
  auto resetBehaviorPlugin(const std::string & name, const std::string & behavior_plugin_name)
    -> void;

  auto despawnEntity(const std::string & name) -> bool;

  // traffics, lanelet
  auto getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

private:
  /* */ Configuration configuration;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface;

  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  /* */ tf2_ros::StaticTransformBroadcaster broadcaster_;

  /* */ tf2_ros::TransformBroadcaster base_link_broadcaster_;

  const rclcpp::Publisher<EntityStatusWithTrajectoryArray>::SharedPtr entity_status_array_pub_ptr_;

  const rclcpp::Publisher<MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;

  /* */ std::unordered_map<std::string, std::shared_ptr<entity::EntityBase>> entities_;

  /* */ bool npc_logic_started_;

  /* */ std::shared_ptr<TrafficLights> traffic_lights_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  /* */ MarkerArray markers_raw_;
};
}  // namespace entity
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
