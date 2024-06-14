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

#ifndef LaneletMapCoreTRAFFIC_SIMULATOR__UTILS__LANELET_MAP_CORE_MEMORY_HPP_
#define LaneletMapCoreTRAFFIC_SIMULATOR__UTILS__LANELET_MAP_CORE_MEMORY_HPP_

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <filesystem>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <mutex>
#include <scenario_simulator_exception/exception.hpp>

namespace std
{
template <>
struct hash<std::tuple<lanelet::Id, lanelet::Id, bool>>
{
public:
  size_t operator()(const std::tuple<lanelet::Id, lanelet::Id, bool> & data) const
  {
    std::hash<lanelet::Id> lanelet_id_hash;
    size_t seed = 0;
    // hash combine like boost library
    seed ^= lanelet_id_hash(std::get<0>(data)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= lanelet_id_hash(std::get<1>(data)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<bool>{}(std::get<2>(data)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace std

namespace traffic_simulator
{
namespace lanelet_map_core
{
using Point = geometry_msgs::msg::Point;
using Spline = math::geometry::CatmullRomSpline;

class RouteCache
{
public:
  auto getRoute(
    const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
    const bool allow_lane_change, const lanelet::LaneletMapPtr & lanelet_map,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph) -> lanelet::Ids
  {
    if (!exists(from_lanelet_id, to_lanelet_id, allow_lane_change)) {
      constexpr int routing_cost_id = 0;
      const auto & from_lanelet = lanelet_map->laneletLayer.get(from_lanelet_id);
      const auto & to_lanelet = lanelet_map->laneletLayer.get(to_lanelet_id);
      if (const auto route =
            routing_graph->getRoute(from_lanelet, to_lanelet, routing_cost_id, allow_lane_change);
          !route || route->shortestPath().empty()) {
        appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, lanelet::Ids());
      } else {
        lanelet::Ids shortest_path_ids;
        for (const auto & lanelet : route->shortestPath()) {
          shortest_path_ids.push_back(lanelet.id());
        }
        appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, shortest_path_ids);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.at({from_lanelet_id, to_lanelet_id, allow_lane_change});
  }

  auto getRoute(const lanelet::Id from, const lanelet::Id to, const bool allow_lane_change)
    -> decltype(auto)
  {
    if (!exists(from, to, allow_lane_change)) {
      THROW_SIMULATION_ERROR(
        "route from : ", from, " to : ", to, (allow_lane_change ? " with" : " without"),
        " lane change does not exists on route cache.");
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      return data_.at({from, to, allow_lane_change});
    }
  }

  std::unordered_map<std::tuple<lanelet::Id, lanelet::Id, bool>, lanelet::Ids> data_;
  std::mutex mutex_;

  auto exists(const lanelet::Id from, const lanelet::Id to, const bool allow_lane_change) -> bool
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::tuple<lanelet::Id, lanelet::Id, bool> key = {from, to, allow_lane_change};
    return data_.find(key) != data_.end();
  }

  auto appendData(
    const lanelet::Id from, const lanelet::Id to, const bool allow_lane_change,
    const lanelet::Ids & route) -> void
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[{from, to, allow_lane_change}] = route;
  }
};

class CenterPointsCache
{
public:
  auto centerPoints(lanelet::Id lanelet_id) -> decltype(auto)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.at(lanelet_id);
  }

  auto centerPointsSpline(lanelet::Id lanelet_id) -> decltype(auto)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return splines_[lanelet_id];
  }

  auto getCenterPoints(const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map)
    -> std::vector<Point>
  {
    if (!exists(lanelet_id)) {
      appendData(lanelet_id, centerPoints(lanelet_id, lanelet_map));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.at(lanelet_id);
  }

  auto getCenterPointsSpline(
    const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map)
    -> std::shared_ptr<Spline>
  {
    if (!exists(lanelet_id)) {
      appendData(lanelet_id, centerPoints(lanelet_id, lanelet_map));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return splines_[lanelet_id];
  }

  std::unordered_map<lanelet::Id, std::vector<Point>> data_;
  std::unordered_map<lanelet::Id, std::shared_ptr<Spline>> splines_;
  std::mutex mutex_;

  auto exists(const lanelet::Id lanelet_id) -> bool
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.find(lanelet_id) != data_.end();
  }

  auto appendData(const lanelet::Id lanelet_id, const std::vector<Point> & route) -> void
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[lanelet_id] = route;
    splines_[lanelet_id] = std::make_shared<Spline>(route);
  }

  auto centerPoints(const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map)
    -> std::vector<Point>
  {
    std::vector<Point> center_points;
    for (const auto & point : lanelet_map->laneletLayer.get(lanelet_id).centerline()) {
      center_points.push_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
    }
    if (center_points.size() == 2) {
      const auto p0 = center_points[0];
      const auto p2 = center_points[1];
      const auto p1 = geometry_msgs::build<Point>()
                        .x((p0.x + p2.x) * 0.5)
                        .y((p0.y + p2.y) * 0.5)
                        .z((p0.z + p2.z) * 0.5);
      center_points.clear();
      center_points.push_back(p0);
      center_points.push_back(p1);
      center_points.push_back(p2);
    }
    return center_points;
  }
};

class LaneletLengthCache
{
public:
  auto getLength(lanelet::Id lanelet_id)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("length of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[lanelet_id];
  }

  auto getLength(const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map) -> double
  {
    if (!exists(lanelet_id)) {
      appendData(
        lanelet_id, lanelet::utils::getLaneletLength2d(lanelet_map->laneletLayer.get(lanelet_id)));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[lanelet_id];
  }

  std::unordered_map<lanelet::Id, double> data_;
  std::mutex mutex_;

  auto exists(const lanelet::Id lanelet_id) -> bool
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.find(lanelet_id) != data_.end();
  }

  auto appendData(const lanelet::Id lanelet_id, double length) -> void
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[lanelet_id] = length;
  }
};

class LaneletMapCore
{
public:
  static auto routeCache() -> RouteCache &;
  static auto centerPointsCache() -> CenterPointsCache &;
  static auto laneletLengthCache() -> LaneletLengthCache &;

  static auto activate(const std::string & lanelet_map_path) -> void;
  static auto map() -> const lanelet::LaneletMapPtr &;
  static auto shoulderLanelets() -> const lanelet::ConstLanelets &;
  static auto vehicleRoutingGraph() -> const lanelet::routing::RoutingGraphConstPtr &;
  static auto pedestrianRoutingGraph() -> const lanelet::routing::RoutingGraphConstPtr &;
  static auto trafficRulesVehicle() -> const lanelet::traffic_rules::TrafficRulesPtr &;
  static auto trafficRulesPedestrian() -> const lanelet::traffic_rules::TrafficRulesPtr &;

private:
  LaneletMapCore(const std::filesystem::path & lanelet2_map_path);
  static LaneletMapCore & getInstance();

  auto calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
    -> std::vector<double>;

  auto resamplePoints(
    const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
    -> lanelet::BasicPoints3d;

  auto overwriteLaneletsCenterline() -> void;

  inline static std::unique_ptr<LaneletMapCore> instance{nullptr};
  inline static std::string lanelet_map_path_{""};
  inline static std::mutex mutex_;

  mutable RouteCache route_cache_;
  mutable CenterPointsCache center_points_cache_;
  mutable LaneletLengthCache lanelet_length_cache_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
};
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // LaneletMapCoreTRAFFIC_SIMULATOR__UTILS__LANELET_MAP_CORE_MEMORY_HPP_
