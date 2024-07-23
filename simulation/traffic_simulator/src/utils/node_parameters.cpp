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

#include <traffic_simulator/utils/node_parameters.hpp>

namespace traffic_simulator
{
auto getOrigin(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
  -> geographic_msgs::msg::GeoPoint
{
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = getParameter(node_parameters, "origin_latitude", 0.0);
  origin.longitude = getParameter(node_parameters, "origin_longitude", 0.0);
  return origin;
}
}  // namespace traffic_simulator
