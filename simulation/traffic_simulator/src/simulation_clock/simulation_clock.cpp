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

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

namespace traffic_simulator
{
SimulationClock::SimulationClock(double realtime_factor, double frame_rate)
: rclcpp::Clock(RCL_ROS_TIME),
  use_raw_clock(true),
  realtime_factor(realtime_factor),
  frame_rate(frame_rate),
  step_time_duration_(rclcpp::Duration::from_seconds(0)),
  initialized_(false),
  is_npc_logic_started_(false)
{
  initialized_ = true;
  initial_simulation_time_ = 0;
  current_simulation_time_ = 0;
  step_time_ = 1.0 / frame_rate * realtime_factor;
  step_time_duration_ = rclcpp::Duration::from_seconds(step_time_);
  time_on_initialize_ = now();
}

auto SimulationClock::update() -> void
{
  if (!initialized_) {
    THROW_SIMULATION_ERROR("SimulationClock has not been initialized yet.");
  }
  current_simulation_time_ = current_simulation_time_ + step_time_;
}

auto SimulationClock::getCurrentRosTimeAsMsg() -> rosgraph_msgs::msg::Clock
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = getCurrentRosTime();
  return clock;
}

auto SimulationClock::getCurrentRosTime() -> rclcpp::Time
{
  if (!initialized_) {
    THROW_SIMULATION_ERROR("SimulationClock has not been initialized yet.");
  }
  if (use_raw_clock) {
    return now();
  } else {
    return time_on_initialize_ +
           rclcpp::Duration::from_seconds(current_simulation_time_ - initial_simulation_time_);
  }
}

auto SimulationClock::onNpcLogicStart() -> void
{
  if (is_npc_logic_started_) {
    THROW_SIMULATION_ERROR(
      "npc logic is already started. Please check simulation clock instance was destroyed.");
  }
  is_npc_logic_started_ = true;
  scenario_time_offset_ = getCurrentSimulationTime();
}

auto SimulationClock::getCurrentScenarioTime() const -> double
{
  if (!is_npc_logic_started_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return getCurrentSimulationTime() - scenario_time_offset_;
}
}  // namespace traffic_simulator
