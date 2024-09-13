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

#ifndef CONCEALER__TRANSITION_ASSERTION_HPP_
#define CONCEALER__TRANSITION_ASSERTION_HPP_

#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
template <typename Autoware>
struct TransitionAssertion
{
  const std::chrono::steady_clock::time_point start;

  const std::chrono::seconds initialize_duration;

  explicit TransitionAssertion()
  : start(std::chrono::steady_clock::now()), initialize_duration([]() {
      auto node = rclcpp::Node("get_parameter", "simulation");
      node.declare_parameter("initialize_duration", 0);
      return node.get_parameter("initialize_duration").as_int();
    }())
  {
  }

  auto makeTransitionError(const std::string & expected) const
  {
    const auto current_state = static_cast<const Autoware &>(*this).getAutowareStateName();
    return common::AutowareError(
      "Simulator waited for the Autoware state to transition to ", expected,
      ", but time is up. The current Autoware state is ",
      (current_state.empty() ? "NOT PUBLISHED YET" : current_state), ".");
  }

#define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE_WITH_INITIALIZE_TIME_LIMIT(STATE)               \
  template <typename Thunk = void (*)(), typename Interval = std::chrono::seconds>           \
  auto waitForAutowareStateToBe##STATE(                                                      \
    Thunk && thunk = [] {}, Interval interval = std::chrono::seconds(1))                     \
  {                                                                                          \
    for (thunk(); not static_cast<const Autoware &>(*this).isStopRequested() and             \
                  not static_cast<const Autoware &>(*this).is##STATE();                      \
         rclcpp::GenericRate<std::chrono::steady_clock>(interval).sleep()) {                 \
      if (auto now = std::chrono::steady_clock::now(); start + initialize_duration <= now) { \
        throw makeTransitionError(#STATE);                                                   \
      } else {                                                                               \
        thunk();                                                                             \
      }                                                                                      \
    }                                                                                        \
  }                                                                                          \
  static_assert(true)

#define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(STATE)                                  \
  template <typename Thunk = void (*)()>                                             \
  auto waitForAutowareStateToBe##STATE(                                              \
    Thunk && thunk = [] {}, std::chrono::seconds interval = std::chrono::seconds(1)) \
  {                                                                                  \
    for (thunk(); not static_cast<const Autoware &>(*this).isStopRequested() and     \
                  not static_cast<const Autoware &>(*this).is##STATE();              \
         rclcpp::GenericRate<std::chrono::steady_clock>(interval).sleep()) {         \
      thunk();                                                                       \
    }                                                                                \
  }                                                                                  \
  static_assert(true)

  /*
     NOTE: The time limit must be ignored in waitForAutowareStateToBeDriving()
     because the current implementation attempts to transition to Driving state
     after initialize_duration seconds have elapsed.
  */
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE_WITH_INITIALIZE_TIME_LIMIT(Initializing);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE_WITH_INITIALIZE_TIME_LIMIT(WaitingForRoute);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE_WITH_INITIALIZE_TIME_LIMIT(Planning);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE_WITH_INITIALIZE_TIME_LIMIT(WaitingForEngage);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Driving);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(ArrivedGoal);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Finalizing);

#undef DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE
};
}  // namespace concealer

#endif  // CONCEALER__TRANSITION_ASSERTION_HPP_
