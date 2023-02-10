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

#ifndef OPENSCENARIO_INTERPRETER__PARAMETER_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__PARAMETER_DISTRIBUTION_HPP_

#include <unordered_map>
#include <vector>

namespace openscenario_interpreter
{
// data container types of distribution
using ParameterList = std::unordered_map<std::string, Object>;
using ParameterListSharedPtr = std::shared_ptr<ParameterList>;
using ParameterDistribution = std::vector<ParameterListSharedPtr>;
using SingleUnnamedParameterDistribution = std::vector<Object>;
struct SingleParameterDistribution
{
  std::string name;
  SingleUnnamedParameterDistribution distribution;
};

// generator types distribution
struct SingleParameterDistributionBase
{
  virtual auto derive() -> SingleUnnamedParameterDistribution = 0;
};

struct MultiParameterDistributionBase
{
  virtual auto derive() -> ParameterDistribution = 0;
};

// container types of distribution data generator
struct SingleParameterDistributionContainer
{
  virtual auto derive() -> SingleParameterDistribution = 0;
};

struct MultiParameterDistributionContainer
{
  virtual auto derive() -> ParameterDistribution = 0;
};

auto mergeParameterDistributionImpl(
  ParameterDistribution && distribution, SingleParameterDistribution && single_distribution)
  -> ParameterDistribution;

auto mergeParameterDistributionImpl(
  ParameterDistribution && distribution, ParameterDistribution && additional_distribution)
  -> ParameterDistribution;

template <typename DistributionT>
ParameterDistribution mergeParameterDistribution(
  ParameterDistribution && distribution, DistributionT && x)
{
  return mergeParameterDistributionImpl(distribution, x.derive());
}

template <typename DistributionT, typename... Ts>
ParameterDistribution mergeParameterDistribution(
  ParameterDistribution && distribution, DistributionT && x, Ts &&... xs)
{
  return mergeParameterDistribution(
    distribution, std::forward<decltype(x)>(x),
    mergeParameterDistribution(distribution, std::forward<decltype(xs)>(xs)...));
}

template <typename DistributionT>
ParameterDistribution mergeParameterDistribution(
  ParameterDistribution && distribution, const std::list<DistributionT> & distribution_list)
{
  for (const auto & additional_distribution : distribution_list) {
    distribution = mergeParameterDistributionImpl(
      distribution,
      apply(
        [](const auto & distribution) { return distribution.derive(); }, additional_distribution));
  }
  return distribution;
}
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__PARAMETER_DISTRIBUTION_HPP_
