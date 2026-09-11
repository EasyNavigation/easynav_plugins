// Copyright 2026 Intelligent Robotics Lab
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

/// \file
/// \brief Declaration of the NoPathEvaluator plugin.

#ifndef EASYNAV_NO_PATH_EVALUATOR__NOPATHEVALUATOR_HPP_
#define EASYNAV_NO_PATH_EVALUATOR__NOPATHEVALUATOR_HPP_

#include "easynav_core/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class NoPathEvaluator
 * @brief Level-1 recovery evaluator that diagnoses a missing or empty planner path.
 *
 * See docs/recoveries_easynav.md, level 1, evaluator catalog. A generic, domain-agnostic
 * evaluator: it only knows about the "path" key that any PlannerMethodBase-derived plugin is
 * expected to produce, not about any specific planner's internals.
 */
class NoPathEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  NoPathEvaluator() = default;
  ~NoPathEvaluator() = default;

  void on_initialize() override;

protected:
  void update(NavState & nav_state) override;
};

}  // namespace easynav

#endif  // EASYNAV_NO_PATH_EVALUATOR__NOPATHEVALUATOR_HPP_
