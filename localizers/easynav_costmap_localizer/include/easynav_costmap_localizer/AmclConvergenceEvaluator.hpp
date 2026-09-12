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
/// \brief Declaration of the AmclConvergenceEvaluator plugin.

#ifndef EASYNAV_COSTMAP_LOCALIZER__AMCLCONVERGENCEEVALUATOR_HPP_
#define EASYNAV_COSTMAP_LOCALIZER__AMCLCONVERGENCEEVALUATOR_HPP_

#include "easynav_core/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class AmclConvergenceEvaluator
 * @brief Level-1 recovery evaluator: diagnoses AMCL particle-filter divergence.
 *
 * Lives in the same package as AMCLLocalizer instead of the generic recovery_evaluators
 * catalog: only the author of the localizer plugin really knows that particle dispersion (here,
 * the trace of the pose covariance AMCLLocalizer already computes) is a good indicator of lost
 * convergence.
 *
 * Reads the fixed key "localizer.amcl.covariance_trace" (written by AMCLLocalizer from both
 * its RT and non-RT cycles) and publishes `hardware_id = "localizer.amcl"`, matched by
 * AmclRelocalizeMitigation in this same package/manifest — no compile-time dependency between
 * the two, only this agreed-upon diagnostic vocabulary.
 */
class AmclConvergenceEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  AmclConvergenceEvaluator() = default;
  ~AmclConvergenceEvaluator() = default;

  void on_initialize() override;

protected:
  void update(NavState & nav_state) override;

private:
  /// @brief Covariance trace (var_x + var_y + var_yaw) above which AMCL is considered
  /// diverged. Starting point only — depends on sensor/robot and needs tuning on the real
  /// platform.
  double covariance_threshold_ {1.0};
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__AMCLCONVERGENCEEVALUATOR_HPP_
