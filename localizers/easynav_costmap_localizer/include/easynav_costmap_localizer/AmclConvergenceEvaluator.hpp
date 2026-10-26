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

#include <optional>

#include "easynav_diagnostic_recovery/RecoveryEvaluatorBase.hpp"
#include "rclcpp/time.hpp"

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
 * Three signs, any of which is an ERROR:
 *  - "localizer.amcl.covariance_trace" above `covariance_threshold`: the particles spread out;
 *  - "localizer.amcl.match_score" below `match_threshold` for `debounce_duration`: the scans do
 *    not match the map where the filter places the robot (a compact cloud can still be wrong);
 *  - "localizer.amcl.quality_ratio" below `quality_ratio_threshold` for `debounce_duration`:
 *    the match has just dropped well below its usual level (e.g. the robot was kidnapped).
 *
 * All written by AMCLLocalizer. Publishes `hardware_id = "localizer.amcl"`, matched by
 * AmclRelocalizeMitigation in this same package/manifest — no compile-time dependency between
 * the two, only this agreed-upon diagnostic vocabulary.
 */
class AmclConvergenceEvaluator : public easynav_diagnostic_recovery::RecoveryEvaluatorBase
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

  /// @brief Best particle's score (fraction of scan points on obstacles of the map) below which
  /// the filter is lost. 0 disables it. Depends on the map, sensor and resolution: in the Kobuki
  /// playground, 0.75 localized and 0.25-0.35 lost (also converged at a wrong pose).
  double match_threshold_ {0.5};

  /// @brief Fast/slow average ratio of the scores below which the match has dropped.
  double quality_ratio_threshold_ {0.5};

  /// @brief Seconds the match or the ratio must stay low before it is an ERROR.
  double debounce_duration_ {2.0};

  /// @brief Since when the match or the ratio is low, if it is.
  std::optional<rclcpp::Time> low_since_;
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__AMCLCONVERGENCEEVALUATOR_HPP_
