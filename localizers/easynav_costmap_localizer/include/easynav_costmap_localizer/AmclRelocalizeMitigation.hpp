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
/// \brief Declaration of the AmclRelocalizeMitigation plugin.

#ifndef EASYNAV_COSTMAP_LOCALIZER__AMCLRELOCALIZEMITIGATION_HPP_
#define EASYNAV_COSTMAP_LOCALIZER__AMCLRELOCALIZEMITIGATION_HPP_

#include "rclcpp/time.hpp"

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class AmclRelocalizeMitigation
 * @brief Level-1 movement mitigation: rotates in place to help AMCL relocalize.
 *
 * See docs/recoveries_easynav.md §5.9. Selected for diagnostics with
 * hardware_id == "localizer.amcl" (the vocabulary shared with AmclConvergenceEvaluator, in the
 * same package). Takes control of "cmd_vel" (requires_control() == true) and rotates slowly in
 * place each RT cycle, re-checking the same covariance trace the evaluator reads, until it
 * drops back under threshold — or until `timeout` elapses without that happening, at which
 * point it gives up (stops, returns FAILED) rather than spinning forever. RecoveryManagerNode
 * then excludes this mitigation from being reselected for the same diagnostic occurrence (see
 * docs/recoveries_easynav_implementation.md, Fase 5).
 */
class AmclRelocalizeMitigation : public easynav::RecoveryMitigationBase
{
public:
  AmclRelocalizeMitigation() = default;
  ~AmclRelocalizeMitigation() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Angular speed commanded while rotating in place (rad/s, "slowly").
  double rotation_speed_ {0.3};

  /// @brief Seconds to keep rotating before giving up. See the class doc comment.
  double timeout_ {5.0};

  /// @brief Same threshold as AmclConvergenceEvaluator by default — a deliberate simplification
  /// (no hysteresis between evaluator and mitigator thresholds yet), same as already noted for
  /// ObstacleTooCloseEvaluator/SafeRetreatRecovery.
  double covariance_threshold_ {1.0};

  rclcpp::Time start_time_;
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__AMCLRELOCALIZEMITIGATION_HPP_
