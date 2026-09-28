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
/// \brief Declaration of the AdvanceRecovery plugin.

#ifndef EASYNAV_ADVANCE_RECOVERY__ADVANCERECOVERY_HPP_
#define EASYNAV_ADVANCE_RECOVERY__ADVANCERECOVERY_HPP_

#include <optional>

#include "rclcpp/time.hpp"

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class AdvanceRecovery
 * @brief Level-1 movement mitigation: advances a short distance when the robot is stuck.
 *
 * Selected for diagnostics with hardware_id == "controller_stuck". Takes control of "cmd_vel"
 * (requires_control() == true) and commands a slow, straight-forward motion — same single gate
 * as any other producer of "cmd_vel" (CollisionSafetyReflex), so "only if there is no obstacle"
 * is enforced by the level-0 reflex, not duplicated here.
 *
 * Deliberately never reports SUCCEEDED as "the stuck condition is fixed": completing one
 * advance only means this activation is done, not that the underlying cause is gone. If it
 * recurs, ControllerStuckEvaluator will simply diagnose it again and this mitigation activates
 * again — by design, "advances a bit" repeatedly for as long as the problem keeps reappearing.
 * It only gives up (FAILED, so RecoveryManagerNode escalates to the next candidate) once the
 * *total* time spent on this recurring episode — summed across every activation — exceeds
 * "escalate_after". A long enough gap between activations resets that clock: see on_start().
 */
class AdvanceRecovery : public easynav::RecoveryMitigationBase
{
public:
  AdvanceRecovery() = default;
  ~AdvanceRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;
  void on_stop(NavState & nav_state) override;

private:
  /// @brief Distance (m) to advance before considering one activation complete.
  double advance_distance_ {0.3};

  /// @brief Forward linear speed commanded while advancing (m/s, "cautious").
  double advance_speed_ {0.1};

  /// @brief Total time (s), summed across activations of the same episode, before giving up.
  double escalate_after_ {15.0};

  /// @brief Gap (s) since this mitigation last stopped beyond which the next activation is
  /// treated as a new episode instead of a continuation (resets the escalation clock).
  double episode_gap_ {10.0};

  /// @brief When the current episode started (first activation, or the first one after a gap
  /// longer than episode_gap_). Reset once escalated.
  std::optional<rclcpp::Time> episode_start_;

  /// @brief When this mitigation last stopped (SUCCEEDED or FAILED), to measure the gap in the
  /// next on_start().
  std::optional<rclcpp::Time> last_stop_time_;

  /// @brief Robot position (x, y) when the current activation started, to measure this
  /// activation's own advance distance.
  std::pair<double, double> start_position_ {0.0, 0.0};
};

}  // namespace easynav

#endif  // EASYNAV_ADVANCE_RECOVERY__ADVANCERECOVERY_HPP_
