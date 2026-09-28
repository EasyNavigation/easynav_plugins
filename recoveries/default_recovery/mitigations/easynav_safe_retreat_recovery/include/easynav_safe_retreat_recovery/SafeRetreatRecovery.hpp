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
/// \brief Declaration of the SafeRetreatRecovery plugin.

#ifndef EASYNAV_SAFE_RETREAT_RECOVERY__SAFERETREATRECOVERY_HPP_
#define EASYNAV_SAFE_RETREAT_RECOVERY__SAFERETREATRECOVERY_HPP_

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class SafeRetreatRecovery
 * @brief Level-1 movement mitigation: retreats straight back from a too-close obstacle.
 *
 * Selected for diagnostics with hardware_id == "obstacle_proximity" (shared with
 * ObstacleTooCloseEvaluator, matched by string). Takes control of "cmd_vel"
 * (requires_control() == true) and commands a slow, straight-backward motion each RT cycle,
 * re-checking the nearest-obstacle distance until it exceeds safe_distance.
 *
 * Only retreats straight back — correct when the obstacle is roughly ahead (an obstacle
 * appearing in the direction of travel), matching Nav2's own reverse-only BackUp behaviour and
 * the differential-drive robots this workspace targets (which cannot strafe anyway). If the
 * nearest obstacle is behind the robot instead, reversing would drive toward it, so on_cycle()
 * fails safely (stops, returns FAILED) instead of blindly reversing.
 */
class SafeRetreatRecovery : public easynav::RecoveryMitigationBase
{
public:
  SafeRetreatRecovery() = default;
  ~SafeRetreatRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Backward linear speed commanded while retreating (m/s, positive magnitude).
  double retreat_speed_ {0.15};

  /// @brief Distance (m) at which the retreat is considered complete.
  double safe_distance_ {0.6};
};

}  // namespace easynav

#endif  // EASYNAV_SAFE_RETREAT_RECOVERY__SAFERETREATRECOVERY_HPP_
