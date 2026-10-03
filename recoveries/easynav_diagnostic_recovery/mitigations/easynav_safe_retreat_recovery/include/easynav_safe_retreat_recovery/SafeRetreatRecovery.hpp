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

#include "easynav_diagnostic_recovery/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class SafeRetreatRecovery
 * @brief Level-1 movement mitigation: moves straight away from a too-close obstacle.
 *
 * Selected for diagnostics with hardware_id == "obstacle_proximity" (shared with
 * ObstacleTooCloseEvaluator, matched by string). Takes control of "cmd_vel"
 * (requires_control() == true) and commands a slow, straight motion each RT cycle, re-checking
 * the nearest-obstacle distance until it exceeds safe_distance.
 *
 * The direction, chosen on the first cycle and kept: backward if the nearest obstacle is ahead,
 * forward if it is behind or beside (a differential-drive robot cannot strafe; moving along its
 * axis also moves it away from an obstacle beside it). If that way is not clear
 * ("min_clearance" along the robot's corridor) and the obstacle is roughly beside (60-120 deg),
 * the other way is tried. It fails, stopped, when no way is clear, when the way gets blocked
 * while moving, or without perception.
 */
class SafeRetreatRecovery : public easynav_diagnostic_recovery::RecoveryMitigationBase
{
public:
  SafeRetreatRecovery() = default;
  ~SafeRetreatRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  easynav_diagnostic_recovery::RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Linear speed commanded while moving away (m/s, positive magnitude).
  double retreat_speed_ {0.15};

  /// @brief Distance (m) at which the retreat is considered complete.
  double safe_distance_ {0.6};

  /// @brief Points below it (robot frame) are not obstacles, e.g. the ground (m).
  double z_min_filter_ {0.0};

  /// @brief Points above it are not obstacles: the robot's height (robot_geometry).
  double robot_height_ {0.5};

  /// @brief The robot's radius (robot_geometry).
  double robot_radius_ {0.3};

  /// @brief Free distance (m) required along the way it moves.
  double min_clearance_ {0.05};

  /// @brief +1 forward, -1 backward, 0 not chosen yet (this episode).
  int direction_ {0};
};

}  // namespace easynav

#endif  // EASYNAV_SAFE_RETREAT_RECOVERY__SAFERETREATRECOVERY_HPP_
