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
/// \brief Declaration of the ObstacleTooCloseEvaluator plugin.

#ifndef EASYNAV_OBSTACLE_TOO_CLOSE_EVALUATOR__OBSTACLETOOCLOSEEVALUATOR_HPP_
#define EASYNAV_OBSTACLE_TOO_CLOSE_EVALUATOR__OBSTACLETOOCLOSEEVALUATOR_HPP_

#include <optional>

#include "rclcpp/time.hpp"

#include "easynav_core/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class ObstacleTooCloseEvaluator
 * @brief Level-1 recovery evaluator: diagnoses "stopped too close to an obstacle".
 *
 * Deliberately a *compound* condition, not just "is something close": it requires the robot to
 * already be (near) stationary before reporting ERROR. The RT-level CollisionSafetyReflex reacts
 * first and stops the robot; this evaluator must not fire while that stop is still happening
 * (still decelerating), or a movement mitigation like SafeRetreatRecovery could take over
 * mid-brake and substitute an unsafe motion for a controlled one. Rather than coupling to the
 * reflex's internal state, "already stopped" is checked against an independent, physically
 * measured signal (the robot's own velocity from "robot_pose"), so this works regardless of
 * *why* the robot stopped.
 *
 * "Stopped" must also be *sustained* for a short debounce window before it is trusted: the RT
 * and non-RT cycles run in parallel, so a single low-velocity sample could still be taken
 * mid-brake. Within that window this evaluator reports OK, not yet the real proximity check.
 */
class ObstacleTooCloseEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  ObstacleTooCloseEvaluator() = default;
  ~ObstacleTooCloseEvaluator() = default;

  void on_initialize() override;

protected:
  void update(NavState & nav_state) override;

private:
  /// @brief Distance (m) below which the robot is considered too close to operate normally.
  /// Deliberately more conservative (larger) than the level-0 reflex's own trigger distance.
  double safe_distance_ {0.6};

  /// @brief Below this linear speed (m/s), the robot is considered stopped.
  double linear_velocity_epsilon_ {0.02};

  /// @brief Below this angular speed (rad/s), the robot is considered stopped.
  double angular_velocity_epsilon_ {0.05};

  /// @brief Seconds the "stopped" condition must hold, uninterrupted, before it is trusted.
  double debounce_duration_ {0.2};

  /// @brief Timestamp since the robot has been continuously stopped, reset the moment it moves.
  std::optional<rclcpp::Time> stopped_since_;
};

}  // namespace easynav

#endif  // EASYNAV_OBSTACLE_TOO_CLOSE_EVALUATOR__OBSTACLETOOCLOSEEVALUATOR_HPP_
