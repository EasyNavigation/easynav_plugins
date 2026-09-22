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
/// \brief Declaration of the ControllerStuckEvaluator plugin.

#ifndef EASYNAV_CONTROLLER_STUCK_EVALUATOR__CONTROLLERSTUCKEVALUATOR_HPP_
#define EASYNAV_CONTROLLER_STUCK_EVALUATOR__CONTROLLERSTUCKEVALUATOR_HPP_

#include <optional>

#include "rclcpp/time.hpp"

#include "easynav_core/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class ControllerStuckEvaluator
 * @brief Level-1 recovery evaluator: diagnoses "commanded to move, but not making progress".
 *
 * Compares "robot_pose" across a debounce window while "cmd_vel" commands non-trivial motion;
 * if the robot barely moves for long enough, reports ERROR with hardware_id "controller_stuck".
 *
 * Four preconditions must hold before this evaluator judges anything, each skipping to OK
 * otherwise:
 * - Navigation must not be paused: the controller plugin keeps writing a non-trivial "cmd_vel"
 *   into NavState while paused, oblivious to it (only the value actually published to the robot
 *   is zeroed, elsewhere). Unlike the other preconditions, this one re-arms
 *   reference_position_/reference_time_ every cycle instead of freezing them, since the robot
 *   genuinely does not move while paused and a frozen reference would fire a false ERROR the
 *   instant navigation resumes.
 * - "control_owner" must be "controller": an evaluator that watches cmd_vel/the controller must
 *   not evaluate while a recovery mitigation owns control, or it would self-diagnose the very
 *   recovery it is part of as a new failure. This one freezes its progress-tracking state
 *   instead (see the .cpp for why the two preconditions need different treatment).
 * - No SafetyReflexBase-derived reflex may currently be intervening — if the robot isn't moving
 *   because the level-0 reflex is holding it back from a real obstacle, that is not "stuck".
 * - There must be an active goal ("goals" non-empty) — without one, nothing was expected to
 *   make progress in the first place.
 */
class ControllerStuckEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  ControllerStuckEvaluator() = default;
  ~ControllerStuckEvaluator() = default;

  void on_initialize() override;

protected:
  void update(NavState & nav_state) override;

private:
  /// @brief Below this commanded linear speed (m/s), the robot is not considered "commanded to
  /// move" at all.
  double linear_velocity_threshold_ {0.02};

  /// @brief Minimum displacement (m) since the reference position to count as "progress".
  double progress_distance_threshold_ {0.05};

  /// @brief Seconds without progress, while commanded to move, before reporting ERROR.
  double stuck_time_threshold_ {2.0};

  /// @brief Last position considered "progress" (x, y), and when it was recorded.
  std::optional<std::pair<double, double>> reference_position_;
  rclcpp::Time reference_time_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER_STUCK_EVALUATOR__CONTROLLERSTUCKEVALUATOR_HPP_
