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

#include <random>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"

#include "easynav_diagnostic_recovery/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class AmclRelocalizeMitigation
 * @brief Level-1 mitigation: resets AMCL's pose around where the robot was when it got lost.
 *
 * Selected for diagnostics with hardware_id == "localizer.amcl" (shared with
 * AmclConvergenceEvaluator, in the same package). Holds the robot still (requires_control())
 * and publishes an initial pose on "initialpose", with RViz2's "2D Pose Estimate" covariance, up
 * to `attempts` times:
 *  - the first one at the robot's pose when the recovery started;
 *  - the next ones at that pose moved up to `max_offset` m in a random direction.
 *
 * Each attempt has `attempt_timeout` s to converge, by the evaluator's criteria: the covariance
 * trace under `covariance_threshold`, and the match score and quality ratio (if reported) at
 * least `match_threshold` and `quality_ratio_threshold`. Only checked after the localizer has had
 * time to correct the new pose (the scores before it are of the old one). SUCCEEDED when an
 * attempt converges, FAILED when none does, or at once if there is no robot pose to reset to.
 *
 * If AMCL has already converged when it starts (the diagnostic that selected it was evaluated
 * before), it resets nothing: SUCCEEDED.
 */
class AmclRelocalizeMitigation : public easynav_diagnostic_recovery::RecoveryMitigationBase
{
public:
  AmclRelocalizeMitigation() = default;
  ~AmclRelocalizeMitigation() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  easynav_diagnostic_recovery::RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Publishes the pose of attempt `attempt_` and restarts its clock.
  void start_attempt(NavState & nav_state);

  /// @brief Whether AMCL has converged (see the class doc comment).
  bool converged(const NavState & nav_state) const;

  /// @brief Initial poses tried, the first one without offset.
  int attempts_ {5};

  /// @brief Seconds each attempt has to converge.
  double attempt_timeout_ {5.0};

  /// @brief Largest offset (m) of the attempts after the first one.
  double max_offset_ {1.0};

  /// @brief Same threshold as AmclConvergenceEvaluator by default — a deliberate simplification
  /// (no hysteresis between evaluator and mitigator thresholds yet), same as already noted for
  /// ObstacleTooCloseEvaluator/SafeRetreatRecovery.
  double covariance_threshold_ {1.0};

  /// @brief And, if the localizer reports it, the match score must be at least this (0: not
  /// checked). Same as AmclConvergenceEvaluator's by default.
  double match_threshold_ {0.5};

  /// @brief And the quality ratio at least this (0: not checked). Same as
  /// AmclConvergenceEvaluator's by default.
  double quality_ratio_threshold_ {0.5};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  std::mt19937 rng_ {std::random_device{}()};

  bool has_lost_pose_ {false};
  bool converged_at_start_ {false};  ///< Nothing to do: SUCCEEDED at the first cycle.
  geometry_msgs::msg::PoseWithCovarianceStamped lost_pose_;  ///< Where the robot got lost.
  int attempt_ {0};
  rclcpp::Time attempt_start_;
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__AMCLRELOCALIZEMITIGATION_HPP_
