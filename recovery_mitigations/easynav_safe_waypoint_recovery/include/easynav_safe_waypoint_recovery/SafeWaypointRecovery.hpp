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
/// \brief Declaration of the SafeWaypointRecovery plugin.

#ifndef EASYNAV_SAFE_WAYPOINT_RECOVERY__SAFEWAYPOINTRECOVERY_HPP_
#define EASYNAV_SAFE_WAYPOINT_RECOVERY__SAFEWAYPOINTRECOVERY_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/publisher.hpp"

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class SafeWaypointRecovery
 * @brief Level-1, mission-scope mitigation: redirects the mission to a preconfigured safe
 * waypoint instead of the goal currently being pursued. See docs/recoveries_easynav.md §5.8
 * (Escenario 5 of TOMASys/SysSelf) and §5.11.
 *
 * Deliberately opt-in: can_handle() only ever returns true if a safe waypoint has actually
 * been configured (parameter "<instance>.enabled", default false) — see the "Decisión de
 * implementación" note in docs/recoveries_easynav_implementation.md, Fase 4, for why: with no
 * configured alternative there would be nothing useful to do, and letting can_handle() return
 * true anyway would either loop retrying a no-op or (worse) require NotifyAndHoldRecovery to
 * be listed before this one to ever get a turn, inverting the intended escalation order. This
 * way an unconfigured instance is simply skipped by RecoveryManagerNode's selection, falling
 * through to whatever is listed after it (typically NotifyAndHoldRecovery).
 *
 * Deviates from the literal design text in one respect: recoveries_easynav.md §5.8 says to
 * "use GoalManagerClient" to send the alternate goal. GoalManagerClient's constructor takes an
 * rclcpp::Node::SharedPtr, and its accept/preempt protocol runs across several non-RT cycles —
 * both awkward for a plugin living inside RecoveryManagerNode (an rclcpp_lifecycle::LifecycleNode)
 * that just wants to hand off one pose. Instead this publishes a single PoseStamped on the
 * "goal_pose" topic — the exact same "external, single-pose goal injection" channel GoalManager
 * already exposes for RViz/GUI use (GoalManager::comanded_pose_callback()), which already
 * handles preempting whatever goal is currently active. See
 * docs/recoveries_easynav_implementation.md for the full rationale.
 *
 * Does not requires_control(): PlannerNode/ControllerNode keep running normally against the
 * new goal once GoalManager accepts it, same as any other goal change.
 */
class SafeWaypointRecovery : public easynav::RecoveryMitigationBase
{
public:
  SafeWaypointRecovery() = default;
  ~SafeWaypointRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Opt-in flag: this mitigation only ever can_handle() when true.
  bool enabled_ {false};

  /// @brief Safe waypoint pose, in \ref frame_.
  double x_ {0.0};
  double y_ {0.0};
  double yaw_ {0.0};

  /// @brief Frame the safe waypoint is expressed in. Defaults to the workspace's configured
  /// map frame (read from RTTFBuffer) if left empty.
  std::string frame_;

  /// @brief Publisher for the single-pose goal injection channel ("goal_pose").
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
};

}  // namespace easynav

#endif  // EASYNAV_SAFE_WAYPOINT_RECOVERY__SAFEWAYPOINTRECOVERY_HPP_
