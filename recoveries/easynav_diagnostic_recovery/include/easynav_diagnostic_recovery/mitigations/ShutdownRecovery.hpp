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
/// \brief Declaration of the ShutdownRecovery plugin.

#ifndef EASYNAV_DIAGNOSTIC_RECOVERY__MITIGATIONS__SHUTDOWNRECOVERY_HPP_
#define EASYNAV_DIAGNOSTIC_RECOVERY__MITIGATIONS__SHUTDOWNRECOVERY_HPP_

#include <string>
#include <vector>

#include "easynav_diagnostic_recovery/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class ShutdownRecovery
 * @brief Level-1 system-level mitigation: terminates EasyNav in an orderly way.
 *
 * For diagnostics that neither an automatic mitigation, a human, nor cancelling the active
 * mission can fix (e.g. a miswired ROS graph): EasyNav cannot navigate correctly as configured,
 * so it must stop, say why, and exit.
 *
 * Only handles ERROR diagnostics whose hardware_id is in "handled_hardware_ids" (default
 * {"ros_graph"}), so it never swallows diagnostics meant for the rest of the escalation ladder.
 *
 * Like CancelMissionRecovery it has no reference to SystemNode or GoalManager; it acts through
 * NavState:
 * - "system_shutdown_requested" (true) and "system_shutdown_reason" (the offending diagnostics):
 *   SystemNode picks them up, and the process leaves the Active state through the lifecycle's
 *   error path (Deactivating -> ErrorProcessing -> Finalized) and terminates.
 * - "mission_cancel_requested", only if there is an active goal, so its client is told why the
 *   mission ended.
 *
 * requires_control() is true: until the process ends, it holds the robot with a zero "cmd_vel".
 * on_cycle() never finishes — there is nothing to return control to.
 */
class ShutdownRecovery : public easynav_diagnostic_recovery::RecoveryMitigationBase
{
public:
  ShutdownRecovery() = default;
  ~ShutdownRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  easynav_diagnostic_recovery::RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief hardware_id values this mitigation handles (at ERROR level or above).
  std::vector<std::string> handled_hardware_ids_ {"ros_graph"};
};

}  // namespace easynav

#endif  // EASYNAV_DIAGNOSTIC_RECOVERY__MITIGATIONS__SHUTDOWNRECOVERY_HPP_
