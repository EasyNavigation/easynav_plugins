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
/// \brief Declaration of the CancelMissionRecovery plugin.

#ifndef EASYNAV_CANCEL_MISSION_RECOVERY__CANCELMISSIONRECOVERY_HPP_
#define EASYNAV_CANCEL_MISSION_RECOVERY__CANCELMISSIONRECOVERY_HPP_

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class CancelMissionRecovery
 * @brief Level-1 mission-level last resort: cancels the active mission, reporting the error.
 *
 * The final rung of the escalation ladder: accepts any ERROR diagnostic no other mitigation
 * resolved. Meant to be configured with the highest priority number of all (tried last).
 *
 * Unlike other mitigations, this one does not move the robot — it has no reference to
 * GoalManager (only SystemNode does), so it asks for the mission to be cancelled via a one-shot
 * NavState signal ("mission_cancel_requested") that GoalManager::update() reads, resets, and
 * acts on. requires_control() is false: it only signals and waits, accepting one non-RT-cycle of
 * latency.
 *
 * on_cycle() reports FAILED once the signal is consumed, not SUCCEEDED: cancelling the mission
 * does not resolve the diagnostic that triggered it (e.g. an AMCL divergence stays a
 * divergence), so claiming success would make it immediately eligible for reselection and
 * re-cancel/re-log forever.
 */
class CancelMissionRecovery : public easynav::RecoveryMitigationBase
{
public:
  CancelMissionRecovery() = default;
  ~CancelMissionRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;
};

}  // namespace easynav

#endif  // EASYNAV_CANCEL_MISSION_RECOVERY__CANCELMISSIONRECOVERY_HPP_
