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
/// \brief Declaration of the NotifyAndHoldRecovery plugin.

#ifndef EASYNAV_NOTIFY_AND_HOLD_RECOVERY__NOTIFYANDHOLDRECOVERY_HPP_
#define EASYNAV_NOTIFY_AND_HOLD_RECOVERY__NOTIFYANDHOLDRECOVERY_HPP_

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class NotifyAndHoldRecovery
 * @brief Level-1, last-resort mitigation: stops the robot and reports the mission as
 * failed/errored via GoalManager, then holds until a new goal is accepted. See
 * docs/recoveries_easynav.md §5.8 and §5.11.
 *
 * Unconditional can_handle() (true for any diagnostic at level ERROR or above): this is meant
 * to be the last entry in "mitigation_types", so RecoveryManagerNode's selection (first
 * applicable candidate in list order, see RecoveryManagerNode.hpp) only ever reaches it for a
 * diagnostic no earlier-listed, more specific mitigation declared can_handle() for.
 *
 * requires_control() == true, unlike the rest of the Fase-4 catalog (see
 * docs/recoveries_easynav_implementation.md for why this was a necessary correction over the
 * design table, which did not mark it "(toma control_owner)"): stopping the robot with a single
 * zero-velocity write is not enough on its own, because nothing would then stop the nominal
 * controller from overwriting "cmd_vel" again on its very next RT cycle while still chasing the
 * (now abandoned) path. Taking control_owner and re-affirming zero velocity every RT cycle
 * (on_cycle() returns RecoveryStatus::RUNNING) is what actually makes the "hold" in the name
 * true. It releases control_owner (returns SUCCEEDED) only once GoalManager reports a new,
 * non-empty goal list (checked via NavState's "goals" key, already kept in sync by
 * GoalManager::update() — no dependency on easynav_system needed for that), i.e. once someone
 * has sent the robot a fresh goal after the failure.
 */
class NotifyAndHoldRecovery : public easynav::RecoveryMitigationBase
{
public:
  NotifyAndHoldRecovery() = default;
  ~NotifyAndHoldRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Whether to report the mission via GoalManager::set_failed() ("failed", the
  /// default) or GoalManager::set_error() ("error"). Parameter "<instance>.report_as".
  std::string report_as_ {"failed"};
};

}  // namespace easynav

#endif  // EASYNAV_NOTIFY_AND_HOLD_RECOVERY__NOTIFYANDHOLDRECOVERY_HPP_
