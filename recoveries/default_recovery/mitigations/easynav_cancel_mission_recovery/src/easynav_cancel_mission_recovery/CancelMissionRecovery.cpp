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
/// \brief Implementation of the CancelMissionRecovery class.

#include "easynav_cancel_mission_recovery/CancelMissionRecovery.hpp"

namespace easynav
{

void CancelMissionRecovery::on_initialize()
{
}

bool CancelMissionRecovery::can_handle(
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void CancelMissionRecovery::on_start(NavState & nav_state)
{
  report(
    nav_state, rcl_interfaces::msg::Log::ERROR,
    "CancelMissionRecovery [" + get_plugin_name() +
    "]: nothing else resolved this — cancelling the active mission");

  nav_state.set("mission_cancel_requested", true);
}

RecoveryStatus CancelMissionRecovery::on_cycle(NavState & nav_state)
{
  // Runs on the non-RT cycle (requires_control() is false), the same thread GoalManager's
  // update() runs on, so a plain get() is safe here — no cross-thread read.
  const bool still_pending = nav_state.has("mission_cancel_requested") &&
    nav_state.get<bool>("mission_cancel_requested");

  if (still_pending) {
    // GoalManager::update() has not consumed the request yet — one non-RT cycle of latency,
    // same as every other one-shot NavState signal in this design.
    return RecoveryStatus::RUNNING;
  }

  // FAILED, not SUCCEEDED: cancelling the mission does not fix whatever diagnostic triggered it
  // (e.g. AMCL stays diverged), so claiming success would make this eligible for immediate
  // reselection every cycle, re-cancelling and re-logging forever.
  return RecoveryStatus::FAILED;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CancelMissionRecovery, easynav::RecoveryMitigationBase)
