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
/// \brief Implementation of the NotifyAndHoldRecovery class.

#include "nav_msgs/msg/goals.hpp"

#include "easynav_notify_and_hold_recovery/NotifyAndHoldRecovery.hpp"

namespace easynav
{

void NotifyAndHoldRecovery::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<std::string>(plugin_name + ".report_as", report_as_);
  node->get_parameter<std::string>(plugin_name + ".report_as", report_as_);

  if (report_as_ != "failed" && report_as_ != "error") {
    RCLCPP_WARN(
      node->get_logger(),
      "NotifyAndHoldRecovery [%s]: invalid report_as '%s' (must be 'failed' or 'error'), "
      "defaulting to 'failed'", plugin_name.c_str(), report_as_.c_str());
    report_as_ = "failed";
  }
}

bool NotifyAndHoldRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  // Unconditional catch-all: see the class doc comment. This is meant to be the last entry in
  // "mitigation_types", so it is only ever reached for a diagnostic nothing earlier-listed
  // could handle.
  return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void NotifyAndHoldRecovery::on_start(NavState & nav_state)
{
  std::string reason =
    "Recovery escalated to mission level: no autonomous mitigation resolved the situation";
  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {
      continue;
    }
    const auto & status = nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      reason = "[" + status.name + "] (" + status.hardware_id + "): " + status.message;
      break;
    }
  }

  RCLCPP_ERROR(
    get_node()->get_logger(),
    "NotifyAndHoldRecovery [%s]: %s -- reporting mission as '%s' and holding",
    get_plugin_name().c_str(), reason.c_str(), report_as_.c_str());

  // Handed to SystemNode::system_cycle(), the only place with a real GoalManager instance to
  // call set_failed()/set_error() on. See docs/recoveries_easynav_implementation.md, Fase 4.
  nav_state.set("goal_manager_request", report_as_);
  nav_state.set("goal_manager_reason", reason);
}

RecoveryStatus NotifyAndHoldRecovery::on_cycle(NavState & nav_state)
{
  stop_robot(nav_state);

  // "goals" crosses the RT/non-RT boundary (written by GoalManager::update() on the non-RT
  // thread), so use get_safe() here, in the RT thread. See NavState.hpp's own guidance.
  const bool has_active_goal = nav_state.has("goals") &&
    !nav_state.get_safe<nav_msgs::msg::Goals>("goals").goals.empty();

  // Someone sent a new goal after the failure: release control_owner and let the nominal
  // controller resume against it. Otherwise, keep holding (RUNNING) indefinitely.
  return has_active_goal ? RecoveryStatus::SUCCEEDED : RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::NotifyAndHoldRecovery, easynav::RecoveryMitigationBase)
