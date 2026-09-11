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
/// \brief Implementation of the HumanAssistanceRecovery class.

#include "easynav_human_assistance_recovery/HumanAssistanceRecovery.hpp"

namespace easynav
{

void HumanAssistanceRecovery::on_initialize()
{
}

bool HumanAssistanceRecovery::can_handle(
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void HumanAssistanceRecovery::on_start(NavState & nav_state)
{
  // Selection (and so on_start()) always runs from RecoveryManagerNode::cycle(), the same
  // non-RT thread the evaluators that wrote "diagnostics" run on, so a plain get() is safe
  // here. This is the observable "asking a human for help" signal §5.15 requires; how it
  // reaches an actual person (app, light, sound) is a deployment decision, not this plugin's.
  std::string summary;
  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    const auto & status = nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      if (!summary.empty()) {summary += ", ";}
      summary += key + " (" + status.message + ")";
    }
  }

  RCLCPP_ERROR(
    get_node()->get_logger(),
    "HumanAssistanceRecovery [%s]: no other mitigation resolved this — requesting human "
    "assistance for: %s",
    get_plugin_name().c_str(), summary.empty() ? "unknown" : summary.c_str());
}

RecoveryStatus HumanAssistanceRecovery::on_cycle(NavState & nav_state)
{
  // Runs on the RT thread (requires_control()), reading a group written on the non-RT thread —
  // get_safe() (a snapshot copy) is required here, not get(). See NavState's own
  // get()/get_safe() guidance.
  bool any_error = false;
  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    const auto status = nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      any_error = true;
      break;
    }
  }

  if (!any_error) {
    // Whatever was wrong is gone — presumably a human fixed it. Unlike the retired
    // NotifyAndHoldRecovery, this does not touch GoalManager: the mission was never failed,
    // just paused, so resuming is simply returning control_owner to the nominal controller.
    stop_robot(nav_state);
    return RecoveryStatus::SUCCEEDED;
  }

  RCLCPP_ERROR_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 10000,
    "HumanAssistanceRecovery [%s]: still waiting for human assistance",
    get_plugin_name().c_str());

  // No timeout: this is the last resort, it waits as long as it takes.
  stop_robot(nav_state);
  return RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::HumanAssistanceRecovery, easynav::RecoveryMitigationBase)
