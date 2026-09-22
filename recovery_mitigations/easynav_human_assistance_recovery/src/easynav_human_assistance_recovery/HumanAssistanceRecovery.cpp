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
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".timeout", timeout_);
  node->get_parameter<double>(plugin_name + ".timeout", timeout_);
}

bool HumanAssistanceRecovery::can_handle(
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void HumanAssistanceRecovery::on_start(NavState & nav_state)
{
  // Selection (and so on_start()) always runs from RecoveryManagerNode::cycle(), the same
  // non-RT thread the evaluators that wrote "diagnostics" run on, so a plain get() is safe here.
  std::string summary;
  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    const auto & status = nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      if (!summary.empty()) {summary += ", ";}
      summary += key + " (" + status.message + ")";
    }
  }

  report(
    nav_state, rcl_interfaces::msg::Log::ERROR,
    "HumanAssistanceRecovery [" + get_plugin_name() + "]: no other mitigation resolved this — "
    "requesting human assistance for: " + (summary.empty() ? "unknown" : summary));

  last_wait_report_.reset();
  if (timeout_ > 0.0) {
    start_time_ = get_node()->now();
  }
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

  if (timeout_ > 0.0 && (get_node()->now() - start_time_).seconds() >= timeout_) {
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "HumanAssistanceRecovery [" + get_plugin_name() + "]: no human response after " +
      std::to_string(timeout_) + " s, giving up");
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }

  // Manual throttle (replaces RCLCPP_ERROR_THROTTLE): report() only keeps the single latest
  // entry, so calling it every RT cycle would still need throttling to avoid flooding it.
  const rclcpp::Time now = get_node()->now();
  if (!last_wait_report_.has_value() ||
    (now - *last_wait_report_).seconds() >= wait_report_period_)
  {
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "HumanAssistanceRecovery [" + get_plugin_name() + "]: still waiting for human assistance");
    last_wait_report_ = now;
  }

  stop_robot(nav_state);
  return RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::HumanAssistanceRecovery, easynav::RecoveryMitigationBase)
