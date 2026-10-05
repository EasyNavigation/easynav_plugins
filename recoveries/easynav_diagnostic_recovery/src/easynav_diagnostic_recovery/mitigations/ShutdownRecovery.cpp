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
/// \brief Implementation of the ShutdownRecovery class.

#include <algorithm>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_diagnostic_recovery/mitigations/ShutdownRecovery.hpp"

namespace easynav
{

void ShutdownRecovery::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  if (!node->has_parameter(plugin_name + ".handled_hardware_ids")) {
    node->declare_parameter<std::vector<std::string>>(
      plugin_name + ".handled_hardware_ids", handled_hardware_ids_);
  }
  node->get_parameter<std::vector<std::string>>(
    plugin_name + ".handled_hardware_ids", handled_hardware_ids_);
}

bool ShutdownRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
         std::find(
    handled_hardware_ids_.begin(), handled_hardware_ids_.end(), status.hardware_id) !=
         handled_hardware_ids_.end();
}

void ShutdownRecovery::on_start(NavState & nav_state)
{
  // The group also holds safety-reflex diagnostics, written from the RT cycle: get_safe().
  std::string reason;
  std::vector<std::string> names;
  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    const auto status = nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (!can_handle(status)) {continue;}
    if (!reason.empty()) {reason += "\n";}
    reason += status.name + ": " + status.message;
    names.push_back(status.name);
  }
  if (reason.empty()) {reason = "unrecoverable diagnostic";}

  std::string names_str;
  for (const auto & name : names) {
    names_str += (names_str.empty() ? "" : ", ") + name;
  }
  // The full reason goes to the blackboard, /diagnostics and SystemNode's final report; the log
  // line only says what is happening and why.
  report(
    nav_state, rcl_interfaces::msg::Log::FATAL,
    "ShutdownRecovery [" + get_plugin_name() + "]: terminating EasyNav (unrecoverable: " +
    names_str + ")");

  nav_state.set("system_shutdown_reason", reason);
  nav_state.set("system_shutdown_requested", true);

  const bool has_active_goal = nav_state.has("goals") &&
    !nav_state.get<nav_msgs::msg::Goals>("goals").goals.empty();
  if (has_active_goal) {
    nav_state.set("mission_cancel_requested", true);
  }
}

easynav_diagnostic_recovery::RecoveryStatus ShutdownRecovery::on_cycle(NavState & nav_state)
{
  geometry_msgs::msg::TwistStamped cmd;
  if (auto node = get_node()) {
    cmd.header.stamp = node->now();
  }
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;

  command_velocity(nav_state, cmd);
  return easynav_diagnostic_recovery::RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::ShutdownRecovery,
  easynav_diagnostic_recovery::RecoveryMitigationBase)
