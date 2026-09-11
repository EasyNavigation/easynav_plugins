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
/// \brief Implementation of the ForceReplanRecovery class.

#include "easynav_force_replan_recovery/ForceReplanRecovery.hpp"

namespace easynav
{

bool ForceReplanRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.hardware_id == "planner" &&
         status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void ForceReplanRecovery::on_start(NavState & nav_state)
{
  RCLCPP_WARN(
    get_node()->get_logger(), "ForceReplanRecovery [%s]: forcing an immediate replan",
    get_plugin_name().c_str());

  nav_state.set("force_replan_requested", true);
}

RecoveryStatus ForceReplanRecovery::on_cycle(NavState & nav_state)
{
  // SystemNode consumes "force_replan_requested" (sets it back to false) in the same
  // non-RT cycle it forces PlannerNode to replan; by the time cycle() reaches
  // RecoveryManagerNode again (next non-RT tick), the request has already been handed off.
  const bool still_pending = nav_state.has("force_replan_requested") &&
    nav_state.get<bool>("force_replan_requested");

  return still_pending ? RecoveryStatus::RUNNING : RecoveryStatus::SUCCEEDED;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::ForceReplanRecovery, easynav::RecoveryMitigationBase)
