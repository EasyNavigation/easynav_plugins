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
/// \brief Implementation of the NoPathEvaluator class.

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"

#include "easynav_no_path_evaluator/NoPathEvaluator.hpp"

namespace easynav
{

void NoPathEvaluator::on_initialize()
{
}

void NoPathEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_plugin_name();
  status.hardware_id = "planner";

  // "goals" is written by GoalManager on the same non-RT thread this evaluator runs on, so a
  // plain get() is safe here too. No active goal means there is nothing to plan toward, so a
  // missing/empty "path" is expected, not something this evaluator should flag.
  const bool has_active_goal = nav_state.has("goals") &&
    !nav_state.get<nav_msgs::msg::Goals>("goals").goals.empty();

  if (!has_active_goal) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "no active goal";
  } else if (!nav_state.has("path")) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "no path published yet";
  } else {
    // Both the planner and this evaluator run on SystemNode's single non-RT cycle thread, so
    // a plain get() is safe here and avoids copying a potentially large Path (see NavState's
    // own get()/get_safe() guidance in NavState.hpp).
    const auto & path = nav_state.get<nav_msgs::msg::Path>("path");
    if (path.poses.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "planner produced an empty path";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "path available";
    }
  }

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::NoPathEvaluator, easynav::RecoveryEvaluatorBase)
