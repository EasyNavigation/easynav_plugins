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
/// \brief Implementation of the ControllerStuckEvaluator class.

#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_controller_stuck_evaluator/ControllerStuckEvaluator.hpp"

namespace easynav
{

void ControllerStuckEvaluator::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(
    plugin_name + ".linear_velocity_threshold", linear_velocity_threshold_);
  node->declare_parameter<double>(
    plugin_name + ".progress_distance_threshold", progress_distance_threshold_);
  node->declare_parameter<double>(plugin_name + ".stuck_time_threshold", stuck_time_threshold_);

  node->get_parameter<double>(
    plugin_name + ".linear_velocity_threshold", linear_velocity_threshold_);
  node->get_parameter<double>(
    plugin_name + ".progress_distance_threshold", progress_distance_threshold_);
  node->get_parameter<double>(plugin_name + ".stuck_time_threshold", stuck_time_threshold_);
}

void ControllerStuckEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_plugin_name();
  status.hardware_id = "controller_stuck";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "making progress";

  // While paused, the controller plugin keeps writing a non-trivial "cmd_vel" into NavState as
  // if still navigating (only the value actually published to the robot is zeroed elsewhere).
  // Unlike the control_owner freeze below, freezing the reference here would not be enough: the
  // robot genuinely does not move while paused, so a frozen reference_time_ would already be
  // older than stuck_time_threshold_ once navigation resumes, firing an immediate false ERROR.
  // Re-arm the reference every cycle instead, so a full window of real non-progress is required
  // again after resuming.
  if (nav_state.has("navigation_paused") && nav_state.get<bool>("navigation_paused")) {
    status.message = "navigation paused";
    if (nav_state.has("robot_pose")) {
      const auto odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
      reference_position_ = {odom.pose.pose.position.x, odom.pose.pose.position.y};
    } else {
      reference_position_.reset();
    }
    reference_time_ = get_node()->now();
    publish_diagnostic(nav_state, status);
    return;
  }

  // An evaluator that watches cmd_vel/the controller must not evaluate while a recovery
  // mitigation owns control_owner, or it would self-diagnose the recovery itself as a new
  // failure. Freeze reference_position_ while this holds, so the first cycle back under
  // "controller" compares against a possibly-stale reference — any real movement made by the
  // mitigation already counts as progress then.
  if (nav_state.has("control_owner") &&
    nav_state.get<std::string>("control_owner") != "controller")
  {
    status.message = "a recovery mitigation owns control_owner";
    publish_diagnostic(nav_state, status);
    return;
  }

  // Do not compete with (or second-guess) a level-0 safety reflex: if the robot isn't moving
  // because a reflex is holding it back from something real, that is not "stuck".
  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    const auto & reflex_status = nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (reflex_status.hardware_id == "safety_reflex" &&
      reflex_status.level != diagnostic_msgs::msg::DiagnosticStatus::OK)
    {
      status.message = "a safety reflex is intervening";
      publish_diagnostic(nav_state, status);
      return;
    }
  }

  // Without an active goal, nothing was expected to make progress in the first place.
  const bool has_active_goal = nav_state.has("goals") &&
    !nav_state.get<nav_msgs::msg::Goals>("goals").goals.empty();
  if (!has_active_goal) {
    status.message = "no active goal";
    publish_diagnostic(nav_state, status);
    return;
  }

  if (!nav_state.has("cmd_vel")) {
    status.message = "no cmd_vel yet";
    publish_diagnostic(nav_state, status);
    return;
  }

  // "cmd_vel" and "robot_pose" are written from the RT cycle; this evaluator runs on
  // RecoveryManagerNode's non-RT cycle, so get_safe() (a snapshot copy) is required for both,
  // not get(). See NavState's own get()/get_safe() guidance.
  const auto cmd = nav_state.get_safe<geometry_msgs::msg::TwistStamped>("cmd_vel");
  const double commanded_speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
  if (commanded_speed < linear_velocity_threshold_) {
    status.message = "not commanded to move";
    publish_diagnostic(nav_state, status);
    return;
  }

  if (!nav_state.has("robot_pose")) {
    status.message = "no robot_pose yet";
    publish_diagnostic(nav_state, status);
    return;
  }

  const auto odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
  const double x = odom.pose.pose.position.x;
  const double y = odom.pose.pose.position.y;
  const rclcpp::Time now = get_node()->now();

  if (!reference_position_.has_value()) {
    reference_position_ = {x, y};
    reference_time_ = now;
    publish_diagnostic(nav_state, status);
    return;
  }

  const double dx = x - reference_position_->first;
  const double dy = y - reference_position_->second;
  if (std::hypot(dx, dy) > progress_distance_threshold_) {
    reference_position_ = {x, y};
    reference_time_ = now;
    publish_diagnostic(nav_state, status);
    return;
  }

  const double stuck_for = (now - reference_time_).seconds();
  if (stuck_for >= stuck_time_threshold_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "commanded to move but not making progress";

    diagnostic_msgs::msg::KeyValue duration_kv;
    duration_kv.key = "stuck_duration";
    duration_kv.value = std::to_string(stuck_for);
    status.values.push_back(duration_kv);
  }

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::ControllerStuckEvaluator, easynav::RecoveryEvaluatorBase)
