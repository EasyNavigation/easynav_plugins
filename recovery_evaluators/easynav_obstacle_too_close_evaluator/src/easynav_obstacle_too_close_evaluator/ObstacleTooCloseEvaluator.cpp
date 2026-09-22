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
/// \brief Implementation of the ObstacleTooCloseEvaluator class.

#include <cmath>

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_core/ObstacleProximity.hpp"

#include "easynav_obstacle_too_close_evaluator/ObstacleTooCloseEvaluator.hpp"

namespace easynav
{

void ObstacleTooCloseEvaluator::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".safe_distance", safe_distance_);
  node->declare_parameter<double>(
    plugin_name + ".linear_velocity_epsilon", linear_velocity_epsilon_);
  node->declare_parameter<double>(
    plugin_name + ".angular_velocity_epsilon", angular_velocity_epsilon_);
  node->declare_parameter<double>(plugin_name + ".debounce_duration", debounce_duration_);

  node->get_parameter<double>(plugin_name + ".safe_distance", safe_distance_);
  node->get_parameter<double>(plugin_name + ".linear_velocity_epsilon", linear_velocity_epsilon_);
  node->get_parameter<double>(
    plugin_name + ".angular_velocity_epsilon", angular_velocity_epsilon_);
  node->get_parameter<double>(plugin_name + ".debounce_duration", debounce_duration_);
}

void ObstacleTooCloseEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_plugin_name();
  // Shared, string-based convention with SafeRetreatRecovery's can_handle() — no compile-time
  // dependency between the two plugins, only this agreed-upon diagnostic vocabulary.
  status.hardware_id = "obstacle_proximity";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "no obstacle too close";

  if (!nav_state.has("robot_pose")) {
    stopped_since_.reset();
    publish_diagnostic(nav_state, status);
    return;
  }

  // "robot_pose" is written by LocalizerNode's RT cycle; this evaluator runs on SystemNode's
  // non-RT cycle, so get_safe() (a snapshot copy) is required here, not get(). See NavState's
  // own get()/get_safe() guidance.
  const auto odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");

  const double linear_speed = std::hypot(
    odom.twist.twist.linear.x, odom.twist.twist.linear.y);
  const double angular_speed = std::abs(odom.twist.twist.angular.z);
  const bool stopped = linear_speed < linear_velocity_epsilon_ &&
    angular_speed < angular_velocity_epsilon_;

  if (!stopped) {
    // Still moving (e.g. the level-0 reflex is still braking): too early to judge proximity as
    // something this evaluator should act on. See the compound-condition rationale in the
    // class doc comment.
    stopped_since_.reset();
    status.message = "still moving";
    publish_diagnostic(nav_state, status);
    return;
  }

  if (!stopped_since_.has_value()) {
    stopped_since_ = get_node()->now();
  }

  const double stopped_for = (get_node()->now() - *stopped_since_).seconds();
  if (stopped_for < debounce_duration_) {
    // "Stopped" must be sustained for a short debounce window before it is trusted — the RT
    // and non-RT cycles run in parallel, so a single low-velocity sample could still be taken
    // mid-brake.
    status.message = "recently stopped, confirming before evaluating proximity";
    publish_diagnostic(nav_state, status);
    return;
  }

  const auto obstacle = compute_nearest_obstacle(nav_state);
  if (std::isfinite(obstacle.distance) && obstacle.distance < safe_distance_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "stopped too close to an obstacle";

    diagnostic_msgs::msg::KeyValue distance_kv;
    distance_kv.key = "distance";
    distance_kv.value = std::to_string(obstacle.distance);
    status.values.push_back(distance_kv);

    diagnostic_msgs::msg::KeyValue bearing_kv;
    bearing_kv.key = "bearing";
    bearing_kv.value = std::to_string(obstacle.bearing);
    status.values.push_back(bearing_kv);
  }

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::ObstacleTooCloseEvaluator, easynav::RecoveryEvaluatorBase)
