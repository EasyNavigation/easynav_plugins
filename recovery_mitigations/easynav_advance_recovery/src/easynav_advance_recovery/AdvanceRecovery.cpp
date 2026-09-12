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
/// \brief Implementation of the AdvanceRecovery class.

#include <cmath>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_advance_recovery/AdvanceRecovery.hpp"

namespace easynav
{

void AdvanceRecovery::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".advance_distance", advance_distance_);
  node->declare_parameter<double>(plugin_name + ".advance_speed", advance_speed_);
  node->declare_parameter<double>(plugin_name + ".escalate_after", escalate_after_);
  node->declare_parameter<double>(plugin_name + ".episode_gap", episode_gap_);

  node->get_parameter<double>(plugin_name + ".advance_distance", advance_distance_);
  node->get_parameter<double>(plugin_name + ".advance_speed", advance_speed_);
  node->get_parameter<double>(plugin_name + ".escalate_after", escalate_after_);
  node->get_parameter<double>(plugin_name + ".episode_gap", episode_gap_);
}

bool AdvanceRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.hardware_id == "controller_stuck" &&
         status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void AdvanceRecovery::on_start(NavState & nav_state)
{
  const rclcpp::Time now = get_node()->now();

  if (!episode_start_.has_value() ||
    (last_stop_time_.has_value() && (now - *last_stop_time_).seconds() > episode_gap_))
  {
    // First activation ever, or a long enough gap since we last stopped that this is a new,
    // unrelated stuck episode rather than a continuation of the previous one.
    episode_start_ = now;
  }

  const auto odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
  start_position_ = {odom.pose.pose.position.x, odom.pose.pose.position.y};

  report(
    nav_state, rcl_interfaces::msg::Log::WARN,
    "AdvanceRecovery [" + get_plugin_name() + "]: robot commanded to move but stuck, advancing " +
    std::to_string(advance_distance_) + " m");
}

RecoveryStatus AdvanceRecovery::on_cycle(NavState & nav_state)
{
  if ((get_node()->now() - *episode_start_).seconds() >= escalate_after_) {
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "AdvanceRecovery [" + get_plugin_name() + "]: stuck episode has lasted over " +
      std::to_string(escalate_after_) + " s in total, giving up");
    episode_start_.reset();
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }

  const auto odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
  const double dx = odom.pose.pose.position.x - start_position_.first;
  const double dy = odom.pose.pose.position.y - start_position_.second;

  if (std::hypot(dx, dy) >= advance_distance_) {
    // This advance is done — deliberately NOT claiming the stuck condition itself is fixed:
    // episode_start_ is left untouched so a quick reactivation keeps counting toward
    // escalate_after_. See the class doc comment.
    stop_robot(nav_state);
    return RecoveryStatus::SUCCEEDED;
  }

  geometry_msgs::msg::TwistStamped cmd;
  if (auto node = get_node()) {
    cmd.header.stamp = node->now();
  }
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd.twist.linear.x = advance_speed_;

  nav_state.set("cmd_vel", cmd);
  return RecoveryStatus::RUNNING;
}

void AdvanceRecovery::on_stop(NavState &)
{
  last_stop_time_ = get_node()->now();
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::AdvanceRecovery, easynav::RecoveryMitigationBase)
