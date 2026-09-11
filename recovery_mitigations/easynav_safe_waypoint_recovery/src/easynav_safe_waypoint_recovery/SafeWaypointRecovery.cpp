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
/// \brief Implementation of the SafeWaypointRecovery class.

#include <cmath>

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_safe_waypoint_recovery/SafeWaypointRecovery.hpp"

namespace easynav
{

void SafeWaypointRecovery::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<bool>(plugin_name + ".enabled", enabled_);
  node->declare_parameter<double>(plugin_name + ".safe_waypoint.x", x_);
  node->declare_parameter<double>(plugin_name + ".safe_waypoint.y", y_);
  node->declare_parameter<double>(plugin_name + ".safe_waypoint.yaw", yaw_);
  node->declare_parameter<std::string>(plugin_name + ".safe_waypoint.frame", frame_);

  node->get_parameter<bool>(plugin_name + ".enabled", enabled_);
  node->get_parameter<double>(plugin_name + ".safe_waypoint.x", x_);
  node->get_parameter<double>(plugin_name + ".safe_waypoint.y", y_);
  node->get_parameter<double>(plugin_name + ".safe_waypoint.yaw", yaw_);
  node->get_parameter<std::string>(plugin_name + ".safe_waypoint.frame", frame_);

  if (frame_.empty()) {
    frame_ = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  }

  // Same topic GoalManager already subscribes to for RViz/GUI single-pose goals
  // (GoalManager::comanded_pose_callback()) — reused here instead of GoalManagerClient, see
  // the class doc comment.
  goal_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
}

bool SafeWaypointRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return enabled_ && status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void SafeWaypointRecovery::on_start([[maybe_unused]] NavState & nav_state)
{
  RCLCPP_WARN(
    get_node()->get_logger(),
    "SafeWaypointRecovery [%s]: redirecting the mission to the safe waypoint "
    "(%.2f, %.2f, yaw=%.2f) in frame '%s'",
    get_plugin_name().c_str(), x_, y_, yaw_, frame_.c_str());

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = get_node()->now();
  goal.header.frame_id = frame_;
  goal.pose.position.x = x_;
  goal.pose.position.y = y_;
  // Planar (yaw-only) rotation: no need to pull in tf2 for this.
  goal.pose.orientation.z = std::sin(yaw_ / 2.0);
  goal.pose.orientation.w = std::cos(yaw_ / 2.0);

  goal_pose_pub_->publish(goal);
}

RecoveryStatus SafeWaypointRecovery::on_cycle([[maybe_unused]] NavState & nav_state)
{
  // The handoff already happened in on_start(): publishing the safe waypoint is a one-shot,
  // fire-and-forget action. Whether the robot actually reaches it is ordinary navigation from
  // here on (PlannerNode/ControllerNode against the new goal), not something this mitigation
  // supervises further.
  return RecoveryStatus::SUCCEEDED;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SafeWaypointRecovery, easynav::RecoveryMitigationBase)
