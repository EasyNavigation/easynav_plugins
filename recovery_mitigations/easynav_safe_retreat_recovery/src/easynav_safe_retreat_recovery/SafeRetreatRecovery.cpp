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
/// \brief Implementation of the SafeRetreatRecovery class.

#include <cmath>

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_core/ObstacleProximity.hpp"

#include "easynav_safe_retreat_recovery/SafeRetreatRecovery.hpp"

namespace easynav
{

void SafeRetreatRecovery::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".retreat_speed", retreat_speed_);
  node->declare_parameter<double>(plugin_name + ".safe_distance", safe_distance_);

  node->get_parameter<double>(plugin_name + ".retreat_speed", retreat_speed_);
  node->get_parameter<double>(plugin_name + ".safe_distance", safe_distance_);
}

bool SafeRetreatRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.hardware_id == "obstacle_proximity" &&
         status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void SafeRetreatRecovery::on_start(NavState &)
{
  RCLCPP_WARN(
    get_node()->get_logger(), "SafeRetreatRecovery [%s]: retreating from a too-close obstacle",
    get_plugin_name().c_str());
}

RecoveryStatus SafeRetreatRecovery::on_cycle(NavState & nav_state)
{
  const auto obstacle = compute_nearest_obstacle(nav_state);

  if (!std::isfinite(obstacle.distance) || obstacle.distance >= safe_distance_) {
    // Nothing to retreat from (perception lost) or already far enough: done.
    stop_robot(nav_state);
    return RecoveryStatus::SUCCEEDED;
  }

  if (std::abs(obstacle.bearing) > M_PI / 2.0) {
    // The nearest obstacle is behind the robot: reversing would drive toward it, not away.
    // Fail safely instead of guessing a direction. See the class doc comment.
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "SafeRetreatRecovery [%s]: nearest obstacle is behind the robot (bearing=%.2f rad), "
      "cannot safely retreat straight back",
      get_plugin_name().c_str(), obstacle.bearing);
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }

  geometry_msgs::msg::TwistStamped cmd;
  if (auto node = get_node()) {
    cmd.header.stamp = node->now();
  }
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd.twist.linear.x = -retreat_speed_;

  nav_state.set("cmd_vel", cmd);
  return RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SafeRetreatRecovery, easynav::RecoveryMitigationBase)
