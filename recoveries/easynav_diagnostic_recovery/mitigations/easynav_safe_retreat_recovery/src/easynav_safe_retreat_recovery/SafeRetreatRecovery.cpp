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

#include "easynav_common/Parameters.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_diagnostic_recovery/ObstacleProximity.hpp"

#include "easynav_safe_retreat_recovery/SafeRetreatRecovery.hpp"

namespace easynav
{

void SafeRetreatRecovery::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".retreat_speed",
      retreat_speed_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".safe_distance",
      safe_distance_);

  node->get_parameter<double>(plugin_name + ".retreat_speed", retreat_speed_);
  node->get_parameter<double>(plugin_name + ".safe_distance", safe_distance_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".z_min_filter", z_min_filter_);
  node->get_parameter<double>(plugin_name + ".z_min_filter", z_min_filter_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".min_clearance", min_clearance_);
  node->get_parameter<double>(plugin_name + ".min_clearance", min_clearance_);
  robot_height_ = get_robot_geometry().height;
  robot_radius_ = get_robot_geometry().radius;
}

bool SafeRetreatRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.hardware_id == "obstacle_proximity" &&
         status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void SafeRetreatRecovery::on_start(NavState & nav_state)
{
  direction_ = 0;  // Chosen on the first cycle, from where the obstacle is then.
  report(
    nav_state, rcl_interfaces::msg::Log::WARN,
    "SafeRetreatRecovery [" + get_plugin_name() + "]: retreating from a too-close obstacle");
}

easynav_diagnostic_recovery::RecoveryStatus SafeRetreatRecovery::on_cycle(NavState & nav_state)
{
  const auto obstacle = easynav_diagnostic_recovery::compute_nearest_obstacle(
    nav_state, z_min_filter_, robot_height_);

  if (!obstacle.perceived) {
    // Blind: neither this nor the collision reflex can see the obstacle. Stay stopped and give
    // up, so the next mitigation (e.g. waiting for a human) takes over.
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "SafeRetreatRecovery [" + get_plugin_name() + "]: no perception, cannot retreat safely");
    stop_robot(nav_state);
    return easynav_diagnostic_recovery::RecoveryStatus::FAILED;
  }

  if (!std::isfinite(obstacle.distance) || obstacle.distance >= safe_distance_) {
    // Nothing near anymore, or already far enough: done.
    stop_robot(nav_state);
    return easynav_diagnostic_recovery::RecoveryStatus::SUCCEEDED;
  }

  auto clear = [&](int direction) {
      return easynav_diagnostic_recovery::free_distance_along_x(
        nav_state, direction, robot_radius_, z_min_filter_, robot_height_) >= min_clearance_;
    };

  if (direction_ == 0) {
    // Away from the obstacle: backward if it is ahead, forward if it is behind or beside.
    const int preferred = std::cos(obstacle.bearing) > 0.0 ? -1 : 1;
    const bool beside = std::abs(std::cos(obstacle.bearing)) < 0.5;  // 60-120 deg
    if (clear(preferred)) {
      direction_ = preferred;
    } else if (beside && clear(-preferred)) {
      direction_ = -preferred;
    } else {
      report(
        nav_state, rcl_interfaces::msg::Log::ERROR,
        "SafeRetreatRecovery [" + get_plugin_name() + "]: no clear way away from the obstacle "
        "(bearing=" + std::to_string(obstacle.bearing) + " rad)");
      stop_robot(nav_state);
      return easynav_diagnostic_recovery::RecoveryStatus::FAILED;
    }
    report(
      nav_state, rcl_interfaces::msg::Log::WARN,
      "SafeRetreatRecovery [" + get_plugin_name() + "]: moving " +
      (direction_ > 0 ? "forward" : "backward") + " away from the obstacle");
  } else if (!clear(direction_)) {
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "SafeRetreatRecovery [" + get_plugin_name() + "]: the way is blocked, stopping");
    stop_robot(nav_state);
    return easynav_diagnostic_recovery::RecoveryStatus::FAILED;
  }

  geometry_msgs::msg::TwistStamped cmd;
  if (auto node = get_node()) {
    cmd.header.stamp = node->now();
  }
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd.twist.linear.x = direction_ * retreat_speed_;

  command_velocity(nav_state, cmd);
  return easynav_diagnostic_recovery::RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SafeRetreatRecovery,
  easynav_diagnostic_recovery::RecoveryMitigationBase)
