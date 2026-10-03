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
/// \brief Implementation of the SimpleRecoveryManager class.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_core/VelocityCommand.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_simple_recovery/SimpleRecoveryManager.hpp"

namespace easynav
{

void
SimpleRecoveryManager::on_initialize()
{
  auto node = get_node();
  const auto & p = get_plugin_name() + ".";

  auto param = [&](const std::string & name, auto & value) {
      declare_parameter_if_absent(*node, p + name, value);
      node->get_parameter(p + name, value);
    };

  param("stop_distance", stop_distance_);
  param("min_obstacle_z", min_obstacle_z_);

  // The area checked ahead is the robot's: "system_node.robot_geometry" (deprecated here).
  const auto geometry = get_robot_geometry({"robot_radius", "", "max_obstacle_z"});
  robot_radius_ = geometry.radius;
  max_obstacle_z_ = geometry.height;
  param("sensors_timeout", sensors_timeout_);
  param("max_position_variance", max_position_variance_);
  param("relocalize_timeout", relocalize_timeout_);
  param("rotate_speed", rotate_speed_);
  param("stuck_time", stuck_time_);
  param("stuck_distance", stuck_distance_);
  param("backup_speed", backup_speed_);
  param("backup_time", backup_time_);
  param("max_backup_attempts", max_backup_attempts_);
  param("slow_down_max_linear_vel", slow_down_max_linear_vel_);
}

void
SimpleRecoveryManager::on_activate()
{
  restart_sensors_watch_ = true;  // Sensors get sensors_timeout again
}

void
SimpleRecoveryManager::on_deactivate()
{
  stop_mitigation();
}

// ─── RT cycle: react fast, and execute the chosen mitigation ─────────────────────────────────

bool
SimpleRecoveryManager::update_rt(NavState & nav_state)
{
  // Fast recovery: about to hit something. Brake dead, overriding everything (even the
  // mitigation below: an override always wins over a takeover).
  if (moving_forward(nav_state) && obstacle_ahead(nav_state)) {
    override_velocity(nav_state, twist(0.0, 0.0));
    return true;
  }

  // Execute the mitigation decided by update(), taking over the controller.
  if (mitigation_ == Mitigation::ROTATE) {
    command_velocity(nav_state, twist(0.0, rotate_speed_));
    return true;
  }

  if (mitigation_ == Mitigation::BACK_UP) {
    command_velocity(nav_state, twist(-backup_speed_, 0.0));
    return true;
  }

  return false;  // Nothing to do: the controller drives
}

// ─── Non-RT cycle: diagnose and decide ───────────────────────────────────────────────────────

void
SimpleRecoveryManager::update(NavState & nav_state)
{
  // Case 1: no sensor data. Driving blind is not safe: terminate EasyNav.
  if (sensors_lost(nav_state)) {
    stop_mitigation();
    request_shutdown("no sensor data for " + std::to_string(sensors_timeout_) + " s");
    return;
  }

  // No mission, nothing to recover. Undo the slow down of the last one, if any.
  if (!has_mission(nav_state)) {
    stop_mitigation();
    backup_attempts_ = 0;
    if (slowed_down(nav_state)) {
      request_restore_parameters("mission ended: normal speed again");
    }
    return;
  }

  // Case 2: localization lost. The pose cannot be trusted, so no goal may be taken as reached
  // (hold), and rotating in place helps the localizer find itself again.
  if (localization_lost(nav_state)) {
    if (mitigation_ != Mitigation::ROTATE) {
      start(Mitigation::ROTATE);
      hold_mission_progress(true);
    } else if (elapsed_in_mitigation() > relocalize_timeout_) {
      stop_mitigation();
      abort_mission("could not relocalize");
    }
    return;
  }

  // Relocalized.
  if (mitigation_ == Mitigation::ROTATE) {
    stop_mitigation();
  }

  // Case 3: the robot does not move although the controller commands it. Back up for a while,
  // then let the controller try again. If that is not enough, try again slower; then give up.
  if (robot_stuck(nav_state)) {
    if (++backup_attempts_ <= max_backup_attempts_) {
      start(Mitigation::BACK_UP);
    } else if (slow_down_max_linear_vel_ > 0.0 && !slowed_down(nav_state)) {
      // EasyNav applies it between cycles, reconfiguring: this recovery system is reloaded too,
      // so its members start over (NavState tells the new one it slowed down).
      const bool accepted = request_reconfigure(
        {{"controller_node",
          rclcpp::Parameter("robot_limits.max_linear_vel", slow_down_max_linear_vel_)}},
        "stuck: slowing down");
      if (!accepted) {  // E.g. safety mode: the configuration is frozen.
        abort_mission("stuck, and unable to slow down");
      }
    } else {
      abort_mission("stuck after " + std::to_string(max_backup_attempts_) + " attempts");
    }
    return;
  }

  if (mitigation_ == Mitigation::BACK_UP && elapsed_in_mitigation() > backup_time_) {
    stop_mitigation();
  }

  // Case 4: all fine, the controller drives.
}

// ─── Cases ───────────────────────────────────────────────────────────────────────────────────

bool
SimpleRecoveryManager::sensors_lost(const NavState & nav_state)
{
  const auto now = steady_clock_.now();
  if (restart_sensors_watch_.exchange(false)) {
    newest_sensor_arrival_ = now;
  }

  // New data changes the newest stamp (not valid: sensors publish their perception before the
  // first data). Stamps are only compared with each other: simulated time stops or restarts.
  bool any_sensor = false;
  int64_t newest_stamp = 0;
  for (const auto & perception : nav_state.get_by_type<PointPerception>()) {
    any_sensor = true;
    if (perception->valid) {
      newest_stamp = std::max(newest_stamp, perception->stamp.nanoseconds());
    }
  }
  if (newest_stamp != 0 && newest_stamp != newest_sensor_stamp_) {
    newest_sensor_stamp_ = newest_stamp;
    newest_sensor_arrival_ = now;
  }
  if (!any_sensor) {
    return false;  // No point sensors configured
  }
  return (now - newest_sensor_arrival_).seconds() > sensors_timeout_;
}

bool
SimpleRecoveryManager::has_mission(const NavState & nav_state) const
{
  return nav_state.has("goals") &&
         !nav_state.get_safe<nav_msgs::msg::Goals>("goals").goals.empty();
}

bool
SimpleRecoveryManager::localization_lost(const NavState & nav_state) const
{
  if (!nav_state.has("robot_pose")) {
    return false;
  }
  const auto & covariance =
    nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.covariance;
  return covariance[0] > max_position_variance_ || covariance[7] > max_position_variance_;
}

bool
SimpleRecoveryManager::slowed_down(const NavState & nav_state) const
{
  // Parameters changed through request_reconfigure(), kept by EasyNav across the reload.
  if (!nav_state.has("reconfigured_parameters")) {
    return false;
  }
  const auto changed = nav_state.get_safe<std::vector<std::string>>("reconfigured_parameters");
  return std::find(
    changed.begin(), changed.end(), "controller_node/robot_limits.max_linear_vel") !=
         changed.end();
}

bool
SimpleRecoveryManager::robot_stuck(const NavState & nav_state)
{
  const auto now = get_node()->now();

  // Only the controller's own motion counts: restart while a mitigation drives, or while the
  // controller does not command any motion.
  constexpr double kMinCommandedSpeed = 0.01;  // [m/s]
  const bool commanded = nav_state.has("cmd_vel") &&
    std::abs(nav_state.get_safe<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x) >
    kMinCommandedSpeed;
  if (mitigation_ != Mitigation::NONE || !commanded || !nav_state.has("robot_pose")) {
    stuck_reference_.reset();
    return false;
  }

  const auto position =
    nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose.position;
  if (!stuck_reference_ ||
    std::hypot(position.x - stuck_reference_->x, position.y - stuck_reference_->y) >
    stuck_distance_)
  {
    stuck_reference_ = position;  // Moving: restart
    stuck_reference_time_ = now;
    return false;
  }

  if ((now - stuck_reference_time_).seconds() < stuck_time_) {
    return false;
  }
  stuck_reference_.reset();
  return true;
}

bool
SimpleRecoveryManager::moving_forward(const NavState & nav_state) const
{
  // Our mitigations never move forward; otherwise, the controller drives.
  if (mitigation_ != Mitigation::NONE) {
    return false;
  }
  const auto command = velocity_command::peek(nav_state, VelocitySource::CONTROLLER);
  return command && command->twist.linear.x > 0.0;
}

bool
SimpleRecoveryManager::obstacle_ahead(const NavState & nav_state) const
{
  const auto perceptions = nav_state.get_by_type<PointPerception>();
  if (perceptions.empty()) {
    return false;
  }

  // Points in front of the robot, within its width and height.
  PointPerceptionsOpsView view(perceptions);
  view.fuse(RTTFBuffer::getInstance()->get_tf_info().robot_frame)
  .filter(
    {0.0, -robot_radius_, min_obstacle_z_},
    {robot_radius_ + stop_distance_, robot_radius_, max_obstacle_z_});

  for (const auto & point : view.as_points().points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      return true;
    }
  }
  return false;
}

// ─── Mitigations ─────────────────────────────────────────────────────────────────────────────

void
SimpleRecoveryManager::start(Mitigation mitigation)
{
  RCLCPP_WARN(
    get_node()->get_logger(), "Recovery: %s",
    mitigation == Mitigation::ROTATE ? "localization lost, rotating" : "stuck, backing up");
  mitigation_start_ = get_node()->now();
  mitigation_ = mitigation;
}

void
SimpleRecoveryManager::stop_mitigation()
{
  if (mitigation_ == Mitigation::ROTATE) {
    hold_mission_progress(false);
  }
  mitigation_ = Mitigation::NONE;
}

double
SimpleRecoveryManager::elapsed_in_mitigation() const
{
  return (get_node()->now() - mitigation_start_).seconds();
}

geometry_msgs::msg::TwistStamped
SimpleRecoveryManager::twist(double linear, double angular) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = get_node()->now();
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd.twist.linear.x = linear;
  cmd.twist.angular.z = angular;
  return cmd;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SimpleRecoveryManager, easynav::RecoveryManagerBase)
