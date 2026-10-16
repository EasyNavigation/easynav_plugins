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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_diagnostic_recovery/mitigations/RotateRecovery.hpp"

namespace easynav
{

namespace
{
std::string fmt(double v)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.0f", v);
  return b;
}
}  // namespace

void RotateRecovery::on_initialize()
{
  auto node = get_node();
  const auto & name = get_plugin_name();
  easynav::declare_parameter_if_absent<double>(*node, name + ".angle", angle_);
  easynav::declare_parameter_if_absent<double>(*node, name + ".angular_speed", angular_speed_);
  easynav::declare_parameter_if_absent<double>(*node, name + ".settle_time", settle_time_);
  easynav::declare_parameter_if_absent<double>(*node, name + ".timeout", timeout_);
  easynav::declare_parameter_if_absent<double>(*node, name + ".episode_gap", episode_gap_);
  easynav::declare_parameter_if_absent(*node, name + ".hardware_ids", hardware_ids_);
  node->get_parameter<double>(name + ".angle", angle_);
  node->get_parameter<double>(name + ".angular_speed", angular_speed_);
  node->get_parameter<double>(name + ".settle_time", settle_time_);
  node->get_parameter<double>(name + ".timeout", timeout_);
  node->get_parameter<double>(name + ".episode_gap", episode_gap_);
  node->get_parameter(name + ".hardware_ids", hardware_ids_);

  if (!(std::abs(angular_speed_) > 0.0) || !std::isfinite(angular_speed_)) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] angular_speed must be non-zero: using 0.5 rad/s", name.c_str());
    angular_speed_ = 0.5;
  }
  angular_speed_ = std::abs(angular_speed_);
  if (!std::isfinite(angle_) || angle_ == 0.0) {
    RCLCPP_WARN(node->get_logger(), "[%s] angle must be non-zero: using a full turn", name.c_str());
    angle_ = 2.0 * M_PI;
  }
  if (!(settle_time_ >= 0.0)) {settle_time_ = 0.0;}
  if (timeout_ <= 0.0 || !std::isfinite(timeout_)) {
    timeout_ = 2.0 * std::abs(angle_) / angular_speed_ + 5.0;
  }
}

bool RotateRecovery::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
         std::find(hardware_ids_.begin(), hardware_ids_.end(), status.hardware_id) !=
         hardware_ids_.end();
}

double RotateRecovery::yaw(NavState & nav_state) const
{
  const auto q = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void RotateRecovery::on_start(NavState & nav_state)
{
  const rclcpp::Time now = get_node()->now();
  if (last_stop_time_.has_value() && (now - *last_stop_time_).seconds() > episode_gap_) {
    done_this_episode_ = false;  // long enough since the last one: a new episode
  }
  // Once per episode: back so soon means the turn did not help
  give_up_ = done_this_episode_;
  start_time_ = now;
  settle_start_.reset();
  turned_ = 0.0;
  last_yaw_ = yaw(nav_state);
  if (give_up_) {
    report(
      nav_state, rcl_interfaces::msg::Log::WARN,
      "RotateRecovery [" + get_plugin_name() + "]: already turned in place, still no path: "
      "escalating");
    return;
  }
  report(
    nav_state, rcl_interfaces::msg::Log::WARN,
    "RotateRecovery [" + get_plugin_name() + "]: no path, turning in place " +
    fmt(std::abs(angle_) * 180.0 / M_PI) + " deg to look around");
}

easynav_diagnostic_recovery::RecoveryStatus RotateRecovery::on_cycle(NavState & nav_state)
{
  using easynav_diagnostic_recovery::RecoveryStatus;
  if (give_up_) {
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }
  const rclcpp::Time now = get_node()->now();

  // Turned enough: stand still for the planner, then done
  if (settle_start_.has_value()) {
    stop_robot(nav_state);
    return (now - *settle_start_).seconds() >= settle_time_ ? RecoveryStatus::SUCCEEDED :
           RecoveryStatus::RUNNING;
  }

  const double y = yaw(nav_state);
  turned_ += std::abs(std::remainder(y - last_yaw_, 2.0 * M_PI));
  last_yaw_ = y;
  if (turned_ >= std::abs(angle_)) {
    done_this_episode_ = true;
    settle_start_ = now;
    stop_robot(nav_state);
    return settle_time_ <= 0.0 ? RecoveryStatus::SUCCEEDED : RecoveryStatus::RUNNING;
  }
  if ((now - start_time_).seconds() >= timeout_) {
    done_this_episode_ = true;
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "RotateRecovery [" + get_plugin_name() + "]: could not turn in " + fmt(timeout_) +
      " s (turned " + fmt(turned_ * 180.0 / M_PI) + " deg): escalating");
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd.twist.angular.z = angle_ > 0.0 ? angular_speed_ : -angular_speed_;
  command_velocity(nav_state, cmd);
  return RecoveryStatus::RUNNING;
}

void RotateRecovery::on_stop(NavState &)
{
  last_stop_time_ = get_node()->now();
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  easynav::RotateRecovery,
  easynav_diagnostic_recovery::RecoveryMitigationBase)
