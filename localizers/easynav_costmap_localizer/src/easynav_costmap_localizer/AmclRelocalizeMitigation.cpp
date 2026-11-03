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
/// \brief Implementation of the AmclRelocalizeMitigation class.

#include <algorithm>
#include <cmath>
#include <string>

#include "easynav_common/Parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_publisher.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_costmap_localizer/AmclRelocalizeMitigation.hpp"

namespace easynav
{

namespace
{

// RViz2's "2D Pose Estimate": 0.5 m in x and y, pi/12 rad in yaw (standard deviations)
constexpr double kInitialVarianceXY = 0.25;
constexpr double kInitialVarianceYaw = 0.06853891945200942;

// The scores of the old pose may still be there: an attempt is checked after this (s)
constexpr double kSettleTime = 1.0;

}  // namespace

void AmclRelocalizeMitigation::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<int>(*node, plugin_name + ".attempts", attempts_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".attempt_timeout", attempt_timeout_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".max_offset", max_offset_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".covariance_threshold", covariance_threshold_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".match_threshold", match_threshold_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".quality_ratio_threshold", quality_ratio_threshold_);

  node->get_parameter<int>(plugin_name + ".attempts", attempts_);
  node->get_parameter<double>(plugin_name + ".attempt_timeout", attempt_timeout_);
  node->get_parameter<double>(plugin_name + ".max_offset", max_offset_);
  node->get_parameter<double>(plugin_name + ".covariance_threshold", covariance_threshold_);
  node->get_parameter<double>(plugin_name + ".match_threshold", match_threshold_);
  node->get_parameter<double>(
    plugin_name + ".quality_ratio_threshold", quality_ratio_threshold_);

  if (attempts_ < 1 || !(attempt_timeout_ > 0.0) || !(max_offset_ >= 0.0)) {
    throw std::runtime_error(
            "[" + plugin_name + "] invalid parameters: attempts >= 1, attempt_timeout > 0, "
            "max_offset >= 0");
  }

  // It no longer rotates: these are not used
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  for (const char * removed : {"rotation_speed", "timeout"}) {
    if (overrides.count(plugin_name + "." + removed) > 0) {
      RCLCPP_WARN(
        node->get_logger(), "[%s] '%s' has no effect any more: remove it (the pose is reset "
        "instead: attempts, attempt_timeout, max_offset)", plugin_name.c_str(), removed);
    }
  }

  // A plain publisher, as RViz2's: active whatever the node's lifecycle state
  initial_pose_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    *node, "initialpose", rclcpp::QoS(10));
}

bool AmclRelocalizeMitigation::can_handle(
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.hardware_id == "localizer.amcl" &&
         status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void AmclRelocalizeMitigation::on_start(NavState & nav_state)
{
  attempt_ = 0;
  // Selected on a diagnostic evaluated before AMCL converged: resetting it would lose it again
  converged_at_start_ = converged(nav_state);
  if (converged_at_start_) {
    return;
  }
  has_lost_pose_ = nav_state.has("robot_pose");
  if (!has_lost_pose_) {
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "AmclRelocalizeMitigation [" + get_plugin_name() + "]: no robot pose to reset to");
    return;
  }
  lost_pose_ = geometry_msgs::msg::PoseWithCovarianceStamped();
  lost_pose_.pose.pose = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  start_attempt(nav_state);
}

void AmclRelocalizeMitigation::start_attempt(NavState & nav_state)
{
  auto pose = lost_pose_;
  pose.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  pose.header.stamp = get_node()->now();
  if (attempt_ > 0) {
    // Uniform over the disc of radius max_offset
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double r = max_offset_ * std::sqrt(unit(rng_));
    const double a = 2.0 * M_PI * unit(rng_);
    pose.pose.pose.position.x += r * std::cos(a);
    pose.pose.pose.position.y += r * std::sin(a);
  }
  pose.pose.covariance[0] = kInitialVarianceXY;
  pose.pose.covariance[7] = kInitialVarianceXY;
  pose.pose.covariance[35] = kInitialVarianceYaw;
  initial_pose_pub_->publish(pose);
  attempt_start_ = get_node()->now();

  const auto & p = pose.pose.pose.position;
  report(
    nav_state, rcl_interfaces::msg::Log::WARN,
    "AmclRelocalizeMitigation [" + get_plugin_name() + "]: localization lost, resetting the " +
    "pose to (" + std::to_string(p.x) + ", " + std::to_string(p.y) + "), attempt " +
    std::to_string(attempt_ + 1) + " of " + std::to_string(attempts_));
}

bool AmclRelocalizeMitigation::converged(const NavState & nav_state) const
{
  // Written by AMCLLocalizer from both its RT and non-RT cycles; read here from
  // RecoveryManagerNode's RT cycle (this mitigation requires_control()), so get_safe() (a
  // snapshot copy) is required, not get(). See NavState's own get()/get_safe() guidance.
  if (!nav_state.has("localizer.amcl.covariance_trace")) {
    return false;
  }
  const double trace = nav_state.get_safe<double>("localizer.amcl.covariance_trace");
  // A compact cloud is not enough: it must also match the map.
  const bool matches = match_threshold_ <= 0.0 ||
    !nav_state.has("localizer.amcl.match_score") ||
    nav_state.get_safe<double>("localizer.amcl.match_score") >= match_threshold_;
  const bool steady = quality_ratio_threshold_ <= 0.0 ||
    !nav_state.has("localizer.amcl.quality_ratio") ||
    nav_state.get_safe<double>("localizer.amcl.quality_ratio") >= quality_ratio_threshold_;
  return trace <= covariance_threshold_ && matches && steady;
}

easynav_diagnostic_recovery::RecoveryStatus AmclRelocalizeMitigation::on_cycle(NavState & nav_state)
{
  stop_robot(nav_state);  // still while AMCL converges
  if (converged_at_start_) {
    return easynav_diagnostic_recovery::RecoveryStatus::SUCCEEDED;
  }
  if (!has_lost_pose_) {
    return easynav_diagnostic_recovery::RecoveryStatus::FAILED;
  }

  const double elapsed = (get_node()->now() - attempt_start_).seconds();
  if (elapsed >= std::min(kSettleTime, attempt_timeout_) && converged(nav_state)) {
    return easynav_diagnostic_recovery::RecoveryStatus::SUCCEEDED;
  }
  if (elapsed < attempt_timeout_) {
    return easynav_diagnostic_recovery::RecoveryStatus::RUNNING;
  }
  if (++attempt_ < attempts_) {
    start_attempt(nav_state);
    return easynav_diagnostic_recovery::RecoveryStatus::RUNNING;
  }
  report(
    nav_state, rcl_interfaces::msg::Log::ERROR,
    "AmclRelocalizeMitigation [" + get_plugin_name() + "]: gave up after " +
    std::to_string(attempts_) + " pose resets without relocalizing");
  return easynav_diagnostic_recovery::RecoveryStatus::FAILED;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  easynav::AmclRelocalizeMitigation,
  easynav_diagnostic_recovery::RecoveryMitigationBase)
