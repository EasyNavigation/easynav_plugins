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

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_costmap_localizer/AmclRelocalizeMitigation.hpp"

namespace easynav
{

void AmclRelocalizeMitigation::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".rotation_speed", rotation_speed_);
  node->declare_parameter<double>(plugin_name + ".timeout", timeout_);
  node->declare_parameter<double>(plugin_name + ".covariance_threshold", covariance_threshold_);

  node->get_parameter<double>(plugin_name + ".rotation_speed", rotation_speed_);
  node->get_parameter<double>(plugin_name + ".timeout", timeout_);
  node->get_parameter<double>(plugin_name + ".covariance_threshold", covariance_threshold_);
}

bool AmclRelocalizeMitigation::can_handle(
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.hardware_id == "localizer.amcl" &&
         status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

void AmclRelocalizeMitigation::on_start(NavState & nav_state)
{
  start_time_ = get_node()->now();
  report(
    nav_state, rcl_interfaces::msg::Log::WARN,
    "AmclRelocalizeMitigation [" + get_plugin_name() +
    "]: localization diverged, rotating in place to relocalize");
}

RecoveryStatus AmclRelocalizeMitigation::on_cycle(NavState & nav_state)
{
  // Written by AMCLLocalizer from both its RT and non-RT cycles; read here from
  // RecoveryManagerNode's RT cycle (this mitigation requires_control()), so get_safe() (a
  // snapshot copy) is required, not get(). See NavState's own get()/get_safe() guidance.
  if (nav_state.has("localizer.amcl.covariance_trace")) {
    const double trace = nav_state.get_safe<double>("localizer.amcl.covariance_trace");
    if (trace <= covariance_threshold_) {
      stop_robot(nav_state);
      return RecoveryStatus::SUCCEEDED;
    }
  }

  if ((get_node()->now() - start_time_).seconds() >= timeout_) {
    report(
      nav_state, rcl_interfaces::msg::Log::ERROR,
      "AmclRelocalizeMitigation [" + get_plugin_name() + "]: gave up after " +
      std::to_string(timeout_) + " s without relocalizing");
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }

  geometry_msgs::msg::TwistStamped cmd;
  if (auto node = get_node()) {
    cmd.header.stamp = node->now();
  }
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd.twist.angular.z = rotation_speed_;

  nav_state.set("cmd_vel", cmd);
  return RecoveryStatus::RUNNING;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::AmclRelocalizeMitigation, easynav::RecoveryMitigationBase)
