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
/// \brief Implementation of the SafetyChannelEvaluator class.

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "easynav_common/Parameters.hpp"
#include "easynav_core/SafetyChannel.hpp"

#include "easynav_safety_channel_evaluator/SafetyChannelEvaluator.hpp"

namespace easynav
{

void SafetyChannelEvaluator::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".max_stop_time", max_stop_time_);
  node->get_parameter<double>(plugin_name + ".max_stop_time", max_stop_time_);
  if (!std::isfinite(max_stop_time_) || max_stop_time_ < 0.0) {
    throw std::runtime_error(
            plugin_name + ".max_stop_time = " + std::to_string(max_stop_time_) + " (>= 0)");
  }
}

void SafetyChannelEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_plugin_name();
  status.hardware_id = "safety_channel";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  if (!nav_state.has(kSafetyStatusKey)) {
    status.message = "no safety status";
    stop_start_.reset();
    publish_diagnostic(nav_state, status);
    return;
  }

  // Written from the RT cycle: get_safe().
  const auto state = nav_state.get_safe<SafetyChannelState>(kSafetyStatusKey);
  if (!state.protective_stop) {
    std::ostringstream message;
    message << "no protective stop";
    if (std::isfinite(state.max_linear_vel)) {
      message << ", speed limited to " << state.max_linear_vel << " m/s";
    }
    status.message = message.str();
    stop_start_.reset();
    publish_diagnostic(nav_state, status);
    return;
  }

  const rclcpp::Time now = get_node()->now();
  if (!stop_start_) {
    stop_start_ = now;
  }
  const double stopped_for = (now - *stop_start_).seconds();

  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = state.status_lost ? "safety status lost: robot stopped" :
    "protective stop by the safety channel";
  if (max_stop_time_ > 0.0 && stopped_for >= max_stop_time_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message += " for too long";
  }

  diagnostic_msgs::msg::KeyValue duration;
  duration.key = "stop_duration";
  duration.value = std::to_string(stopped_for);
  status.values.push_back(duration);
  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SafetyChannelEvaluator,
  easynav_diagnostic_recovery::RecoveryEvaluatorBase)
