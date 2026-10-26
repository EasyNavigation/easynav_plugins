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
/// \brief Implementation of the AmclConvergenceEvaluator class.

#include "easynav_common/Parameters.hpp"
#include "easynav_costmap_localizer/AmclConvergenceEvaluator.hpp"

namespace easynav
{

void AmclConvergenceEvaluator::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".covariance_threshold",
    covariance_threshold_);
  node->get_parameter<double>(plugin_name + ".covariance_threshold", covariance_threshold_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".match_threshold", match_threshold_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".quality_ratio_threshold", quality_ratio_threshold_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".debounce_duration", debounce_duration_);
  node->get_parameter<double>(plugin_name + ".match_threshold", match_threshold_);
  node->get_parameter<double>(plugin_name + ".quality_ratio_threshold", quality_ratio_threshold_);
  node->get_parameter<double>(plugin_name + ".debounce_duration", debounce_duration_);
}

void AmclConvergenceEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_plugin_name();
  status.hardware_id = "localizer.amcl";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "converged";

  if (!nav_state.has("localizer.amcl.covariance_trace")) {
    status.message = "no covariance data yet";
    publish_diagnostic(nav_state, status);
    return;
  }

  auto add_value = [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = std::to_string(value);
      status.values.push_back(kv);
    };

  // Written by AMCLLocalizer from both its RT and non-RT cycles; this evaluator runs on
  // RecoveryManagerNode's non-RT cycle, so get_safe() (a snapshot copy) is required here, not
  // get(). See NavState's own get()/get_safe() guidance.
  const double trace = nav_state.get_safe<double>("localizer.amcl.covariance_trace");
  add_value("covariance_trace", trace);

  // The scores, if the localizer reports them.
  std::string low_reason;
  if (nav_state.has("localizer.amcl.match_score")) {
    const double match = nav_state.get_safe<double>("localizer.amcl.match_score");
    add_value("match_score", match);
    if (match_threshold_ > 0.0 && match < match_threshold_) {
      low_reason = "scans do not match the map (lost)";
    }
  }
  if (nav_state.has("localizer.amcl.quality_ratio")) {
    const double ratio = nav_state.get_safe<double>("localizer.amcl.quality_ratio");
    add_value("quality_ratio", ratio);
    if (low_reason.empty() && ratio < quality_ratio_threshold_) {
      low_reason = "scan match dropped (kidnapped?)";
    }
  }

  const auto now = get_node()->now();
  if (low_reason.empty()) {
    low_since_.reset();
  } else if (!low_since_) {
    low_since_ = now;
  }

  if (trace > covariance_threshold_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "particle filter diverged";
  } else if (low_since_ && (now - *low_since_).seconds() >= debounce_duration_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = low_reason;
  }

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  easynav::AmclConvergenceEvaluator,
  easynav_diagnostic_recovery::RecoveryEvaluatorBase)
