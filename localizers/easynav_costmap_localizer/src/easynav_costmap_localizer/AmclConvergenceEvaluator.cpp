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

#include "easynav_costmap_localizer/AmclConvergenceEvaluator.hpp"

namespace easynav
{

void AmclConvergenceEvaluator::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".covariance_threshold", covariance_threshold_);
  node->get_parameter<double>(plugin_name + ".covariance_threshold", covariance_threshold_);
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

  // Written by AMCLLocalizer from both its RT and non-RT cycles; this evaluator runs on
  // RecoveryManagerNode's non-RT cycle, so get_safe() (a snapshot copy) is required here, not
  // get(). See NavState's own get()/get_safe() guidance.
  const double trace = nav_state.get_safe<double>("localizer.amcl.covariance_trace");

  if (trace > covariance_threshold_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "particle filter diverged";

    diagnostic_msgs::msg::KeyValue trace_kv;
    trace_kv.key = "covariance_trace";
    trace_kv.value = std::to_string(trace);
    status.values.push_back(trace_kv);
  }

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::AmclConvergenceEvaluator, easynav::RecoveryEvaluatorBase)
