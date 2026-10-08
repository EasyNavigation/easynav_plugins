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
/// \brief Implementation of the DummySafetyReflex class.

#include "easynav_common/Parameters.hpp"
#include "easynav_diagnostic_recovery/DummySafetyReflex.hpp"

namespace easynav_diagnostic_recovery
{

void DummySafetyReflex::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<bool>(*node, plugin_name + ".trigger", trigger_);
  node->get_parameter<bool>(plugin_name + ".trigger", trigger_);
}

bool DummySafetyReflex::check([[maybe_unused]] easynav::NavState & nav_state)
{
  return trigger_;
}

void DummySafetyReflex::mitigate(easynav::NavState & nav_state)
{
  stop_robot(nav_state);
}

}  // namespace easynav_diagnostic_recovery

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  easynav_diagnostic_recovery::DummySafetyReflex,
  easynav_diagnostic_recovery::SafetyReflexBase)
