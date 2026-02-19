// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

#include <expected>
#include <string>

#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

NavMapFilter::NavMapFilter()
{

}

std::expected<void, std::string>
NavMapFilter::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name,
  const std::string & tf_prefix
)
{
  parent_node_ = parent_node;
  plugin_name_ = plugin_name;
  tf_prefix_ = tf_prefix;

  return on_initialize();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
NavMapFilter::get_node() const
{
  return parent_node_;
}

const std::string &
NavMapFilter::get_plugin_name() const
{
  return plugin_name_;
}

const std::string &
NavMapFilter::get_tf_prefix() const
{
  return tf_prefix_;
}

}  // namespace navmap
}  // namespace easynav
