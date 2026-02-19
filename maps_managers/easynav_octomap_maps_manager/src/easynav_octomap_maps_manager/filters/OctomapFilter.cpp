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

#include <string>

#include "octomap/octomap.h"

#include "easynav_octomap_maps_manager/filters/OctomapFilter.hpp"

namespace easynav
{
namespace octomap
{

OctomapFilter::OctomapFilter()
{

}

void
OctomapFilter::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  parent_node_ = parent_node;
  plugin_name_ = plugin_name;
  on_initialize();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
OctomapFilter::get_node() const
{
  return parent_node_;
}

const std::string &
OctomapFilter::get_plugin_name() const
{
  return plugin_name_;
}

}  // namespace octomap
}  // namespace easynav
