// Copyright 2026 José Miguel Guerrero
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

// Out-of-line members of easynav::navmap::NavMapFilter, copied from easynav_navmap_maps_manager.
//
// Why they are here: BigMapMapsManager loads that package's filters (ObstaclesFilter,
// InflationFilter) through pluginlib. If this plugin were *linked* against
// libeasynav_navmap_maps_manager.so, the dynamic linker would load it first and pluginlib's
// ClassLoader would later find no factories for its classes ("no factory exists"). So we only
// use that package's headers and provide these four trivial members ourselves; the filter
// classes in the other library keep working on the same ABI.
#include <memory>
#include <string>

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

NavMapFilter::NavMapFilter() {}

void NavMapFilter::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  parent_node_ = parent_node;
  plugin_name_ = plugin_name;
  on_initialize();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> NavMapFilter::get_node() const
{
  return parent_node_.lock();
}

const std::string & NavMapFilter::get_plugin_name() const
{
  return plugin_name_;
}

}  // namespace navmap
}  // namespace easynav
