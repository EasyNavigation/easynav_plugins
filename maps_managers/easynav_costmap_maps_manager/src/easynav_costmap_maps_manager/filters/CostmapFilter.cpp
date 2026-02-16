// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#include <string>

#include "easynav_costmap_maps_manager/filters/CostmapFilter.hpp"

namespace easynav
{

CostmapFilter::CostmapFilter()
{

}

void
CostmapFilter::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  parent_node_ = parent_node;
  plugin_name_ = plugin_name;

  on_initialize();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
CostmapFilter::get_node() const
{
  return parent_node_;
}

const std::string &
CostmapFilter::get_plugin_name() const
{
  return plugin_name_;
}


}  // namespace easynav
