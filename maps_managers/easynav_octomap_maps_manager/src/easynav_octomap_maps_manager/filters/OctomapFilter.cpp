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


#include <expected>
#include <string>

#include "octomap/octomap.h"
#include "easynav_common/types/NavState.hpp"

#include "easynav_octomap_maps_manager/filters/OctomapFilter.hpp"

namespace easynav
{
namespace octomap
{

OctomapFilter::OctomapFilter()
{

}

std::expected<void, std::string>
OctomapFilter::initialize(
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
OctomapFilter::get_node() const
{
  return parent_node_;
}

const std::string &
OctomapFilter::get_plugin_name() const
{
  return plugin_name_;
}

const std::string &
OctomapFilter::get_tf_prefix() const
{
  return tf_prefix_;
}

}  // namespace octomap
}  // namespace easynav
