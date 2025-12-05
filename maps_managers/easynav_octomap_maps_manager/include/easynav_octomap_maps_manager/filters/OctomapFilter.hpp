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


#ifndef EASYNAV_PLANNER__FILTERS__OCTOMAPFILTER_HPP_
#define EASYNAV_PLANNER__FILTERS__OCTOMAPFILTER_HPP_

#include <expected>
#include <string>

#include "octomap/octomap.h"
#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/TFInfo.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{
namespace octomap
{

class OctomapFilter
{
public:
  OctomapFilter();

  std::expected<void, std::string>
  initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name,
    const TFInfo & tf_info);

  virtual std::expected<void, std::string> on_initialize() = 0;
  virtual void update(::easynav::NavState & nav_state) = 0;

  float get_map_resolution() {return map_resolution_;}
  void set_map_resolution(float resolution) {map_resolution_ = resolution;}

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

  const std::string & get_plugin_name() const;

  const TFInfo & get_tf_info() const;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};
  std::string plugin_name_;
  TFInfo tf_info_;

  float map_resolution_ {0.1};
};
}  // namespace octomap
}  // namespace easynav
#endif  // EASYNAV_PLANNER__FILTERS__OCTOMAPFILTER_HPP_
