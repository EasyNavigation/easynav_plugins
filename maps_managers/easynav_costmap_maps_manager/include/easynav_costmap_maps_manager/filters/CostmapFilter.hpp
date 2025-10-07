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


#ifndef EASYNAV_PLANNER__FILTERS__COSTMAPFILTER_HPP_
#define EASYNAV_PLANNER__FILTERS__COSTMAPFILTER_HPP_

#include <expected>
#include <string>

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

class CostmapFilter
{
public:
  CostmapFilter();

  std::expected<void, std::string>
  initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name,
    const std::string & tf_prefix = "");

  virtual std::expected<void, std::string> on_initialize() = 0;
  virtual void update(NavState & nav_state) = 0;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

  const std::string & get_plugin_name() const;

  const std::string & get_tf_prefix() const;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};
  std::string plugin_name_;
  std::string tf_prefix_;
};
}  // namespace easynav

#endif  // EASYNAV_PLANNER__FILTERS__COSTMAPFILTER_HPP_
