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


#ifndef EASYNAV_PLANNER__FILTERS__COSTMAPFILTER_HPP_
#define EASYNAV_PLANNER__FILTERS__COSTMAPFILTER_HPP_

#include <string>

#include "easynav_common/types/NavState.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

struct ObstacleBounds
{
  double min_x, min_y, max_x, max_y;
};


class CostmapFilter
{
public:
  CostmapFilter();

  void
  initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name);

  virtual void on_initialize() = 0;
  virtual void update(NavState & nav_state) = 0;

  const std::string & get_plugin_name() const;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};
  std::string plugin_name_;
};
}  // namespace easynav

#endif  // EASYNAV_PLANNER__FILTERS__COSTMAPFILTER_HPP_
