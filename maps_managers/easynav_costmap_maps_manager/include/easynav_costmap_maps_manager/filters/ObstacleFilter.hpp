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


#ifndef EASYNAV_PLANNER__FILTERS__OBSTACLEFILTER_HPP_
#define EASYNAV_PLANNER__FILTERS__OBSTACLEFILTER_HPP_

#include <expected>
#include <string>

#include "pluginlib/class_loader.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_costmap_maps_manager/filters/CostmapFilter.hpp"

namespace easynav
{

class ObstacleFilter : public CostmapFilter
{
public:
  ObstacleFilter();

  virtual std::expected<void, std::string> on_initialize();
  virtual void update(NavState & nav_state);
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__FILTERS__OBSTACLEFILTER_HPP_
