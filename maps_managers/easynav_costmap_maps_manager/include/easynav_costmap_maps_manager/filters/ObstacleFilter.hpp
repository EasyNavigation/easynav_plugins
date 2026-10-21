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


#ifndef EASYNAV_COSTMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_
#define EASYNAV_COSTMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_

#include <string>

#include "easynav_common/types/NavState.hpp"

#include "easynav_costmap_maps_manager/filters/CostmapFilter.hpp"

namespace easynav
{

class ObstacleFilter : public CostmapFilter
{
public:
  ObstacleFilter();

  virtual void on_initialize();
  virtual void update(NavState & nav_state);

protected:
  /// Points below this height (m, map frame) are ignored: floor hits ("<plugin>.min_height")
  double min_height_ {0.1};
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_
