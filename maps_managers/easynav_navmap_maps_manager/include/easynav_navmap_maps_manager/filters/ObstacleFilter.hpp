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


#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_

#include <string>
#include <limits>

#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

/**
 * @class ObstacleFilter
 * @brief Marks as obstacles the NavCels under the points that rise above the NavMap surface.
 *
 * Points within max_range of the robot and below max_height (robot frame) are grouped in
 * 0.3 m columns; a column is an obstacle when its highest point is more than min_height (plus
 * min_height_per_meter for each meter from the robot) above the NavMap surface under it. So a 2D
 * laser sees obstacles too, and the ground or a ramp, being on the surface, is not one.
 */
class ObstacleFilter : public NavMapFilter
{
public:
  ObstacleFilter();

  void on_initialize() override;
  void update(::easynav::NavState & nav_state) override;

  bool is_adding_layer() override {return true;}
  std::string get_layer_name() override {return "obstacles";}

private:
  ::navmap::NavMap navmap_;
  double max_range_ {10.0};   ///< Points farther than this from the robot (x or y) are ignored.
  double min_height_ {0.1};   ///< Above the NavMap surface: lower points are the ground.
  double max_height_ {std::numeric_limits<double>::quiet_NaN()};  ///< Robot frame; NaN: no limit.
  double downsample_resolution_ {0.3};  ///< Voxel (m) the points are reduced to first; <= 0: off.
  double min_height_per_meter_ {0.0};   ///< min_height grows this much per meter from the robot.
};

}  // namespace navmap
}  // namespace easynav
#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_
