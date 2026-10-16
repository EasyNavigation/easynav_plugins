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


#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__INFLATIONFILTER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__INFLATIONFILTER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "navmap_core/DistanceField.hpp"
#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

/**
 * @class InflationFilter
 * @brief "inflated_obstacles": cost by distance (along the surface) to the obstacles of the
 *        "obstacles" layer and, optionally, to the open edges of the NavMap.
 *
 * Incremental: an exact distance field (navmap::DistanceField) is updated with the obstacle cells
 * that appear or disappear, and only the cells whose distance or obstacle value changed are
 * rewritten (and recorded as changed). A new map or obstacle layer starts it over.
 */
class InflationFilter : public NavMapFilter
{
public:
  InflationFilter();
  ~InflationFilter() override;

  void on_initialize() override;
  void update(::easynav::NavState & nav_state) override;

  bool inflate_layer_u8(
    ::navmap::NavMap & nm,
    const std::string & src_layer,
    const std::string & dst_layer,
    float inflation_radius,
    float cost_scaling_factor,
    float inscribed_radius = 0.0f,
    bool inflate_borders = true);

  bool is_adding_layer() override {return true;}
  std::string get_layer_name() override {return "inflated_obstacles";}

protected:
  double inflation_radius_, cost_scaling_factor_, inscribed_radius_;
  bool inflate_borders_{true};  // open edges of the NavMap (walls, drops) inflate like obstacles

  // Incremental state, bound to the map (and layers) it was built on
  std::unique_ptr<::navmap::DistanceField> df_;
  const ::navmap::NavMap * df_map_{nullptr};
  std::uint64_t df_geometry_{0};
  std::weak_ptr<::navmap::LayerViewBase> last_src_, last_dst_;  // replaced or gone: rebuild
  std::vector<char> lethal_;  // obstacle cells the distance field is seeded with
};

}  // namespace navmap

}  // namespace easynav

#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__INFLATIONFILTER_HPP_
