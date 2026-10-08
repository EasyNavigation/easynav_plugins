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

#include <queue>
#include <string>

#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

class InflationFilter : public NavMapFilter
{
public:
  InflationFilter();

  virtual void on_initialize() override;
  virtual void update(::easynav::NavState & nav_state) override;

  bool inflate_layer_u8(
    ::navmap::NavMap & nm,
    const std::string & src_layer,
    const std::string & dst_layer,
    float inflation_radius,
    float cost_scaling_factor,
    float inscribed_radius = 0.0f);

  bool is_adding_layer() override {return true;}
  std::string get_layer_name() override {return "inflated_obstacles";}

protected:
  ::navmap::NavMap navmap_;

  double inflation_radius_, cost_scaling_factor_, inscribed_radius_;
};

}  // namespace navmap

}  // namespace easynav

#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__IINFLATIONFILTER_HPP_
