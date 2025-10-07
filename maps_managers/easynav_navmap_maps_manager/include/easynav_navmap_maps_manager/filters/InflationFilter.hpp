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


#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__INFLATIONFILTER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__INFLATIONFILTER_HPP_

#include <expected>
#include <string>


#include "pluginlib/class_loader.hpp"

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

  virtual std::expected<void, std::string> on_initialize();
  virtual void update(::easynav::NavState & nav_state);

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
