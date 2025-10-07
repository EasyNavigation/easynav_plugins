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

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "navmap_core/NavMap.hpp"

#include "easynav_navmap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
{
namespace navmap
{

ObstacleFilter::ObstacleFilter()
{

}

std::expected<void, std::string>
ObstacleFilter::on_initialize()
{
  return {};
}

void
ObstacleFilter::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has("map")) {
    return;
  }
  if (!nav_state.has("points")) {
    return;
  }

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  navmap_ = nav_state.get<::navmap::NavMap>("map");

  if (!navmap_.layer_copy<uint8_t>("occupancy", "obstacles")) {
    RCLCPP_ERROR(parent_node_->get_logger(), "Error copying layers at ObstacleFilter");
    return;
  }

  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(get_map_resolution())
    .fuse(get_tf_prefix() + "map")
    ->filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .as_points();

  size_t sidx = 0;
  for (const auto & p : fused) {
    if (std::isnan(p.x)) {continue;}
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary;
    Eigen::Vector3f hit;

    if (navmap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit)) {
      uint8_t v = navmap_.layer_get<uint8_t>("occupancy", cid, 255);
      if (v == 0) {
        navmap_.set_area<uint8_t>(hit, 254, "obstacles", ::navmap::AreaShape::CIRCULAR, 0.1);
      }
    }
  }

  nav_state.set("map", navmap_);
}

}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::ObstacleFilter, easynav::navmap::NavMapFilter)
