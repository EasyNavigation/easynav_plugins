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

#include <expected>
#include <string>

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"

#include "easynav_costmap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
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
ObstacleFilter::update(NavState & nav_state)
{
  if (!nav_state.has("points")) {
    return;
  }

  const auto & perceptions = nav_state.get<PointPerceptions>("points");

  Costmap2D dynamic_map = nav_state.get<Costmap2D>("map.dynamic.filtered");

  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(dynamic_map.getResolution())
    .fuse(get_tf_prefix() + "map")
    ->filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .as_points();

  for (const auto & p : fused) {
    unsigned int cx, cy;
    if (dynamic_map.worldToMap(p.x, p.y, cx, cy)) {
      dynamic_map.setCost(cx, cy, LETHAL_OBSTACLE);
    }
  }

  nav_state.set("map.dynamic.filtered", dynamic_map);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::ObstacleFilter, easynav::CostmapFilter)
