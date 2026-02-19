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

#include <string>

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"

#include "easynav_costmap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
{

ObstacleFilter::ObstacleFilter()
{

}

void
ObstacleFilter::on_initialize()
{}

void
ObstacleFilter::update(NavState & nav_state)
{
  if (!nav_state.has("points")) {
    return;
  }

  const auto & perceptions = nav_state.get<PointPerceptions>("points");

  auto dynamic_map_ptr = nav_state.get_ptr<Costmap2D>("map.dynamic.filtered");
  Costmap2D & dynamic_map = *dynamic_map_ptr;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  rclcpp::Time stamp;

  auto view = PointPerceptionsOpsView(perceptions);
  view.downsample(dynamic_map.getResolution())
  .fuse(tf_info.robot_frame)
  .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN});

  const auto & fused = view.as_points();

  nav_state.set("map_time", stamp);

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & p : fused) {
    min_x = std::min(min_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_x = std::max(max_x, static_cast<double>(p.x));
    max_y = std::max(max_y, static_cast<double>(p.y));

    unsigned int cx, cy;
    if (dynamic_map.worldToMap(p.x, p.y, cx, cy)) {
      dynamic_map.setCost(cx, cy, LETHAL_OBSTACLE);
    }
  }

  if (!fused.empty()) {
    ObstacleBounds bb{min_x, min_y, max_x, max_y};
    nav_state.set("map.dynamic.obstacle_bounds", bb);
  }
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::ObstacleFilter, easynav::CostmapFilter)
