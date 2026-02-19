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

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "octomap_core/Octomap.hpp"

#include "easynav_octomap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
{
namespace octomap
{

ObstacleFilter::ObstacleFilter()
{

}

void
ObstacleFilter::on_initialize()
{}

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
  octomap_ = nav_state.get<::octomap::Octomap>("map");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  if (!octomap_.layer_copy<uint8_t>("occupancy", "obstacles")) {
    RCLCPP_ERROR(parent_node_->get_logger(), "Error copying layers at ObstacleFilter");
    return;
  }

  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(get_map_resolution())
    .fuse(tf_info.map_frame)
    .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .as_points();

  size_t sidx = 0;
  for (const auto & p : fused) {
    if (std::isnan(p.x)) {continue;}
    ::octomap::NavCelId cid;
    Eigen::Vector3f bary;
    Eigen::Vector3f hit;

    if (octomap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit)) {
      uint8_t v = octomap_.layer_get<uint8_t>("occupancy", cid, 255);
      if (v == 0) {
        octomap_.set_area<uint8_t>(hit, 254, "obstacles", ::octomap::AreaShape::CIRCULAR, 0.1);
      }
    }
  }

  nav_state.set("map", octomap_);
}

}  // namespace octomap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::octomap::ObstacleFilter, easynav::octomap::OctomapFilter)
