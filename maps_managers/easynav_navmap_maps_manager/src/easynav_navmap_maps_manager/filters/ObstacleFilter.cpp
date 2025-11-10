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
  auto t0 = parent_node_->now();

  std::cerr << "ObstacleFilter::update" << std::endl;
  if (!nav_state.has("map.navmap")) {
    return;
  }
  if (!nav_state.has("points")) {
    return;
  }

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");

  navmap_.layer_clear<float>(get_layer_name(), 0.0f);

  auto t1 = parent_node_->now();

  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(0.1)
    .fuse(get_tf_prefix() + "map")
    ->filter({-5.0, -5.0, NAN}, {5.0, 5.0, NAN})
    .as_points();

  auto t2 = parent_node_->now();

  size_t sidx = 0;
  std::optional<::navmap::NavCelId> last_cid;

    auto t3 = parent_node_->now();

  for (const auto & p : fused) {
    if (std::isnan(p.x) || std::isinf(p.x)) {continue;}
    
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;
    bool located = false;

    {
      ::navmap::NavMap::LocateOpts opts;
      if (last_cid) opts.hint_cid = *last_cid;
      located = navmap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit, opts);
    }

    if (!located) {
      located = navmap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit);
      if (!located) continue;
    }

    last_cid = cid;

    const float h = static_cast<float>(p.z) - hit.z();
    if ((h < 0.0f) || !std::isfinite(h)) continue;

    if (h > 0.1) {
      navmap_.layer_set<float>(get_layer_name(), cid, h);
    }
  }

  auto t4 = parent_node_->now();
  nav_state.set("map.navmap", navmap_);

  auto t5 = parent_node_->now();

  std::cerr << "t1 = " << std::fixed << std::setprecision(10) << (t1 - t0).seconds() << std::endl;
  std::cerr << "t2 = " << std::fixed << std::setprecision(10) << (t2 - t1).seconds() << std::endl;
  std::cerr << "t3 = " << std::fixed << std::setprecision(10) << (t3 - t2).seconds() << std::endl;
  std::cerr << "t4 = " << std::fixed << std::setprecision(10) << (t4 - t3).seconds() << std::endl;
  std::cerr << "t5 = " << std::fixed << std::setprecision(10) << (t5 - t4).seconds() << std::endl;
}

}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::ObstacleFilter, easynav::navmap::NavMapFilter)
