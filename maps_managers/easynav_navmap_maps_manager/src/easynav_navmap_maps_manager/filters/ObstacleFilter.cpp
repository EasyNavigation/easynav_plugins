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
#include <cstdint>

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

void ObstacleFilter::update(::easynav::NavState & nav_state)
{
  auto t0 = parent_node_->now();
  std::cerr << "ObstacleFilter::update" << std::endl;

  if (!nav_state.has("map.navmap")) {return;}
  if (!nav_state.has("points")) {return;}

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");

  navmap_.layer_clear<uint8_t>(get_layer_name(), 0);

  auto t1 = parent_node_->now();

  const auto & points = PointPerceptionsOpsView(perceptions)
    .filter({-10.0, -10.0, NAN}, {10.0, 10.0, NAN})
    .downsample(0.3)
    .fuse("map")
    ->as_points();

  auto t2 = parent_node_->now();

  const float voxel_xy = 0.30f;
  const float voxel_z = 0.20f;

  struct Accum
  {
    std::unordered_set<int> z_bins;
    float max_z = -std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
  };

  struct Key
  {
    int ix, iy;
    bool operator==(const Key & o) const noexcept {return ix == o.ix && iy == o.iy;}
  };
  struct KeyHash
  {
    std::size_t operator()(const Key & k) const noexcept
    {
      std::size_t h1 = std::hash<long long>{}(static_cast<long long>(k.ix));
      std::size_t h2 = std::hash<long long>{}(static_cast<long long>(k.iy));
      return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
  };

  std::unordered_map<Key, Accum, KeyHash> bins;
  bins.reserve(points.size() / 4 + 1);

  for (const auto & p : points.points) {
    const float x = p.x;
    const float y = p.y;
    const float z = p.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}

    const int ix = static_cast<int>(std::floor(x / voxel_xy));
    const int iy = static_cast<int>(std::floor(y / voxel_xy));
    const int iz = static_cast<int>(std::floor(z / voxel_z));

    auto & acc = bins[{ix, iy}];
    acc.z_bins.insert(iz);
    if (z > acc.max_z) {acc.max_z = z;}
    if (z < acc.min_z) {acc.min_z = z;}
  }

  auto t3 = parent_node_->now();

  std::optional<size_t> last_surface;
  std::optional<::navmap::NavCelId> last_cid;

  const float height_threshold = 0.25f;

  for (const auto & kv : bins) {
    const auto & key = kv.first;
    const auto & acc = kv.second;

    const float cx = (static_cast<float>(key.ix) + 0.5f) * voxel_xy;
    const float cy = (static_cast<float>(key.iy) + 0.5f) * voxel_xy;
    const float dz = acc.max_z - acc.min_z;

    // std::cerr << "[ObstacleFilter] voxel (" << cx << ", " << cy << ") "
    //         << "vertical_bins=" << acc.z_bins.size()
    //         << " z_range=[" << acc.min_z << ", " << acc.max_z
    //         << "] Δz=" << dz << std::endl;

    if (acc.z_bins.size() <= 2 && dz <= height_threshold) {
      continue;
    }

    const float cz = acc.max_z;
    Eigen::Vector3f query(cx, cy, cz);

    size_t surface_idx = 0;
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;

    ::navmap::NavMap::LocateOpts opts;
    opts.use_downward_ray = true;
    opts.height_eps = 0.50f;
    if (last_surface) {opts.hint_surface = *last_surface;}
    if (last_cid) {opts.hint_cid = *last_cid;}

    bool ok = navmap_.locate_navcel(query, surface_idx, cid, bary, &hit, opts);

    if (!ok) {
      ::navmap::NavMap::LocateOpts nohint;
      nohint.use_downward_ray = true;
      nohint.height_eps = 0.50f;
      ok = navmap_.locate_navcel(query, surface_idx, cid, bary, &hit, nohint);
      if (!ok) {
        ok = navmap_.locate_navcel(query, surface_idx, cid, bary, &hit);
      }
    }

    if (ok) {
      navmap_.layer_set<uint8_t>(get_layer_name(), cid, static_cast<uint8_t>(255));
      last_surface = surface_idx;
      last_cid = cid;
    }
  }


  auto t4 = parent_node_->now();
  nav_state.set("map.navmap", navmap_);
  auto t5 = parent_node_->now();

  // std::cerr << "t1 = " << std::fixed << std::setprecision(10) << (t1 - t0).seconds() << std::endl;
  // std::cerr << "t2 = " << std::fixed << std::setprecision(10) << (t2 - t1).seconds() << std::endl;
  // std::cerr << "t3 = " << std::fixed << std::setprecision(10) << (t3 - t2).seconds() << std::endl;
  // std::cerr << "t4 = " << std::fixed << std::setprecision(10) << (t4 - t3).seconds() << std::endl;
  // std::cerr << "t5 = " << std::fixed << std::setprecision(10) << (t5 - t4).seconds() << std::endl;
}


}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::ObstacleFilter, easynav::navmap::NavMapFilter)
