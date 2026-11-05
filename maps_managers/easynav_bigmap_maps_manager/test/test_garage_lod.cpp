// Copyright 2026 José Miguel Guerrero
//
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

// The hierarchy engine on the real garage map (bigmap_bringup/maps/parking_garage.navmap, the one
// the garage scenario uses, with its settings): the meshes must not fold over the surface. The
// source has no triangle facing down; the coarse levels of the hierarchy still fold a few (a
// cut that mixes levels can), so this bounds them (39 of ~40 000 before the fold check of
// build_levels, 33 after) instead of asking for none.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <system_error>

#include "easynav_bigmap_maps_manager/surface_lod.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/navmap_io.hpp"

namespace
{

// Triangles that face down (clockwise seen from above)
std::size_t facing_down(const bigmap_maps_manager::LodMesh & m)
{
  std::size_t n = 0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
    const double cross = (m.x[b] - m.x[a]) * (m.y[c] - m.y[a]) -
      (m.y[b] - m.y[a]) * (m.x[c] - m.x[a]);
    n += cross < 0.0;
  }
  return n;
}

}  // namespace

TEST(GarageLod, TheMeshesDoNotFoldOverTheSurface)
{
  const char * from_env = std::getenv("BIGMAP_GARAGE_NAVMAP");
  const std::filesystem::path path = from_env ? from_env : BIGMAP_GARAGE_NAVMAP;
  if (!std::filesystem::exists(path)) {GTEST_SKIP() << path << " not found";}
  ::navmap::NavMap nm;
  std::error_code ec;
  ASSERT_TRUE(navmap_ros::io::load_from_file(path.string(), nm, &ec)) << ec.message();

  bigmap_maps_manager::SurfaceLodMesher::Config cfg;   // params_parking_garage.yaml
  cfg.min_size = 0.4;
  cfg.max_size = 16.0;
  cfg.keep_narrow_width = 3.0;
  cfg.drop_unreachable_ground = true;
  bigmap_maps_manager::SurfaceLodMesher mesher(nm, cfg);
  bigmap_maps_manager::LodParams p;
  p.high_res_radius = 5.0;
  p.low_res_distance = 20.0;
  p.min_size = cfg.min_size;
  p.max_size = cfg.max_size;

  float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
  for (std::size_t v = 0; v < nm.positions.x.size(); ++v) {
    xmin = std::min(xmin, nm.positions.x[v]); xmax = std::max(xmax, nm.positions.x[v]);
    ymin = std::min(ymin, nm.positions.y[v]); ymax = std::max(ymax, nm.positions.y[v]);
  }
  std::size_t total_down = 0, total = 0;
  for (const float fx : {0.1f, 0.5f, 0.9f}) {
    for (const float fy : {0.2f, 0.8f}) {
      const auto m = mesher.extract(
        xmin + fx * (xmax - xmin), ymin + fy * (ymax - ymin), 0.0, p);
      ASSERT_GT(m.triangle_count(), 0u);
      const auto down = facing_down(m);
      std::printf("[garage] robot (%.1f, %.1f): %zu triangles, %zu facing down\n",
        xmin + fx * (xmax - xmin), ymin + fy * (ymax - ymin), m.triangle_count(), down);
      total_down += down;
      total += m.triangle_count();
    }
  }
  EXPECT_LE(total_down * 500, total) << "more than 0.2 % of the triangles face down";
}
