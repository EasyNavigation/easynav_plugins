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

#include "easynav_bigmap_maps_manager/lod_mesher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <utility>

namespace bigmap_maps_manager
{

namespace
{
struct Extractor
{
  const RtinErrorMap & em;
  const DemGrid & dem;
  const LodParams & p;
  LodParams po;                // the policy for the observed ground
  double rx, ry;
  const KnownMap * known;
  const KnownMap * observed;
  const std::vector<std::uint8_t> * valid;       // per node; empty: all valid
  const std::vector<std::uint8_t> * cell_valid;  // per cell; empty: the node rule
  LodMesh & out;
  std::unordered_map<std::uint32_t, std::uint32_t> & index;  // grid node -> output vertex
  int center;                  // index of the DEM origin node
  int last;                    // last real node index

  uint32_t vertex(int x, int y)
  {
    const auto key = static_cast<std::uint32_t>(y) * static_cast<std::uint32_t>(em.grid_size()) +
      static_cast<std::uint32_t>(x);
    const auto it = index.emplace(key, static_cast<std::uint32_t>(out.x.size()));
    if (it.second) {
      out.x.push_back(static_cast<float>((x - center) * dem.resolution));
      out.y.push_back(static_cast<float>((y - center) * dem.resolution));
      out.z.push_back(em.height(x, y));
      if (!dem.rgb.empty()) {
        // padding nodes (beyond the real DEM) replicate the edge colour
        const std::size_t kk = (static_cast<std::size_t>(std::min(y, last)) * dem.side +
          std::min(x, last)) * 3;
        out.r.push_back(dem.rgb[kk]);
        out.g.push_back(dem.rgb[kk + 1]);
        out.b.push_back(dem.rgb[kk + 2]);
      }
    }
    return it.first->second;
  }

  // the smaller of the robot's effective distance and those of the traversed / observed ground
  double distance(int mx, int my, double hyp) const
  {
    const double dist = std::hypot((mx - center) * dem.resolution - rx,
        (my - center) * dem.resolution - ry);
    double d_eff = effective_distance(dist, hyp);
    // each set keeps full resolution within its margin of it, wherever the robot is; the finer
    // distance decides (the minimum keeps the crack-free order, see lod_policy.hpp)
    if (known && !known->empty()) {
      d_eff = std::min(d_eff, known_effective_distance(p, known->distance(mx, my), hyp));
    }
    if (observed && !observed->empty()) {
      d_eff = std::min(d_eff, known_effective_distance(po, observed->distance(mx, my), hyp));
    }
    return d_eff;
  }

  bool emitted(int ax, int ay, int bx, int by, int cx, int cy) const
  {
    if (!cell_valid->empty()) {
      // the cell under the centroid (exact for a NavCel of the grid's resolution: it is inside
      // one cell); a cell of the padding is no data
      const double gx = (ax + bx + cx) / 3.0, gy = (ay + by + cy) / 3.0;
      const int i = static_cast<int>(std::floor(gx)), j = static_cast<int>(std::floor(gy));
      if (i < 0 || j < 0 || i >= last || j >= last) {return false;}
      return (*cell_valid)[static_cast<std::size_t>(j) * last + i] != 0;
    }
    if (!valid->empty()) {
      // a NavCel centred on no-data ground does not exist
      const int gx = (ax + bx + cx + 1) / 3, gy = (ay + by + cy + 1) / 3;
      return gx <= last && gy <= last && (*valid)[static_cast<std::size_t>(gy) * dem.side + gx];
    }
    return true;
  }

  void process(int ax, int ay, int bx, int by, int cx, int cy)
  {
    // Whole subtree lies in the padding: nothing real to show (children stay inside the bbox).
    if (std::min({ax, bx, cx}) >= last || std::min({ay, by, cy}) >= last) {
      return;
    }
    const int mx = (ax + bx) >> 1;
    const int my = (ay + by) >> 1;
    const bool splittable = std::abs(ax - cx) + std::abs(ay - cy) > 1;
    if (splittable) {
      const double leg = std::hypot(ax - cx, ay - cy) * dem.resolution;
      const double hyp = leg * M_SQRT2;
      const double d_eff = distance(mx, my, hyp);
      // a narrow passage is split down to the grid's resolution whatever the distance
      if (em.pinned(mx, my) || should_split_at(p, em.error(mx, my), leg, d_eff)) {
        process(cx, cy, ax, ay, mx, my);
        process(bx, by, cx, cy, mx, my);
        return;
      }
    }
    if (!emitted(ax, ay, bx, by, cx, cy)) {return;}
    // Source winding is clockwise (y up); emit CCW.
    const uint32_t va = vertex(ax, ay), vb = vertex(bx, by), vc = vertex(cx, cy);
    out.tri.insert(out.tri.end(), {va, vc, vb});
    out.leg.push_back(static_cast<float>(std::hypot(ax - cx, ay - cy) * dem.resolution));
  }
};
}  // namespace

std::vector<std::uint8_t> hole_border_nodes(int side, const std::vector<std::uint8_t> & cell_valid)
{
  const int nc = side - 1;
  if (nc < 1 || cell_valid.size() != static_cast<std::size_t>(nc) * nc) {return {};}
  std::vector<std::uint8_t> border(static_cast<std::size_t>(side) * side, 0);
  for (int y = 0; y < side; ++y) {
    for (int x = 0; x < side; ++x) {
      bool some = false, none = false;
      for (int j = y - 1; j <= y; ++j) {
        for (int i = x - 1; i <= x; ++i) {
          if (i < 0 || j < 0 || i >= nc || j >= nc) {continue;}
          (cell_valid[static_cast<std::size_t>(j) * nc + i] ? some : none) = true;
        }
      }
      border[static_cast<std::size_t>(y) * side + x] = some && none;
    }
  }
  return border;
}

LodMesher::LodMesher(
  DemGrid dem, std::vector<std::uint8_t> valid,
  const std::vector<std::uint8_t> & pinned, std::vector<std::uint8_t> cell_valid,
  double hole_error)
: dem_(std::move(dem)),
  errors_(dem_, pinned,
    hole_error > 0.0 ? hole_border_nodes(dem_.side, cell_valid) : std::vector<std::uint8_t>{},
    hole_error),
  valid_(std::move(valid))
{
  const std::size_t nc = static_cast<std::size_t>(std::max(0, dem.side - 1));
  if (cell_valid.size() == nc * nc) {cell_valid_ = std::move(cell_valid);}
}

LodMesh LodMesher::extract(
  double robot_x, double robot_y, const LodParams & params, const KnownMap * known,
  const KnownMap * observed) const
{
  LodMesh mesh;
  std::unordered_map<std::uint32_t, std::uint32_t> index;
  index.reserve(4096);
  const int center = (dem_.side - 1) / 2;
  Extractor ex{errors_, dem_, params, observed_params(params), robot_x, robot_y, known, observed,
    &valid_, &cell_valid_,
    mesh, index, center, dem_.side - 1};
  const int ts = errors_.tile_size();
  ex.process(0, 0, ts, ts, ts, 0);
  ex.process(ts, ts, 0, 0, 0, ts);
  return mesh;
}

std::size_t LodMesher::memory_bytes() const
{
  return dem_.z.capacity() * sizeof(float) + dem_.rgb.capacity() + errors_.memory_bytes() +
         valid_.capacity() + cell_valid_.capacity();
}

}  // namespace bigmap_maps_manager
