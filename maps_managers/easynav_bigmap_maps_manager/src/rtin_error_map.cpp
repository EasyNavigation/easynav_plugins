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

#include "easynav_bigmap_maps_manager/rtin_error_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace bigmap_maps_manager
{

std::uint16_t RtinErrorMap::quantize(float e)
{
  // rounded up: an error is never stored smaller than it is (a NaN height: the largest). Called
  // once per RTIN triangle (half a billion on a 16k grid): a multiplication and an integer
  // round-up, not a division and std::ceil (a libm call without SSE4.1)
  const float q = e * (1.0f / kErrorStep);
  if (!(q < 65535.0f)) {return 65535;}
  if (!(q > 0.0f)) {return 0;}
  const auto i = static_cast<std::uint32_t>(q);
  return static_cast<std::uint16_t>(static_cast<float>(i) < q ? i + 1 : i);
}

RtinErrorMap::RtinErrorMap(
  const DemGrid & dem, const std::vector<std::uint8_t> & pinned,
  const std::vector<std::uint8_t> & hole_nodes, double hole_error)
: dem_(&dem)
{
  if (dem.side < 3) {
    throw std::invalid_argument("DEM too small for RTIN");
  }
  real_side_ = dem.side;
  tile_size_ = 1;
  while (tile_size_ + 1 < real_side_) {
    tile_size_ <<= 1;
  }
  const int ts = tile_size_;
  const int gs = grid_size();
  const std::size_t n = static_cast<std::size_t>(gs) * gs;

  errors_.assign(n, 0);
  if (hole_error > 0.0 && hole_nodes.size() == dem.z.size()) {
    // the border of a hole counts as an error of hole_error: propagated up with the rest
    const std::uint16_t he = quantize(static_cast<float>(hole_error));
    for (int y = 0; y < real_side_; ++y) {
      for (int x = 0; x < real_side_; ++x) {
        if (hole_nodes[static_cast<std::size_t>(y) * real_side_ + x]) {errors_[idx(x, y)] = he;}
      }
    }
  }
  const bool any_pinned = std::find(pinned.begin(), pinned.end(), 1) != pinned.end();
  if (any_pinned) {
    pinned_.assign(n, 0);
    for (int y = 0; y < real_side_; ++y) {
      for (int x = 0; x < real_side_; ++x) {
        pinned_[idx(x, y)] = pinned[static_cast<std::size_t>(y) * real_side_ + x] ? 1 : 0;
      }
    }
  }

  const long num_triangles = static_cast<long>(ts) * ts * 2 - 2;       // NOLINT
  const long num_parents = num_triangles - static_cast<long>(ts) * ts;  // NOLINT

  // Walk the triangle tree from the smallest triangles up so children are done before parents.
  for (long i = num_triangles - 1; i >= 0; --i) {  // NOLINT
    long id = i + 2;                               // NOLINT
    int ax = 0, ay = 0, bx = 0, by = 0, cx = 0, cy = 0;
    if (id & 1) {
      bx = by = cx = ts;
    } else {
      ax = ay = cy = ts;
    }
    while ((id >>= 1) > 1) {
      const int mx = (ax + bx) >> 1;
      const int my = (ay + by) >> 1;
      if (id & 1) {
        bx = ax; by = ay; ax = cx; ay = cy;
      } else {
        ax = bx; ay = by; bx = cx; by = cy;
      }
      cx = mx; cy = my;
    }

    const int mx = (ax + bx) >> 1;
    const int my = (ay + by) >> 1;
    cx = mx + my - ay;
    cy = my + ax - mx;

    const float interpolated = (height(ax, ay) + height(bx, by)) * 0.5f;
    const std::size_t mid = idx(mx, my);
    errors_[mid] = std::max(errors_[mid], quantize(std::abs(interpolated - height(mx, my))));

    if (any_pinned) {
      if (i < num_parents) {
        pinned_[mid] |= pinned_[idx((ax + cx) >> 1, (ay + cy) >> 1)] |
          pinned_[idx((bx + cx) >> 1, (by + cy) >> 1)];
      } else {
        pinned_[mid] |= pinned_[idx(ax, ay)] | pinned_[idx(bx, by)] | pinned_[idx(cx, cy)];
      }
    }
    if (i < num_parents) {
      const std::size_t left = idx((ax + cx) >> 1, (ay + cy) >> 1);
      const std::size_t right = idx((bx + cx) >> 1, (by + cy) >> 1);
      errors_[mid] = std::max({errors_[mid], errors_[left], errors_[right]});
    }
  }
}

}  // namespace bigmap_maps_manager
