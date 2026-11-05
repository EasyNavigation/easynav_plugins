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

#include "easynav_bigmap_maps_manager/narrow_passages.hpp"

#include <algorithm>
#include <cmath>

namespace bigmap_maps_manager
{

std::vector<std::uint8_t> drivable_nodes(
  const DemGrid & dem, const std::vector<std::uint8_t> & valid, double max_slope_deg)
{
  const int n = dem.side;
  const auto at = [n](int x, int y) {return static_cast<std::size_t>(y) * n + x;};
  const double max_dz = std::tan(max_slope_deg * M_PI / 180.0) * dem.resolution;
  std::vector<std::uint8_t> ok(static_cast<std::size_t>(n) * n, 0);
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      if (!valid.empty() && !valid[at(x, y)]) {continue;}
      const float z = dem.z[at(x, y)];
      bool flat = true;
      for (const auto & [dx, dy] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
        const int u = x + dx, v = y + dy;
        if (u < 0 || v < 0 || u >= n || v >= n) {continue;}
        if (!valid.empty() && !valid[at(u, v)]) {continue;}
        if (z - dem.z[at(u, v)] > max_dz) {flat = false; break;}
      }
      ok[at(x, y)] = flat;
    }
  }
  return ok;
}

std::vector<std::uint8_t> keep_main_piece(
  const DemGrid & dem, const std::vector<std::uint8_t> & valid, double max_slope_deg,
  std::size_t * dropped)
{
  const int n = dem.side;
  const std::size_t total = static_cast<std::size_t>(n) * n;
  const auto ok = drivable_nodes(dem, valid, max_slope_deg);
  // pieces of drivable nodes (4-connected), by flood fill
  std::vector<int> piece(total, -1);
  std::vector<std::size_t> sizes, stack;
  for (std::size_t seed = 0; seed < total; ++seed) {
    if (!ok[seed] || piece[seed] >= 0) {continue;}
    const int id = static_cast<int>(sizes.size());
    sizes.push_back(0);
    stack.assign(1, seed);
    piece[seed] = id;
    while (!stack.empty()) {
      const std::size_t k = stack.back();
      stack.pop_back();
      ++sizes[id];
      const int x = static_cast<int>(k % n), y = static_cast<int>(k / n);
      for (const auto & [dx, dy] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
        const int u = x + dx, v = y + dy;
        if (u < 0 || v < 0 || u >= n || v >= n) {continue;}
        const std::size_t q = static_cast<std::size_t>(v) * n + u;
        if (ok[q] && piece[q] < 0) {
          piece[q] = id;
          stack.push_back(q);
        }
      }
    }
  }
  std::vector<std::uint8_t> out(total, 0);
  if (sizes.empty()) {
    if (dropped) {*dropped = 0;}
    return valid.empty() ? std::vector<std::uint8_t>(total, 1) : valid;   // nothing drivable
  }
  const int main = static_cast<int>(
    std::max_element(sizes.begin(), sizes.end()) - sizes.begin());
  std::size_t lost = 0;
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      const std::size_t k = static_cast<std::size_t>(y) * n + x;
      const bool had = valid.empty() || valid[k];
      bool keep = false;
      if (had) {
        if (ok[k]) {
          keep = piece[k] == main;
        } else {   // steep: kept if it touches the main piece (8-neighbourhood)
          for (int dy = -1; dy <= 1 && !keep; ++dy) {
            for (int dx = -1; dx <= 1 && !keep; ++dx) {
              const int u = x + dx, v = y + dy;
              if (u < 0 || v < 0 || u >= n || v >= n) {continue;}
              keep = piece[static_cast<std::size_t>(v) * n + u] == main;
            }
          }
        }
      }
      out[k] = keep;
      lost += had && !keep;
    }
  }
  if (dropped) {*dropped = lost;}
  return out;
}

std::vector<std::uint8_t> narrow_passage_nodes(
  const DemGrid & dem, const std::vector<std::uint8_t> & valid,
  double max_slope_deg, double width)
{
  if (width <= 0.0 || dem.side < 3) {return {};}
  const int n = dem.side;
  const auto at = [n](int x, int y) {return static_cast<std::size_t>(y) * n + x;};
  // drivable: data present and not on the face of a step or of a slope: the floor at the foot
  // of a wall stays drivable
  const auto ok = drivable_nodes(dem, valid, max_slope_deg);

  // Distance of every drivable node to the nearest undrivable one (the grid's own edge is not a
  // border: the map goes on); the passage half-width around a node is the largest such distance
  // in its surroundings, and the node is narrow when that is below half the width.
  const double half = 0.5 * width;
  const double reach = half + 0.5 * dem.resolution;
  const int rr = static_cast<int>(std::ceil(reach / dem.resolution));
  const auto far = static_cast<float>(reach + dem.resolution);
  std::vector<float> to_border(ok.size(), 0.0f);
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      if (!ok[at(x, y)]) {continue;}
      float best = far;
      for (int j = std::max(0, y - rr); j <= std::min(n - 1, y + rr); ++j) {
        for (int i = std::max(0, x - rr); i <= std::min(n - 1, x + rr); ++i) {
          if (ok[at(i, j)]) {continue;}
          const auto d = static_cast<float>(std::hypot(i - x, j - y) * dem.resolution);
          best = std::min(best, d);
        }
      }
      to_border[at(x, y)] = best;
    }
  }
  std::vector<std::uint8_t> narrow(ok.size(), 0);
  std::vector<std::uint8_t> core(ok.size(), 0);
  const int look = static_cast<int>(std::ceil((half + 0.25 * dem.resolution) / dem.resolution));
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      if (!ok[at(x, y)] || to_border[at(x, y)] >= far) {continue;}  // open ground
      float widest = 0.0f;
      for (int j = std::max(0, y - look); j <= std::min(n - 1, y + look); ++j) {
        for (int i = std::max(0, x - look); i <= std::min(n - 1, x + look); ++i) {
          if (std::hypot(i - x, j - y) * dem.resolution > look * dem.resolution) {continue;}
          widest = std::max(widest, to_border[at(i, j)]);
        }
      }
      core[at(x, y)] = widest < half;
    }
  }
  // the undrivable nodes next to a narrow passage (its walls) come with it, so its sides are as
  // fine as its floor; walls and slopes with no drivable ground nearby are left alone
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      if (!core[at(x, y)]) {continue;}
      for (int j = std::max(0, y - 1); j <= std::min(n - 1, y + 1); ++j) {
        for (int i = std::max(0, x - 1); i <= std::min(n - 1, x + 1); ++i) {
          narrow[at(i, j)] = 1;
        }
      }
    }
  }
  return narrow;
}

}  // namespace bigmap_maps_manager
