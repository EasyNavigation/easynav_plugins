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

#include "easynav_bigmap_maps_manager/obstacle_memory.hpp"

#include <algorithm>
#include <cmath>

namespace bigmap_maps_manager
{

std::int64_t ObstacleMemory::key(double x, double y, double z) const
{
  const auto ix = static_cast<std::int64_t>(std::floor(x / cell_)) & 0xfffff;
  const auto iy = static_cast<std::int64_t>(std::floor(y / cell_)) & 0xfffff;
  const auto iz = static_cast<std::int64_t>(std::lround(z)) & 0xfffff;   // a band per metre
  return (ix << 40) | (iy << 20) | iz;
}

bool ObstacleMemory::add(const std::vector<SustainedHit> & detections, double now)
{
  bool changed = false;
  for (const auto & d : detections) {
    auto it = cells_.find(key(d.x, d.y, d.surface));
    if (it == cells_.end()) {
      cells_[key(d.x, d.y, d.surface)] = {d.x, d.y, d.surface, 1, now};
      changed = true;
      continue;
    }
    // A spot holds several detection cells, each with its own run, started at different moments:
    // they report their periods at different times. One hit per spot per period (with 10 %
    // allowance for the update rate), so max_hits hits need about max_hits x min_time seconds
    if (now - it->second.last < 0.9 * min_time_ || it->second.hits >= max_hits_) {continue;}
    ++it->second.hits;
    it->second.last = now;
    changed = true;
  }
  return changed;
}

void ObstacleMemory::restore(double x, double y, double z, int hits)
{
  hits = std::clamp(hits, 1, max_hits_);
  auto & cell = cells_[key(x, y, z)];
  if (cell.hits == 0) {
    cell = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0,
      -1e9};
  }
  cell.hits = std::max(cell.hits, hits);
}

std::size_t ObstacleMemory::fixed_count() const
{
  return static_cast<std::size_t>(std::count_if(
    cells_.begin(), cells_.end(), [this](const auto & kv) {return fixed(kv.second);}));
}

}  // namespace bigmap_maps_manager
