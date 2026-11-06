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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__OBSTACLE_MEMORY_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__OBSTACLE_MEMORY_HPP_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "easynav_bigmap_maps_manager/dynamic_layer.hpp"

namespace bigmap_maps_manager
{

/// How many times an obstacle has been detected on each spot of the map, whatever the LOD mesh is
/// at the moment (NavCel ids change with every re-mesh, so the count lives on a grid of its own,
/// one cell per `cell` metres and per metre of height, and is read back onto the NavCels).
///
/// A hit is a detection that went on for `min_time` seconds over the spot (see
/// DynamicLayer::sustained); `max_hits` hits make the obstacle FIXED: it stays for good, whether
/// it is still seen or not. Below that, the count is only the memory of how often it was seen.
class ObstacleMemory
{
public:
  struct Cell
  {
    float x, y, z;       ///< where the first detection was
    int hits;            ///< 1 .. max_hits
    double last;         ///< when the last hit was added (s)
  };

  ObstacleMemory(double cell, double min_time, int max_hits)
  : cell_(cell), min_time_(min_time), max_hits_(max_hits) {}

  /// Adds a hit to the cell of every detection (one per cell, however many detections it holds).
  /// Returns true if any count changed.
  bool add(const std::vector<SustainedHit> & detections, double now);

  /// A cell known from a saved map: `hits` (clamped to 1..max_hits), at least what it had.
  /// Its last hit counts as long ago, so a detection can add to it at once.
  void restore(double x, double y, double z, int hits);

  double cell_size() const {return cell_;}
  int max_hits() const {return max_hits_;}
  bool fixed(const Cell & c) const {return c.hits >= max_hits_;}
  const std::unordered_map<std::int64_t, Cell> & cells() const {return cells_;}
  std::size_t fixed_count() const;
  /// Forgets everything (what a recovery would do to let the obstacles go)
  void clear() {cells_.clear();}

private:
  std::int64_t key(double x, double y, double z) const;
  double cell_, min_time_;
  int max_hits_;
  std::unordered_map<std::int64_t, Cell> cells_;
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__OBSTACLE_MEMORY_HPP_
