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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__RTIN_ERROR_MAP_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__RTIN_ERROR_MAP_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

#include "easynav_bigmap_maps_manager/dem_grid.hpp"

namespace bigmap_maps_manager
{

/// RTIN (Right-Triangulated Irregular Network) error map, after Evans et al. / "martini".
///
/// The DEM is padded (edge replication, on the high-index side) to (2^k + 1)^2 nodes. Every node
/// that is the hypotenuse midpoint of some triangle of the implicit binary triangle tree gets the
/// worst height error incurred by *not* splitting there, propagated monotonically from the
/// descendants (error[parent] >= error[children]). Computed once, O(N log N).
///
/// Memory: the heights are not copied (they are read from the DEM, clamped to its edge for the
/// padding), and the errors are stored as centimetres in 16 bits, rounded up (so the order
/// error[parent] >= error[children] the crack-free argument needs is kept, and a split is never
/// missed: at most decided for an error up to 1 cm smaller). Errors saturate at 655 m.
/// The DEM must outlive the map (LodMesher owns both).
class RtinErrorMap
{
public:
  /// `pinned`: optional, one byte per DEM node (same order as dem.z): 1 = this node must be kept
  /// at full resolution (a narrow passage). Like the error it is propagated up the triangle tree,
  /// so a triangle that holds one is flagged and "split(child) => split(parent)" still holds.
  /// `hole_nodes` + `hole_error`: optional, one byte per DEM node: 1 = on the border of a hole of
  /// the map (an obstacle): its error is at least `hole_error` m, so the triangles around a hole
  /// are split, like those over a step of that height, wherever the tolerance is below it, and the
  /// hole is not covered by a coarse triangle there.
  explicit RtinErrorMap(
    const DemGrid & dem, const std::vector<std::uint8_t> & pinned = {},
    const std::vector<std::uint8_t> & hole_nodes = {}, double hole_error = 0.0);

  RtinErrorMap(const RtinErrorMap &) = delete;
  RtinErrorMap & operator=(const RtinErrorMap &) = delete;

  int tile_size() const {return tile_size_;}        ///< 2^k
  int grid_size() const {return tile_size_ + 1;}    ///< nodes per axis after padding
  int real_side() const {return real_side_;}        ///< nodes per axis of the original DEM

  float height(int x, int y) const
  {
    return dem_->at(std::min(x, real_side_ - 1), std::min(y, real_side_ - 1));
  }
  float error(int x, int y) const {return static_cast<float>(errors_[idx(x, y)]) * kErrorStep;}
  /// True if the triangle split at this node holds a pinned node
  bool pinned(int x, int y) const {return !pinned_.empty() && pinned_[idx(x, y)] != 0;}

  /// Resolution of the stored errors (m)
  static constexpr float kErrorStep = 0.01f;
  /// Bytes held (the errors, the pinned flags)
  std::size_t memory_bytes() const
  {
    return errors_.capacity() * sizeof(std::uint16_t) + pinned_.capacity();
  }

private:
  std::size_t idx(int x, int y) const
  {
    return static_cast<std::size_t>(y) * grid_size() + x;
  }
  static std::uint16_t quantize(float e);

  const DemGrid * dem_;
  int tile_size_{0};
  int real_side_{0};
  std::vector<std::uint16_t> errors_;
  std::vector<std::uint8_t> pinned_;  ///< empty: nothing pinned
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__RTIN_ERROR_MAP_HPP_
