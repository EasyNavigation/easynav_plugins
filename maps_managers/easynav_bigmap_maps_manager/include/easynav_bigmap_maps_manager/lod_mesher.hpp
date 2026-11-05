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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__LOD_MESHER_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__LOD_MESHER_HPP_

#include <cstdint>
#include <vector>

#include "easynav_bigmap_maps_manager/dem_grid.hpp"
#include "easynav_bigmap_maps_manager/known_map.hpp"
#include "easynav_bigmap_maps_manager/lod_policy.hpp"
#include "easynav_bigmap_maps_manager/rtin_error_map.hpp"

namespace bigmap_maps_manager
{

/// Adaptive triangle mesh in the local ENU frame. Triangles are CCW seen from +z.
struct LodMesh
{
  std::vector<float> x, y, z;       ///< shared vertices (all of them are DEM nodes)
  std::vector<uint8_t> r, g, b;     ///< per-vertex imagery colour (empty if the DEM has none)
  std::vector<uint32_t> tri;        ///< 3 vertex indices per triangle
  std::vector<float> leg;           ///< leg length (m) of each triangle, i.e. its LOD size
  /// Per triangle, 1 on ground the robot has driven over (only the surface mesher fills it)
  std::vector<uint8_t> traversed;
  /// Per triangle, the steepest slope (degrees) of the source triangles it stands for (only the
  /// surface mesher fills it): what can be driven is decided by the data, not by the simplified
  /// geometry, which clustering makes steeper than it is
  std::vector<float> slope;
  std::size_t triangle_count() const {return tri.size() / 3;}
};

/// Extracts a crack-free adaptive mesh centred on a robot position. extract() is const but NOT
/// thread-safe: one extraction at a time.
class LodMesher
{
public:
  /// `valid`: optional, one byte per DEM node (same order as dem.z): 0 = no data. Without
  /// `cell_valid`, NavCels whose centre is on a no-data node are not emitted.
  /// `pinned`: optional, one byte per DEM node: 1 = narrow passage, always at the grid's full
  /// resolution (see narrow_passage_nodes()).
  /// `cell_valid`: optional, one byte per DEM cell ((side - 1)^2, cell (i, j) between nodes i..i+1
  /// and j..j+1): 1 = the source map has ground at the cell's centre. With it, a NavCel is
  /// emitted only if the cell under its centroid has ground: at full resolution the holes of the
  /// source (its obstacles) are exactly where they are, whatever their size (the node rule lost
  /// every hole narrower than three cells).
  /// `hole_error` (m, with `cell_valid`): the border of every hole counts as a height error of that
  /// much (see RtinErrorMap), so holes keep their shape as far as the tolerance stays below it.
  /// The DEM is taken by value: move it in (a big grid is not copied).
  explicit LodMesher(
    DemGrid dem, std::vector<std::uint8_t> valid = {},
    const std::vector<std::uint8_t> & pinned = {}, std::vector<std::uint8_t> cell_valid = {},
    double hole_error = 0.0);

  LodMesher(const LodMesher &) = delete;
  LodMesher & operator=(const LodMesher &) = delete;

  /// With `known` (distance field up to date) the ground it marks as traversed stays at full
  /// resolution: the policy's distance becomes the smaller of the robot's and the traversed set's.
  /// `observed` (optional, same rule): obstacle borders the robot has seen at full resolution,
  /// kept so (a hole seen from near does not close again when the robot goes away, which would
  /// let a plan through it and make the robot come back: an oscillation).
  LodMesh extract(
    double robot_x, double robot_y, const LodParams & params,
    const KnownMap * known = nullptr, const KnownMap * observed = nullptr) const;

  const DemGrid & dem() const {return dem_;}
  const RtinErrorMap & error_map() const {return errors_;}
  /// Per DEM cell, 1 = ground (empty: no cell mask)
  const std::vector<std::uint8_t> & cell_valid() const {return cell_valid_;}
  /// Bytes held by the mesher (the DEM, the error map, the masks)
  std::size_t memory_bytes() const;

private:
  DemGrid dem_;
  RtinErrorMap errors_;
  std::vector<std::uint8_t> valid_;
  std::vector<std::uint8_t> cell_valid_;
};

/// Nodes on the border of a hole: some of their (up to four) cells have ground and some do not.
/// One byte per DEM node; empty if `cell_valid` is.
std::vector<std::uint8_t> hole_border_nodes(int side, const std::vector<std::uint8_t> & cell_valid);

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__LOD_MESHER_HPP_
