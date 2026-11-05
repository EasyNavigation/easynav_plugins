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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__NAVMAP_DEM_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__NAVMAP_DEM_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "easynav_bigmap_maps_manager/dem_grid.hpp"
#include "navmap_core/NavMap.hpp"

namespace bigmap_maps_manager
{

/// What the meshers read of a NavMap, as plain arrays: built from a NavMap, or straight from a
/// NavMap file's message (the plugin does that), so that a big map is never built as a NavMap
/// (normals, adjacency, BVH: several times the size of the arrays) only to be read once.
struct SourceMesh
{
  std::vector<float> x, y, z;                ///< vertices
  std::vector<std::uint32_t> v0, v1, v2;     ///< triangles (vertex indices)
  std::vector<std::uint8_t> r, g, b;         ///< colour per vertex (empty: none)
  std::vector<std::uint8_t> traversed;       ///< per triangle (empty: the map has no such layer)
  std::vector<std::uint8_t> obstacle_hits;   ///< per triangle (empty: no such layer)
  bool has_traversed_layer{false};

  std::size_t triangle_count() const {return v0.size();}
  bool coloured() const
  {
    return !r.empty() && r.size() == x.size() && g.size() == x.size() && b.size() == x.size();
  }
  /// Throws std::invalid_argument naming what is wrong: arrays of different sizes, a vertex index
  /// out of range, a non-finite position (a corrupt file).
  void validate() const;
  /// The arrays of a NavMap (its `traversed` and `obstacle_hits` layers, if uint8)
  static SourceMesh from_navmap(const ::navmap::NavMap & map);
};

/// A NavMap of any resolution turned into the regular height grid the LOD mesher works from.
struct NavMapDem
{
  DemGrid dem;  ///< centred on the map's (0, 0), no geolocation
  /// 1 where the NavMap has surface; the grid is square, the map is not: the rest is no data
  std::vector<std::uint8_t> valid;
  /// Per grid cell ((side - 1)^2, cell (i, j) between nodes i..i+1 and j..j+1): 1 = a NavCel of
  /// the map covers the cell's centre. The holes of the map exactly (no closing: a hole of one
  /// cell is an obstacle of one cell), which the LodMesher uses to emit its NavCels.
  std::vector<std::uint8_t> cell_valid;
  /// DEM nodes (i, j) covered by NavCels of the `traversed` layer, if the map has one
  std::vector<std::pair<int, int>> traversed_nodes;
  bool has_traversed{false};
  std::size_t covered_nodes{0};
};

/// Rasterises the NavMap's NavCels onto a grid of `resolution` metres centred on (0, 0): height by
/// barycentric interpolation (the highest where several NavCels overlap), imagery colour from the
/// vertex colours. Node gaps narrower than two nodes (sampling of very small NavCels) are closed
/// for the heights; the cells (cell_valid) keep every hole. Throws std::invalid_argument for an
/// empty map or a grid beyond `max_side`.
NavMapDem dem_from_navmap(const SourceMesh & map, double resolution, int max_side = 8193);
NavMapDem dem_from_navmap(
  const ::navmap::NavMap & map, double resolution, int max_side = 8193);

/// Georeference of a map, saved next to it: the position of the map's (0, 0, 0).
struct MapOrigin
{
  bool georeferenced{false};
  double lat{0.0}, lon{0.0}, altitude{0.0};
};

/// `<map>.navmap` -> `<map>.yaml`
std::string origin_file_for(const std::string & navmap_path);
bool save_origin(const std::string & navmap_path, const MapOrigin & origin, double resolution);
/// Missing or unreadable file: origin 0, 0, 0 and not georeferenced.
MapOrigin load_origin(const std::string & navmap_path);

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__NAVMAP_DEM_HPP_
