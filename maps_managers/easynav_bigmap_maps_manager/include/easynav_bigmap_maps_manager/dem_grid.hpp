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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__DEM_GRID_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__DEM_GRID_HPP_

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace bigmap_maps_manager
{

/// Square regular height grid in the local ENU frame, centred on the GPS origin: the DEM file the
/// plugin reads (terrain.dem_file). It is made by navmap_tools' navmap_gis_tool (--dem-out), which
/// writes the format of save() below (geo/dem_file.py there); test/data/dem_golden.bin keeps both
/// sides honest.
/// Vertex (i, j) sits at x = (i - c) * resolution, y = (j - c) * resolution, c = (side - 1) / 2,
/// so the origin is an exact node. z is relative to `reference_altitude`.
struct DemGrid
{
  /// where the heights come from (0 was Google's)
  enum Source {kSynthetic = 1, kIgn = 2, kMesh = 3, kTerrainTiles = 4, kCopernicus = 5};

  int side{0};                    ///< nodes per axis (always odd)
  double resolution{0.0};         ///< metres between nodes of this grid (the mesh resolution)
  double data_resolution{0.0};    ///< spacing at which the source was sampled (0: = resolution)
  double radius{0.0};             ///< requested half-side (m)
  double origin_lat{0.0};
  double origin_lon{0.0};
  /// Altitude as configured (NaN = automatic, i.e. DEM value at the origin).
  double requested_reference{std::numeric_limits<double>::quiet_NaN()};
  double reference_altitude{0.0};  ///< metres ASL mapped to z = 0
  int source{kIgn};
  int imagery_zoom{0};            ///< zoom of the imagery in `rgb` (0 = none)
  std::vector<uint8_t> rgb;       ///< optional imagery colour per node, 3 bytes, same order as z
  std::vector<float> z;           ///< row-major, index = j * side + i
  /// Optional: which cells have ground, (side - 1)^2 bytes, cell (i, j) between nodes i..i+1 and
  /// j..j+1, 1 = ground, 0 = a hole (a building, a wall: something the robot cannot be on). Empty:
  /// every cell has ground. A file without it (older) reads as empty.
  std::vector<uint8_t> cell_valid;

  float at(int i, int j) const {return z[static_cast<std::size_t>(j) * side + i];}
  double x(int i) const {return (i - (side - 1) / 2) * resolution;}
  double y(int j) const {return (j - (side - 1) / 2) * resolution;}

  /// Terrain height (m, relative to the reference altitude) at metric (x, y): bilinear, clamped to
  /// the grid.
  double height_at(double x, double y) const;

  /// Binary file. load() returns nullopt if missing/corrupt.
  bool save(const std::string & path) const;
  static std::optional<DemGrid> load(const std::string & path);

  /// A grid with every `stride`-th node of this one ((side - 1) must be divisible by stride).
  /// The imagery colours are not subsampled (empty rgb). A coarse cell has ground only if all its
  /// cells do (a hole is never closed).
  DemGrid subsample(int stride) const;
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__DEM_GRID_HPP_
