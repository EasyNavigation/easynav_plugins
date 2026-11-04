// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

/// \file
/// \brief Square grid that gives, for any point, the nearest of a set of seed points in O(1).

#ifndef EASYNAV_MPPI_CONTROLLER__NEARESTGRID_HPP_
#define EASYNAV_MPPI_CONTROLLER__NEARESTGRID_HPP_

#include <cstdint>
#include <vector>

namespace easynav
{

/// \brief Square grid where each cell knows its nearest seed (8SSEDT-like propagation).
///
/// Built once per control cycle from the seeds (obstacle points, path samples), then queried
/// for every rollout state. Queries outside the grid use its nearest border cell.
class NearestGrid
{
public:
  /// \brief Reserves a grid of up to \p max_cells_per_side cells per side (no allocations later).
  void reserve(int max_cells_per_side);

  /// \brief Empties the grid, centered on (\p cx, \p cy), \p half_size to each side.
  /// The resolution is \p resolution, coarser if needed to fit the reserved cells.
  void reset(double cx, double cy, double half_size, double resolution);

  /// \brief Adds seed \p id at (\p x, \p y); seeds outside the grid are ignored.
  void add_seed(int id, double x, double y);

  /// \brief Propagates the seeds to every cell. Call after the last add_seed().
  void propagate();

  /// \brief Seed nearest to (\p x, \p y) (index for seed_id()/seed_x()/seed_y()), -1 if none.
  int nearest(double x, double y) const;

  /// \brief Distance from (\p x, \p y) to the nearest seed, \p none without seeds.
  double distance(double x, double y, double none) const;

  /// \brief Number of seeds in the grid.
  int seeds() const {return static_cast<int>(seed_x_.size());}

  int seed_id(int seed) const {return seed_ids_[seed];}
  double seed_x(int seed) const {return seed_x_[seed];}
  double seed_y(int seed) const {return seed_y_[seed];}

  double resolution() const {return resolution_;}

private:
  int side_ {0};            ///< Cells per side.
  int max_side_ {0};        ///< Reserved cells per side.
  double resolution_ {0.05};
  double origin_x_ {0.0};   ///< Corner of cell (0, 0).
  double origin_y_ {0.0};
  std::vector<int32_t> cell_seed_;  ///< Index into seed_x_/seed_y_, -1 = none.
  std::vector<int32_t> seed_ids_;   ///< Caller's id of each seed.
  std::vector<double> seed_x_;
  std::vector<double> seed_y_;

  /// \brief Squared distance from the center of cell (\p cx, \p cy) to seed \p s.
  double cell_dist2(int cx, int cy, int32_t s) const;
  /// \brief Takes the seed of neighbour (\p nx, \p ny) for cell (\p cx, \p cy) if nearer.
  void relax(int cx, int cy, int nx, int ny, double & best);
};

}  // namespace easynav

#endif  // EASYNAV_MPPI_CONTROLLER__NEARESTGRID_HPP_
