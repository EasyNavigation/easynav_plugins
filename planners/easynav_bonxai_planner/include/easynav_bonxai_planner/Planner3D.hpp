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
/// \brief Path planning in 3D space for aerial robots: a grid of the distance to the nearest
/// obstacle, and A* through the cells far enough from them.

#ifndef EASYNAV_BONXAI_PLANNER__PLANNER3D_HPP_
#define EASYNAV_BONXAI_PLANNER__PLANNER3D_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Core"

namespace easynav
{
namespace bonxai_planner
{

/// @brief What a map of surfaces (what the sensors saw) leaves solid below them, in each column.
enum class FillBelow
{
  None,     ///< Only the observed points are obstacles.
  Lowest,   ///< Below the lowest point of each column: the ground is solid.
  Highest,  ///< Below the highest one: buildings are solid blocks (an elevation map).
};

/// @brief "none", "lowest" or "highest" to \p fill; false if it is none of them.
bool parse_fill_below(const std::string & name, FillBelow & fill);

/// @brief A box of cubic cells with the distance (m) from each cell's centre to the nearest
/// obstacle point (Euclidean distance transform).
class ClearanceGrid
{
public:
  /// @brief Builds the grid over the obstacles' bounding box grown by \p margin, from z_min to
  /// z_max (NaN: the obstacles' own), with what \p fill makes solid below them. Columns with no
  /// points (gaps in the scans) take their neighbours' height.
  /// @return false if there are no obstacles or the grid would exceed \p max_cells.
  bool build(
    const std::vector<Eigen::Vector3d> & obstacles, double resolution, double margin,
    double z_min, double z_max, FillBelow fill = FillBelow::None,
    std::size_t max_cells = 60000000);

  bool empty() const {return dist_.empty();}
  double resolution() const {return res_;}
  const Eigen::Vector3d & origin() const {return origin_;}
  int nx() const {return nx_;}
  int ny() const {return ny_;}
  int nz() const {return nz_;}

  bool inside(int x, int y, int z) const
  {
    return x >= 0 && y >= 0 && z >= 0 && x < nx_ && y < ny_ && z < nz_;
  }
  std::size_t index(int x, int y, int z) const
  {
    return (static_cast<std::size_t>(z) * ny_ + y) * nx_ + x;
  }
  /// Distance (m) at a cell; 0 outside the grid.
  double distance(int x, int y, int z) const
  {
    return inside(x, y, z) ? dist_[index(x, y, z)] : 0.0;
  }
  /// Distance (m) at a point (its cell's); 0 outside the grid.
  double distance_at(const Eigen::Vector3d & p) const;
  bool cell_of(const Eigen::Vector3d & p, int & x, int & y, int & z) const;
  Eigen::Vector3d center(int x, int y, int z) const;

private:
  double res_ {1.0};
  Eigen::Vector3d origin_ {Eigen::Vector3d::Zero()};  ///< Corner of cell (0, 0, 0).
  int nx_ {0}, ny_ {0}, nz_ {0};
  std::vector<float> dist_;
};

struct PlannerParams
{
  double min_clearance {1.5};        ///< Cells nearer than this to an obstacle are not used (m).
  double preferred_clearance {4.0};  ///< Nearer than this costs more (m).
  double clearance_weight {2.0};     ///< Extra cost per m at no clearance (quadratic).
  std::size_t max_expansions {3000000};
  /// Start or goal nearer than min_clearance: the nearest cell that is not, within this (m).
  double snap_radius {5.0};
};

/// @brief Why plan() found no path, or Found.
enum class PlanStatus {Found, NoGrid, StartBlocked, GoalBlocked, Unreachable};

/// @brief A* over the grid's cells (26-connected), then straight segments wherever the
/// clearance along them allows. Empty if there is no path (\p status says why).
std::vector<Eigen::Vector3d> plan(
  const ClearanceGrid & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const PlannerParams & params, PlanStatus * status = nullptr);

struct SmootherParams
{
  int iterations {200};
  double smooth_weight {0.5};  ///< Pull of each point towards its neighbours' mean.
  /// Pull back towards where the planner put it: 0.01 rounds a 90 deg corner to ~2.5 m radius.
  double data_weight {0.01};
};

/// @brief \p path sampled every \p step, then relaxed: each point moves towards its neighbours'
/// mean (corners become curves), kept near its original place, and never closer to an obstacle
/// than \p clearance (a point that would be stays). The ends do not move.
std::vector<Eigen::Vector3d> smooth(
  const ClearanceGrid & grid, const std::vector<Eigen::Vector3d> & path, double step,
  double clearance, const SmootherParams & params);

/// @brief Whether every point of the segment keeps \p clearance (sampled every half cell).
bool segment_clear(
  const ClearanceGrid & grid, const Eigen::Vector3d & a, const Eigen::Vector3d & b,
  double clearance);

}  // namespace bonxai_planner
}  // namespace easynav

#endif  // EASYNAV_BONXAI_PLANNER__PLANNER3D_HPP_
