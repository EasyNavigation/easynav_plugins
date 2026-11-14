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

/// \brief Synthetic maps and scans for the tests.

#ifndef LOCALIZERS__EASYNAV_ORBIT_LOCALIZER__TESTS__TEST_MAPS_HPP_
#define LOCALIZERS__EASYNAV_ORBIT_LOCALIZER__TESTS__TEST_MAPS_HPP_

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_orbit_localizer/DistanceField.hpp"
#include "easynav_orbit_localizer/Lie.hpp"

namespace test_maps
{

using easynav::orbit::Pose;

/// A map of the given size (m) at 5 cm, all free, with its border as walls.
inline easynav::Costmap2D emptyRoom(double width, double height, double res = 0.05)
{
  const unsigned int w = static_cast<unsigned int>(width / res);
  const unsigned int h = static_cast<unsigned int>(height / res);
  easynav::Costmap2D map(w, h, res, 0.0, 0.0, easynav::FREE_SPACE);
  for (unsigned int i = 0; i < w; ++i) {
    map.setCost(i, 0, easynav::LETHAL_OBSTACLE);
    map.setCost(i, h - 1, easynav::LETHAL_OBSTACLE);
  }
  for (unsigned int j = 0; j < h; ++j) {
    map.setCost(0, j, easynav::LETHAL_OBSTACLE);
    map.setCost(w - 1, j, easynav::LETHAL_OBSTACLE);
  }
  return map;
}

/// Fills the cells of the rectangle [x0, x1] x [y0, y1] (m) with cost.
inline void fill(easynav::Costmap2D & map, double x0, double y0, double x1, double y1, uint8_t cost)
{
  int i0, j0, i1, j1;
  map.worldToMapEnforceBounds(x0, y0, i0, j0);
  map.worldToMapEnforceBounds(x1, y1, i1, j1);
  i1 = std::min(i1, static_cast<int>(map.getSizeInCellsX()) - 1);   // the far edge is the last cell
  j1 = std::min(j1, static_cast<int>(map.getSizeInCellsY()) - 1);
  for (int i = i0; i <= i1; ++i) {
    for (int j = j0; j <= j1; ++j) {
      map.setCost(i, j, cost);
    }
  }
}

/// A solid block as a SLAM map has it: a lethal outline around unknown cells.
inline void block(easynav::Costmap2D & map, double x0, double y0, double x1, double y1)
{
  fill(map, x0, y0, x1, y1, easynav::LETHAL_OBSTACLE);
  const double r = map.getResolution();
  if (x1 - x0 > 3.0 * r && y1 - y0 > 3.0 * r) {
    // The outline is a wall, except where the block reaches the map's edge: there the edge is
    // unknown too, as a SLAM map has it, never a wall behind unknown cells
    const double ex0 = map.getOriginX(), ey0 = map.getOriginY();
    const double ex1 = ex0 + map.getSizeInMetersX(), ey1 = ey0 + map.getSizeInMetersY();
    fill(
      map, x0 <= ex0 + r ? ex0 : x0 + 1.5 * r, y0 <= ey0 + r ? ey0 : y0 + 1.5 * r,
      x1 >= ex1 - r ? ex1 : x1 - 1.5 * r, y1 >= ey1 - r ? ey1 : y1 - 1.5 * r,
      easynav::NO_INFORMATION);
  }
}

/// Two identical 4 x 4 m rooms (x 0..4 and 5..9) under a corridor 1 m wide along the top
/// (y 4..5), each with its door at the same place of its north wall and the same table: a
/// place in one room looks exactly like the same place of the other, 5 m to the east.
inline easynav::Costmap2D twinRooms()
{
  auto map = emptyRoom(9.0, 5.0);
  block(map, 4.0, 0.0, 5.0, 4.0);                              // the wall between the rooms
  fill(map, 0.0, 4.0, 9.0, 4.05, easynav::LETHAL_OBSTACLE);    // the rooms' north wall
  fill(map, 1.5, 4.0, 2.5, 4.05, easynav::FREE_SPACE);         // the doors
  fill(map, 6.5, 4.0, 7.5, 4.05, easynav::FREE_SPACE);
  block(map, 0.5, 0.5, 1.5, 1.0);                              // the tables
  block(map, 5.5, 0.5, 6.5, 1.0);
  return map;
}

/// An L-shaped room with arms of different widths and a few objects: nothing in it looks like
/// anything else.
inline easynav::Costmap2D lRoom()
{
  auto map = emptyRoom(8.0, 8.0);
  block(map, 3.0, 4.5, 8.0, 8.0);
  block(map, 5.0, 2.5, 6.5, 4.0);                              // a pillar
  fill(map, 1.0, 1.0, 1.5, 1.5, easynav::LETHAL_OBSTACLE);
  fill(map, 6.0, 1.0, 7.5, 1.3, easynav::LETHAL_OBSTACLE);
  fill(map, 0.5, 6.0, 0.8, 7.5, easynav::LETHAL_OBSTACLE);
  return map;
}

/// A long corridor 1.2 m wide and 20 m long: nothing tells where along it the robot is.
inline easynav::Costmap2D corridor()
{
  return emptyRoom(20.0, 1.2);
}

/// A 360 degree scan from the pose, as points in the body frame (yaw 0 heading x).
inline std::vector<Eigen::Vector3d> scan(
  const easynav::orbit::GridDistanceField2D & field, const Pose & X, int rays = 360,
  double max_range = 12.0, double noise = 0.0, unsigned seed = 1)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, noise > 0.0 ? noise : 1e-12);
  std::vector<Eigen::Vector3d> out;
  const double yaw = easynav::orbit::yawOf(X);
  for (int k = 0; k < rays; ++k) {
    const double a = 2.0 * M_PI * k / rays - M_PI;
    double r = field.raycast(X.translation().x(), X.translation().y(), yaw + a, max_range);
    if (r >= max_range) {continue;}
    if (noise > 0.0) {r += n(rng);}
    out.emplace_back(r * std::cos(a), r * std::sin(a), 0.0);
  }
  return out;
}

inline double planarError(const Pose & a, const Pose & b)
{
  return (a.translation().head<2>() - b.translation().head<2>()).norm();
}

inline double yawError(const Pose & a, const Pose & b)
{
  return std::abs(easynav::orbit::wrapAngle(
      easynav::orbit::yawOf(a) - easynav::orbit::yawOf(b)));
}

}  // namespace test_maps

#endif  // LOCALIZERS__EASYNAV_ORBIT_LOCALIZER__TESTS__TEST_MAPS_HPP_
