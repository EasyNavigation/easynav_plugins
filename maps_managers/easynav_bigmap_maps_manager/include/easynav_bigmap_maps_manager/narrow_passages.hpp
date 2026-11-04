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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__NARROW_PASSAGES_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__NARROW_PASSAGES_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "easynav_bigmap_maps_manager/dem_grid.hpp"

namespace bigmap_maps_manager
{

/// Nodes of a height grid the robot can drive on: data present and no 4-neighbour lower by more
/// than the slope allows, i.e. not on the face of a step or a slope (the floor at the foot of a
/// wall is drivable). `valid`: as below. One byte per node, 1 = drivable.
std::vector<std::uint8_t> drivable_nodes(
  const DemGrid & dem, const std::vector<std::uint8_t> & valid, double max_slope_deg);

/// Keeps the ground the robot can get to: the largest piece of drivable nodes (4-connected) and
/// the not drivable nodes that touch it (the faces of walls and slopes, which are the obstacles);
/// every other drivable piece, such as the roofs of buildings or a plateau with no way up, and
/// the steep ground that only touches those, becomes "no data". A goal picked on a roof (the
/// topmost surface under the mouse) would otherwise be a place no plan can reach.
/// Returns the new `valid` (one byte per node, same order as dem.z); `dropped` (if given) is the
/// number of nodes that had data and no longer do.
std::vector<std::uint8_t> keep_main_piece(
  const DemGrid & dem, const std::vector<std::uint8_t> & valid, double max_slope_deg,
  std::size_t * dropped = nullptr);

/// Nodes of a height grid that lie in a passage narrower than `width` metres: ground the robot can
/// drive on (slope <= max_slope_deg, data present) whose free width, measured between the nodes
/// that are not drivable (too steep, or no data), is below `width`. The mesher keeps them at the
/// grid's full resolution however far the robot is: coarse NavCels in a narrow passage leave no
/// room for the robot's footprint and the planner cannot find the way through from far away. The
/// grid-engine counterpart of SurfaceLodMesher's keep_narrow_width.
///
/// `valid`: as LodMesher's (one byte per node, 0 = no data; empty = all data).
/// Returns one byte per node (same order as dem.z), 1 = narrow; empty if width <= 0.
std::vector<std::uint8_t> narrow_passage_nodes(
  const DemGrid & dem, const std::vector<std::uint8_t> & valid,
  double max_slope_deg, double width);

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__NARROW_PASSAGES_HPP_
