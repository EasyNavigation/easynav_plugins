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

/**
 * @file tiled_lod.hpp
 * @brief A NavMap at several levels of detail, by tiles: fine near a point, coarse far from it.
 *
 * build() cuts the map along a grid (navmap::cut_along_lines) into square tiles and simplifies
 * them (navmap::simplify) into a quadtree: a tile of level k is its four children of level k - 1,
 * put together and simplified with twice their error. The borders of a tile keep the vertices
 * where they meet the borders of the map and its corners (SimplifyParams::keep_vertices), so
 * tiles simplified apart still meet. Done once.
 *
 * select() picks, around a point, the coarsest tile each place allows (level k from radius(k)
 * on), and assemble() puts them in one NavMap, stitching their borders: where a tile has a
 * vertex on a border its neighbour does not, the neighbour's NavCel there is split to reach it.
 * The result is conforming (no cracks), so a planner crosses from tile to tile. Milliseconds.
 */

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__TILED_LOD_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__TILED_LOD_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "navmap_core/NavMap.hpp"

namespace easynav
{
namespace bigmap
{

struct LodParams
{
  double cell_min{0.5};     ///< m: the finest level keeps the map within cell_min / 2
  double cell_max{8.0};     ///< m: no NavCel side longer than this; the coarsest level's error
  double min_radius{15.0};  ///< m: the finest level within it
  double max_radius{60.0};  ///< m: the coarsest level from it
  double tile{0.0};         ///< m: side of the finest tiles; 0: min_radius / 2
};

class TiledLod
{
public:
  struct Stats
  {
    std::vector<std::size_t> navcels;  ///< per level, in all its tiles
    std::size_t tiles{0};              ///< finest tiles with ground
    double seconds{0.0};
  };

  /// Cuts @p map into tiles and simplifies every level of them (on @p threads threads). The map
  /// is taken by value: pass it with std::move when it is no longer needed, so that it is cut in
  /// place instead of copied (a large map would be in memory twice).
  Stats build(::navmap::NavMap map, const LodParams & params, unsigned threads = 1);

  bool empty() const {return nodes_.empty();}
  int levels() const {return top_ + 1;}
  /// Cell size of a level (its error is half of it)
  double cell(int level) const;
  /// From this distance (m, on the plane) a tile may be of this level
  double radius(int level) const;
  /// The level of the places at distance d (m) from the robot
  int level_at(double d) const;
  const LodParams & params() const {return p_;}

  /// The tiles to put together around (x, y): the coarsest each place allows
  std::vector<std::uint32_t> select(double x, double y) const;
  /// Those tiles in one NavMap, their borders stitched (without geometry accelerations if
  /// @p accels is false: detail() builds them)
  ::navmap::NavMap assemble(const std::vector<std::uint32_t> & tiles, bool accels = true) const;

  /**
   * @brief The detail of the obstacles around (x, y), on a map assemble() made there.
   *
   * Within min_radius, the borders of the map (walls, drops, the edges of the holes) and the
   * @p obstacles (points on the surface, e.g. sensed) are refined (navmap::refine) to cell_min,
   * and so is every NavCel within @p margin (m; at least cell_min) of them: what an inflation of
   * that radius reads. Farther, the obstacles get the cell size of their place. The NavCels of
   * the obstacles are marked LETHAL_OBSTACLE in the layer "obstacles" (created if absent). The
   * geometry accelerations of @p nm are built at the end.
   */
  void detail(
    ::navmap::NavMap & nm, double x, double y, const std::vector<Eigen::Vector3f> & obstacles,
    double margin = 0.0) const;

  /// Level of a tile
  int level(std::uint32_t tile) const {return nodes_[tile].level;}
  /// Bounds of a tile: x0, x1, y0, y1 (m)
  std::array<float, 4> bounds(std::uint32_t tile) const
  {
    const auto & l = nodes_[tile].lines;
    return {xs_[l[0]], xs_[l[1]], ys_[l[2]], ys_[l[3]]};
  }

private:
  struct Node
  {
    int level{0};
    std::array<int, 4> lines{0, 0, 0, 0};   ///< its borders: xs_[x0], xs_[x1], ys_[y0], ys_[y1]
    std::array<int, 4> child{-1, -1, -1, -1};
    ::navmap::NavMap mesh;
  };

  ::navmap::NavMap stitch(const std::vector<std::uint32_t> & nodes, bool accels) const;
  std::vector<::navmap::PointId> border_locks(const Node & n) const;
  void simplify_node(Node & n) const;
  double distance(const Node & n, double x, double y) const;

  LodParams p_;
  int top_{0};
  std::vector<float> xs_, ys_;   // grid lines, increasing
  std::vector<Node> nodes_;
  std::vector<std::uint32_t> roots_;
};

}  // namespace bigmap
}  // namespace easynav

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__TILED_LOD_HPP_
