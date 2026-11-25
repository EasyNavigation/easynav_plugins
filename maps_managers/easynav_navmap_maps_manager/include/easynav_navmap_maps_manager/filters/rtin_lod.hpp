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
/// \brief Merge/split level of detail of a grid NavMap driven by the values of one layer.

#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__RTIN_LOD_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__RTIN_LOD_HPP_

#include <cstdint>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"

namespace easynav
{
namespace navmap
{

/**
 * @brief Simplified copy of a grid NavMap: same-valued neighbour NavCels merged, split back where
 * the values change.
 *
 * The source must be a regular grid of right isosceles triangles (two per square cell, legs =
 * resolution), as pointcloud_to_navmap writes. A binary triangle tree (RTIN) is laid over it: two
 * sibling triangles merge into their parent (twice the area) when all the cells below have the
 * same value of the layer, the same height (within a tolerance) and no hole; otherwise the
 * triangle is split in two along its hypotenuse, down to the cell halves. A split forces the
 * neighbour sharing the hypotenuse to split too, so the mesh has no T-junctions and every edge
 * has its neighbour (A* crosses between levels).
 *
 * A finest triangle may use the other diagonal of its cell than the source: then it takes the
 * highest value of the two source halves and exists only if both exist.
 */
class RtinLod
{
public:
  struct Params
  {
    float max_angle_deg{5.0f};       ///< deg: merged NavCels differ at most this in slope
    float height_tolerance{0.05f};   ///< m: source vertices at most this far from a merged
                                     ///< NavCel's plane (a bound, accumulated per merge)
    float tile_size{50.0f};          ///< m: side of the tiles extract() caches (about)
  };

  /// Statistics of the last extraction.
  struct Stats
  {
    std::size_t triangles{0};       ///< NavCels of the simplified map
    std::size_t finest{0};          ///< ...of them at the source resolution
    int max_level{0};               ///< largest number of merges of one NavCel
    std::size_t tiles{0};           ///< tiles with NavCels
    std::size_t tiles_rebuilt{0};   ///< ...walked again in this extraction (the rest reused)
    std::size_t dead{0};            ///< dead NavCels in the map (degenerate, unreachable)
    double ms_mesh{0.0};            ///< time walking the changed tiles of the tree
    double ms_assemble{0.0};        ///< time writing the changed tiles into the NavMap
    double ms_finish{0.0};          ///< time of NavMap's surface indices
  };

  /// Builds the tree over `src` with the values of `layer` (uint8). false + `error` if `src` is
  /// not a regular grid of right isosceles triangles or has no such layer.
  bool build(
    const ::navmap::NavMap & src, const std::string & layer, const Params & params,
    std::string * error = nullptr);

  /// Reads the layer again and merges/splits where it changed. Returns the source NavCels whose
  /// value changed (0: nothing to do). The source must be the same map given to build().
  std::size_t update(const ::navmap::NavMap & src);

  /// As update(), reading only the NavCels within `radius` of (x, y) (where the sensors change
  /// the costs) and the next `sweep` NavCels of the whole map in turns (what changes far away,
  /// as an obstacle memory forgetting, is caught within source_navcels() / sweep calls). The
  /// cost does not depend on the size of the map.
  std::size_t update_window(
    const ::navmap::NavMap & src, double x, double y, double radius, std::size_t sweep);

  /// Builds the simplified NavMap: layers `<layer>` (uint8, the value of each NavCel),
  /// `lod_level` (uint8, merges: 0 = source resolution, each level doubles the area) and
  /// `modified` (uint8, 1 if the NavCel is not at the source resolution). One surface per tile
  /// of the tree (about tile_size m). The map is the same object every call, edited in place:
  /// only the tiles where something changed since the last call are walked and written again
  /// (into their range of NavCels, or a new one at the end when they grew); the others are left
  /// as they are. NavCels a tile no longer uses are dead: degenerate (all on vertex 0), without
  /// neighbours, in no surface, value 254; when they outnumber the live ones the map is written
  /// again from the tiles' cache. Whoever holds the pointer must not read it while extract() runs.
  std::shared_ptr<::navmap::NavMap> extract(Stats * stats = nullptr);

  /// What extract() changed in the output since the last take_changes(), to send only that.
  struct Changes
  {
    bool full{true};                    ///< a new output (first, rebuilt or compacted): send it all
    std::uint32_t first_vertex{0};      ///< vertices from here on are new
    std::vector<std::uint32_t> navcels;  ///< rewritten NavCels (sorted)
    std::vector<std::size_t> surfaces;  ///< replaced surfaces (sorted)
  };

  /// The changes accumulated since the last call (or since the output was created), and starts
  /// accumulating again.
  Changes take_changes();

  bool ready() const {return built_;}
  double resolution() const {return res_;}
  std::size_t source_navcels() const {return src_count_;}

private:
  static constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();
  // node state bits
  static constexpr std::uint8_t kExists = 1;    ///< leaf: covered by the source
  static constexpr std::uint8_t kEmpty = 2;     ///< no leaf below exists
  static constexpr std::uint8_t kUniform = 4;   ///< complete, flat, one value: may stay merged
  static constexpr std::uint8_t kFlat = 8;      ///< static: one plane (angle and deviation)
  static constexpr std::uint8_t kComplete = 16;  ///< static: every leaf below exists
  static constexpr std::uint8_t kSplit = 32;    ///< split in the current mesh

  struct Tri
  {
    int ax, ay, bx, by, cx, cy;   ///< doubled grid units; a-b hypotenuse, c right angle
  };

  void leaf_state(std::uint32_t k, const Tri & t);
  bool need_split(std::uint32_t k) const;
  const std::vector<std::uint8_t> * layer_data(const ::navmap::NavMap & src) const;
  bool read(std::uint32_t i, const std::vector<std::uint8_t> & v);   ///< queues its leaves
  std::size_t apply(std::size_t changed);   ///< merges/splits for the queued leaves
  void refresh_leaf(std::uint32_t k, std::size_t cell, int slot);
  bool recompute_merge(std::uint32_t k);
  /// The NavCels of one tile (or of the top of the tree), in local numbering
  struct Tile
  {
    std::vector<std::uint32_t> grid;          ///< per local vertex: its cell corner (shared by
                                              ///< the tiles once assembled)
    std::vector<::navmap::NavCel> cells;      ///< local vertices and neighbours (kNone: border)
    std::vector<std::uint8_t> values, levels;
    std::vector<::navmap::BVHNode> bvh;       ///< root at 0; leaves index `prim`
    std::vector<int> prim;                    ///< local NavCel ids
    std::vector<std::pair<std::uint32_t, std::uint32_t>> border;  ///< (edge midpoint, cell*3+e)
    std::size_t finest{0};
    int max_level{0};
    bool dirty{true};
  };
  /// Emits the NavCels under node k into `tile`; returns its BVH node (-1: none). With
  /// `stop_depth`, nodes at that depth are not walked (the top of the tree).
  int extract_node(std::uint32_t k, const Tri & t, int depth, int stop_depth, Tile & tile);
  void extract_tile(std::size_t i);
  void reset_output();
  void write_tile(std::size_t i);
  void kill(std::uint32_t from, std::uint32_t to);   ///< NavCels [from, to) dead
  void mark(std::uint32_t k, bool split_changed);   ///< the tiles whose mesh changes with k
  Tri node_tri(std::uint32_t k, int depth) const;
  static Tri child(const Tri & t, int side);
  float zat(int gx, int gy) const;   ///< height of a cell corner (cell grid units)

  bool built_{false};
  std::string layer_;
  Params params_;
  double res_{0.0}, ox_{0.0}, oy_{0.0};
  int n_{0};                 ///< cells per side of the padded square (power of two)
  int leaf_depth_{0};        ///< depth of the cell halves (roots: 1)
  std::size_t src_count_{0};

  std::vector<float> z_;                   ///< (n+1)^2 cell corner heights (NaN: none)
  std::vector<std::uint32_t> cell_src_;    ///< 2 per cell: source NavCel of each half, or kNone
  std::vector<std::uint8_t> cell_diag_;    ///< per cell: 1 if the source halves split it by the
                                           ///< main diagonal ((0,0)-(1,1)), 0 the other one
  std::vector<std::uint32_t> cell_leaf_;   ///< 2 per cell: the tree's leaves on it
  std::vector<std::uint8_t> cell_leaf_main_;  ///< per cell: 1 if the tree splits it by the main
                                              ///< diagonal
  std::vector<std::uint32_t> src_cell_;    ///< per source NavCel: its cell
  std::vector<std::uint8_t> src_value_;    ///< per source NavCel: last value read
  std::vector<std::uint64_t> pending_;     ///< leaves (cell << 1 | half) to refresh
  std::size_t cursor_{0};                  ///< next source NavCel of the sweep

  std::vector<std::uint8_t> value_;        ///< per tree node: its value (if uniform)
  std::vector<std::uint8_t> state_;        ///< per tree node: k* bits
  std::vector<std::uint32_t> partner_;     ///< per tree node: the one sharing its hypotenuse

  int tile_depth_{1};                      ///< depth of the tiles' root nodes
  std::vector<Tile> tiles_;                ///< 2^tile_depth_ tiles + the top of the tree (last)

  // the output, edited in place
  struct Slot
  {
    std::uint32_t base{0}, cap{0}, count{0};   ///< its range of NavCels
    std::vector<std::pair<std::uint32_t, std::uint32_t>> border;  ///< (midpoint, NavCel*3+e)
  };
  std::shared_ptr<::navmap::NavMap> out_;
  std::vector<Slot> slots_;                ///< per tile
  std::unordered_map<std::uint32_t, std::array<std::uint32_t, 2>> border_edges_;
  std::vector<std::uint32_t> gvert_;       ///< (n+1)^2 cell corners: output vertex
  std::vector<std::uint32_t> local_to_global_;
  std::size_t dead_{0};
  // changes since the last take_changes()
  bool changes_full_{true};
  std::uint32_t changes_first_vertex_{0};
  std::vector<std::pair<std::uint32_t, std::uint32_t>> changed_ranges_;   ///< [from, to)
  std::vector<std::size_t> changed_surfaces_;
  std::shared_ptr<::navmap::LayerView<std::uint8_t>> values_l_, levels_l_, modified_l_;

  // scratch of extract(), kept empty between calls (only the used entries are reset)
  std::vector<std::uint32_t> vmap_;        ///< (n+1)^2 cell corners: tile vertex
  std::vector<std::uint32_t> edge_;        ///< (2n+1)^2 edge midpoints: first NavCel * 3 + edge
  std::vector<std::uint32_t> touched_v_, touched_e_;
};

}  // namespace navmap
}  // namespace easynav

#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__RTIN_LOD_HPP_
