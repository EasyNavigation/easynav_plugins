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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__SURFACE_LOD_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__SURFACE_LOD_HPP_

#include <atomic>
#include <cstdint>
#include <vector>

#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/lod_policy.hpp"
#include "easynav_bigmap_maps_manager/navmap_dem.hpp"
#include "navmap_core/NavMap.hpp"

namespace bigmap_maps_manager
{

/// Level of detail for a multi-level surface (floors that overlap in x, y, ramps, any NavMap):
/// view-dependent simplification by hierarchical vertex clustering. It works on the NavMap's own
/// triangles, not on a height grid, so several floors over the same (x, y) are not a problem.
///
/// Built once: level 0 are the (welded) vertices of the map; level k merges the nodes of level
/// k-1 that fall in the same cell of size `min_size * 2^(k-1)` and whose heights span at most
/// `cluster_height` (so two floors are never merged). Each node keeps the mean position and
/// colour of its vertices and the error of replacing them by it. The vertices on the border of
/// the mesh (the sides of a ramp, a hole) only merge with each other, so the border stays where it
/// is instead of shrinking by half a cell: a narrow ramp keeps its width.
///
/// Per extraction every vertex climbs the tree while the nodes allow it (the policy of LodParams:
/// distance in 3D to the robot, to the ground it has driven over, and the node's error). The
/// result is a cut of the tree, and a triangle is the one its three vertices end in: the mesh is
/// closed wherever the source is (no cracks between different sizes, no seams between floors and
/// ramps), whatever the geometry.
///
/// Incremental: far from the robot and from the driven-over / observed ground a node's decision
/// does not depend on where the robot is (see `dynamic radius` in the source). That cut and its
/// triangles are computed once; an extraction only climbs again the vertices under the nodes near
/// the robot or that ground, and only remakes the triangles they touch. Its cost is that of the
/// region around the robot, not of the map. extract_reference() is the full algorithm (the same
/// result, for the tests).
class SurfaceLodMesher
{
public:
  struct Config
  {
    double min_size{1.0};          ///< size of the smallest cluster (leaf) cells, m
    double max_size{32.0};         ///< size of the largest ones
    double cluster_height{1.2};    ///< a cluster never spans more height than this, m
    double vertical_tolerance{1.5};  ///< driven-over ground is the one this close in z, m
    /// Height counts this many times in the distance to the robot: the floor above or below is
    /// farther than its 3D distance says
    double vertical_weight{4.0};
    /// A corridor, ramp or door narrower than this (m) is kept at the full resolution of the map
    /// however far the robot is: coarse NavCels in a narrow passage leave no room for the robot's
    /// inflation and the planner could not find the way through. 0: off.
    double keep_narrow_width{0.0};
    /// Only the largest piece of the map (triangles joined through shared edges) is used: the roof
    /// and any other surface the robot cannot get to are dropped. A goal picked on them (the
    /// topmost surface under the mouse) could never be reached.
    bool drop_unreachable_ground{false};
    /// The clusters on the border of the mesh (holes: obstacles; edges) count as an error of at
    /// least this much (m), so they stay fine as far as the tolerance is below it and a hole is
    /// not closed by a coarse cluster near the robot. 0: off.
    double hole_error{0.0};
  };

  SurfaceLodMesher(const SourceMesh & map, const Config & config);
  SurfaceLodMesher(const ::navmap::NavMap & map, const Config & config);

  SurfaceLodMesher(const SurfaceLodMesher &) = delete;
  SurfaceLodMesher & operator=(const SurfaceLodMesher &) = delete;

  /// Marks as driven over the vertices within `radius` (xy) of (x, y) and `vertical_tolerance`
  /// of z. Returns how many were new.
  int mark(double x, double y, double z, double radius);
  /// Forgets the driven-over vertices farther than `radius` (3D) from (x, y, z).
  std::size_t forget_outside(double x, double y, double z, double radius);
  std::size_t traversed_count() const {return n_known_;}
  /// The source map had a `traversed` layer
  bool had_traversed_layer() const {return had_traversed_;}

  /// Marks as observed the border vertices (holes, edges: obstacles) within `radius` (xy) of
  /// (x, y) and `vertical_tolerance` of z: they stay at full resolution like the driven-over
  /// ground (within LodParams::observed_margin), wherever the robot goes: a hole seen from near
  /// keeps its shape from far, so the plan does not change with the robot's position and make it
  /// come back. Returns how many were new.
  int mark_observed(double x, double y, double z, double radius);
  /// Forgets the observed vertices farther than `radius` (3D) from (x, y, z).
  std::size_t forget_observed_outside(double x, double y, double z, double radius);
  std::size_t observed_count() const {return observed_list_.size();}

  LodMesh extract(double robot_x, double robot_y, double robot_z, const LodParams & p) const;
  /// The full algorithm (every vertex climbs, every source triangle is mapped): the same mesh as
  /// extract(), up to the order of the vertices and triangles. For the tests.
  LodMesh extract_reference(
    double robot_x, double robot_y, double robot_z, const LodParams & p) const;

  std::size_t vertex_count() const {return levels_[0].x.size();}
  std::size_t narrow_vertex_count() const {return n_pinned_;}
  std::size_t dropped_triangles() const {return n_dropped_;}
  std::size_t source_triangle_count() const {return tri_.size() / 3;}
  int level_count() const {return static_cast<int>(levels_.size());}
  double source_edge() const {return source_edge_;}
  bool coloured() const {return coloured_;}
  /// Bytes held by the hierarchy (levels, triangles, indexes, the precomputed far cut)
  std::size_t memory_bytes() const;

private:
  struct Level
  {
    std::vector<float> x, y, z;
    std::vector<float> err;              ///< error of the node, m (empty for level 0)
    std::vector<std::uint8_t> r, g, b;   ///< mean colour (empty if the map has none)
    std::vector<std::uint8_t> flags;     ///< kBoundary | kPinned
    std::vector<std::uint32_t> parent;   ///< node of the next level
    std::vector<std::uint32_t> child_start, child;  ///< nodes of the previous level (CSR)
    double size{0.0};
    float max_err{0.0f};                 ///< the largest err of the level
  };
  static constexpr std::uint8_t kBoundary = 1, kPinned = 2;

  /// Items (vertices or nodes) by square cell of `cell` m, as a CSR over the sorted cell keys
  struct CellIndex
  {
    double cell{2.0};
    std::vector<std::int64_t> keys;
    std::vector<std::uint32_t> start, items;
    void build(double cell_size, const std::vector<float> & x, const std::vector<float> & y);
    /// Calls f(item) for the items of the cells that meet the disc (a superset of the disc)
    template<class F>
    void visit_disc(double x, double y, double radius, F && f) const;
    template<class F>
    void visit_cell(std::int64_t key, F && f) const;
    std::size_t memory_bytes() const;
  };

  struct KdNode
  {
    float x, y, z;
    bool by_x;
  };
  struct Query;

  void init(const SourceMesh & map);
  void build_levels();
  void pin_narrow_passages();
  void build_far_cut(const LodParams & p) const;
  bool far_cut_matches(const LodParams & p) const;
  void next_epoch() const;
  int level_of(std::uint32_t global) const;
  std::uint32_t static_leaf(std::uint32_t v) const;
  std::uint32_t climb(std::uint32_t v, const Query & q) const;

  bool static_allowed(int k, std::uint32_t c) const;
  bool allowed_now(int k, std::uint32_t c, const Query & q) const;
  bool allowed_memo(int k, std::uint32_t c, const Query & q) const;
  void build_kd() const;
  static void kd_build(std::vector<KdNode> & kd, std::size_t lo, std::size_t hi);
  void kd_nearest(
    const std::vector<KdNode> & kd, std::size_t lo, std::size_t hi, double x, double y, double z,
    double & best) const;
  double near_distance(
    const std::vector<KdNode> & kd, double x, double y, double z, double reach) const;

  Config cfg_;
  bool coloured_{false};
  double source_edge_{0.0};  ///< median edge length of the source, m
  std::size_t n_pinned_{0};
  std::size_t n_dropped_{0};
  bool had_traversed_{false};
  std::vector<Level> levels_;
  std::vector<std::size_t> offset_;      // global id of node (k, c) = offset_[k] + c
  std::vector<std::uint32_t> tri_;       // welded vertex ids, 3 per source triangle
  std::vector<std::uint16_t> tri_slope_;  // per source triangle: degrees x 100, rounded up
  std::vector<std::uint32_t> vtx_tri_start_, vtx_tri_;  // the source triangles of each vertex
  std::vector<std::uint8_t> known_;      // per vertex: driven over
  std::vector<std::uint32_t> known_list_;
  std::atomic<std::size_t> n_known_{0};  // read by the save services from another thread
  std::vector<std::uint8_t> observed_;   // per vertex: an obstacle border seen near
  std::vector<std::uint32_t> observed_list_;
  CellIndex grid_;                       // vertices by 2 m cell
  std::vector<CellIndex> node_grid_;     // per level (k >= 1): nodes by cell
  static constexpr double kGridCell = 2.0;

  // The driven-over and the observed vertices as 2-D trees, rebuilt when they change
  mutable std::vector<KdNode> kd_known_, kd_observed_;
  mutable bool kd_dirty_{true};

  // The far cut: the one of a robot infinitely far, with no driven-over or observed ground (it
  // depends on the policy's min_size, max_size, base_error and error_growth only, and is rebuilt
  // when they change). Its triangles (global node ids, in the order of the first source triangle
  // that ends in each), the steepest source slope, the largest node size, their source triangles
  // (CSR, in source order) and, per source triangle, its far triangle (kNone: degenerate)
  mutable bool far_built_{false};
  mutable double far_min_{0.0}, far_max_{0.0}, far_base_{0.0}, far_growth_{0.0};
  mutable std::vector<std::uint32_t> far_tri_;
  mutable std::vector<std::uint16_t> far_slope_;
  mutable std::vector<float> far_size_;
  mutable std::vector<std::uint32_t> far_src_start_, far_src_, far_of_src_;
  static constexpr std::uint32_t kNone = 0xffffffffu;

  // Scratch of extract() (not thread-safe: one extraction at a time), reset by stamps
  mutable std::uint32_t epoch_{0};
  mutable std::vector<std::uint32_t> node_stamp_;   // per global node: decision memoised
  mutable std::vector<std::uint8_t> node_memo_;     // the decision
  mutable std::vector<std::uint32_t> walk_stamp_;   // per global node: subtree collected
  mutable std::vector<std::uint32_t> slot_stamp_, slot_;  // per global node: output vertex
  mutable std::vector<std::uint32_t> known_stamp_;  // per global node: holds a driven-over vertex
  mutable std::vector<std::uint32_t> vtx_stamp_;    // per vertex: climbed again
  mutable std::vector<std::uint32_t> cur_node_;     // its node (global id), if climbed again
  mutable std::vector<std::uint32_t> far_stamp_;    // per far triangle: remade
  mutable std::vector<std::uint32_t> src_stamp_;    // per source triangle: remade
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__SURFACE_LOD_HPP_
