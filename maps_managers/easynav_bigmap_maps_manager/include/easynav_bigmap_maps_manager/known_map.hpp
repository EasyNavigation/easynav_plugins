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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__KNOWN_MAP_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__KNOWN_MAP_HPP_

#include <atomic>
#include <cstdint>
#include <vector>

namespace bigmap_maps_manager
{

/// The DEM nodes the robot has driven over: ground it has proved navigable. The mesher keeps
/// the NavCels there at full resolution, however far the robot is now.
///
/// distance() is the exact Euclidean distance to the nearest traversed node: a 1-Lipschitz
/// function of position, which is all the crack-free argument of the LOD policy needs. It is
/// answered from a 2-D tree of the traversed nodes (built by update()), not from a distance
/// field of the whole grid: the mesher asks for a few tens of thousands of nodes per re-mesh.
class KnownMap
{
public:
  /// `side` x `side` DEM nodes, `resolution` m apart, node (side-1)/2 at the origin. Only those
  /// nodes can be marked; distance() is exact for any node, the padding of the RTIN tree (2^k + 1)
  /// included, which keeps it 1-Lipschitz there too (the crack-free argument needs it). `grid`:
  /// unused (it sized the marks for the padding; kept for source compatibility).
  KnownMap(int side, double resolution, int grid = 0);

  /// Marks the nodes within `radius` m of (x, y) (local frame, metres). Returns how many were new.
  int mark(double x, double y, double radius);

  /// Marks one node. Returns true if it was new.
  bool mark_node(int ix, int iy);

  /// Recomputes the distance field from all the traversed nodes, if something was marked since
  /// the last call.
  void update();

  /// The same, but only the traversed nodes within `radius` m of (x, y) count: ground the robot
  /// drove over farther away than that is forgotten as far as resolution goes (it stays marked).
  void update_within(double x, double y, double radius);

  /// Forgets (unmarks) the traversed nodes farther than `radius` m from (x, y): they are ground
  /// the robot has not driven over as far as the map is concerned, until it drives over it again.
  /// Returns how many were forgotten.
  std::size_t forget_outside(double x, double y, double radius);

  /// True if a traversed node lies within `reach` nodes (Chebyshev) of node (ix, iy).
  bool near_marked(int ix, int iy, int reach) const;

  /// Nodes the distance field is currently measured from.
  std::size_t sources() const {return sources_;}

  bool empty() const {return count_ == 0;}
  std::size_t count() const {return count_;}

  /// Distance (m) from node (ix, iy) to the nearest traversed node (any indices, also beyond the
  /// grid). Infinite when nothing was traversed. Call update() first.
  double distance(int ix, int iy) const;

  /// True if the node (ix, iy) itself was traversed.
  bool known(int ix, int iy) const;

private:
  int side_;   // nodes per axis of the field (the DEM's, or the padded grid's)
  double res_;
  int center_;
  // read by the save services from another thread (has_known): atomic, so that the read is
  // well defined (the mesher's thread is the only writer)
  std::atomic<std::size_t> count_{0};
  std::size_t sources_{0};
  std::vector<std::uint32_t> nodes_;  // marked nodes, in the order they were marked
  void compute(const std::vector<std::uint32_t> & sources);
  bool dirty_{false};
  std::vector<std::uint8_t> mark_;
  // the sources as a balanced 2-D tree over this array (see compute): the node of a range
  // [lo, hi) is its middle element, split along x or y
  struct Node
  {
    int x, y;
    bool by_x{true};
  };
  std::vector<Node> tree_;
  void build(std::size_t lo, std::size_t hi);
  void nearest(std::size_t lo, std::size_t hi, int x, int y, double & best) const;
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__KNOWN_MAP_HPP_
