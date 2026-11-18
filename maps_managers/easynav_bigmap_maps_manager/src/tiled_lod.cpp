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

#include "easynav_bigmap_maps_manager/tiled_lod.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navmap_core/Costs.hpp"
#include "navmap_core/Simplify.hpp"
#include "navmap_core/Subdivide.hpp"

namespace easynav
{
namespace bigmap
{

namespace
{

using ::navmap::NavCel;
using ::navmap::NavCelId;
using ::navmap::NavMap;
using ::navmap::PointId;

// Calls f(view) with the typed view of @p base (no-op for an unknown type).
template<class F>
void with_view(const std::shared_ptr<::navmap::LayerViewBase> & base, F && f)
{
  using ::navmap::LayerView;
  if (auto v = std::dynamic_pointer_cast<LayerView<std::uint8_t>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<float>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<double>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<std::int16_t>>(base)) {f(*v); return;}
}

// Runs fn(i) for i in [0, n) on up to `threads` threads
void parallel_for(std::size_t n, unsigned threads, const std::function<void(std::size_t)> & fn)
{
  threads = std::max(1u, std::min<unsigned>(threads, static_cast<unsigned>(n)));
  if (threads <= 1) {
    for (std::size_t i = 0; i < n; ++i) {
      fn(i);
    }
    return;
  }
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < threads; ++t) {
    pool.emplace_back(
      [&]() {
        for (std::size_t i = next++; i < n; i = next++) {
          fn(i);
        }
      });
  }
  for (auto & th : pool) {
    th.join();
  }
}

// The surface of each NavCel
std::vector<std::size_t> surface_of(const NavMap & nm)
{
  std::vector<std::size_t> s(nm.navcels.size(), 0);
  for (std::size_t k = 0; k < nm.surfaces.size(); ++k) {
    for (const NavCelId c : nm.surfaces[k].navcels) {
      if (c < s.size()) {s[c] = k;}
    }
  }
  return s;
}

// A NavMap of some NavCels of `src` (vertices compacted) with their layers, surfaces, colours
NavMap subset(
  const NavMap & src, const std::vector<NavCelId> & cels, const std::vector<std::size_t> & surf)
{
  NavMap out;
  for (const auto & s : src.surfaces) {
    out.create_surface(s.frame_id);
  }
  const bool colored = src.colors && src.colors->r.size() == src.positions.size();
  if (colored) {out.colors.emplace();}
  std::unordered_map<PointId, PointId> vmap;
  vmap.reserve(cels.size() * 2);
  for (const NavCelId c : cels) {
    NavCel cel;
    for (int k = 0; k < 3; ++k) {
      const PointId v = src.navcels[c].v[k];
      auto [it, fresh] = vmap.try_emplace(v, static_cast<PointId>(out.positions.size()));
      if (fresh) {
        out.positions.x.push_back(src.positions.x[v]);
        out.positions.y.push_back(src.positions.y[v]);
        out.positions.z.push_back(src.positions.z[v]);
        if (colored) {
          out.colors->r.push_back(src.colors->r[v]);
          out.colors->g.push_back(src.colors->g[v]);
          out.colors->b.push_back(src.colors->b[v]);
          out.colors->a.push_back(src.colors->a[v]);
        }
      }
      cel.v[k] = it->second;
    }
    out.surfaces[surf[c]].navcels.push_back(static_cast<NavCelId>(out.navcels.size()));
    out.navcels.push_back(cel);
  }
  for (const auto & name : src.list_layers()) {
    with_view(
      src.layers.get(name), [&](const auto & v) {
        using T = std::decay_t<decltype(v.data()[0])>;
        if (v.size() != src.navcels.size()) {return;}
        auto dst = out.add_layer<T>(name);
        for (std::size_t i = 0; i < cels.size(); ++i) {
          dst->data()[i] = v.data()[cels[i]];
        }
      });
  }
  out.layer_meta = src.layer_meta;
  return out;
}

// What a tile keeps of a NavMap: its geometry, surfaces, layers and colours. Not the
// accelerations (BVH, adjacency, locate grids), which only the assembled map needs: with tens of
// thousands of tiles they would be most of the memory
NavMap slim(NavMap && m)
{
  NavMap out;
  out.positions = std::move(m.positions);
  out.positions.x.shrink_to_fit();
  out.positions.y.shrink_to_fit();
  out.positions.z.shrink_to_fit();
  out.colors = std::move(m.colors);
  out.navcels.resize(m.navcels.size());
  for (std::size_t i = 0; i < m.navcels.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      out.navcels[i].v[k] = m.navcels[i].v[k];
    }
  }
  for (auto & s : m.surfaces) {
    out.create_surface(s.frame_id);
    out.surfaces.back().navcels = std::move(s.navcels);
  }
  out.layers = std::move(m.layers);
  out.layer_meta = std::move(m.layer_meta);
  return out;
}

// Exact position of a vertex, as a hash key
struct Key
{
  float x, y, z;
  bool operator==(const Key & o) const {return x == o.x && y == o.y && z == o.z;}
};
struct KeyHash
{
  std::size_t operator()(const Key & k) const
  {
    std::uint32_t a, b, c;
    std::memcpy(&a, &k.x, 4);
    std::memcpy(&b, &k.y, 4);
    std::memcpy(&c, &k.z, 4);
    return (static_cast<std::size_t>(a) * 0x9E3779B97F4A7C15ull) ^
           (static_cast<std::size_t>(b) * 0xC2B2AE3D27D4EB4Full) ^ (c * 0x165667B19E3779F9ull);
  }
};

int line_index(const std::vector<float> & lines, float v)
{
  auto it = std::lower_bound(lines.begin(), lines.end(), v);
  return it != lines.end() && *it == v ? static_cast<int>(it - lines.begin()) : -1;
}

}  // namespace

double TiledLod::cell(int level) const
{
  return std::min(p_.cell_min * std::pow(2.0, level), p_.cell_max);
}

double TiledLod::radius(int level) const
{
  if (level <= 0) {return 0.0;}
  if (top_ <= 1) {return p_.min_radius;}
  return p_.min_radius *
         std::pow(p_.max_radius / p_.min_radius, static_cast<double>(level - 1) / (top_ - 1));
}

int TiledLod::level_at(double d) const
{
  int k = 0;
  while (k < top_ && d >= radius(k + 1)) {++k;}
  return k;
}

TiledLod::Stats TiledLod::build(NavMap map, const LodParams & params, unsigned threads)
{
  const auto t0 = std::chrono::steady_clock::now();
  p_ = params;
  p_.cell_min = std::max(1e-3, p_.cell_min);
  p_.cell_max = std::max(p_.cell_min, p_.cell_max);
  p_.min_radius = std::max(0.0, p_.min_radius);
  p_.max_radius = std::max(p_.min_radius, p_.max_radius);
  const double tile = p_.tile > 0.0 ? p_.tile : std::max(p_.min_radius / 2.0, 4.0 * p_.cell_min);
  p_.tile = tile;
  top_ = p_.cell_max > p_.cell_min ?
    static_cast<int>(std::ceil(std::log2(p_.cell_max / p_.cell_min) - 1e-9)) : 0;
  nodes_.clear();
  roots_.clear();
  xs_.clear();
  ys_.clear();
  Stats st;
  st.navcels.assign(top_ + 1, 0);
  if (map.navcels.empty()) {return st;}

  // The grid
  float lo[2] = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
  float hi[2] = {-lo[0], -lo[1]};
  for (std::size_t i = 0; i < map.positions.size(); ++i) {
    lo[0] = std::min(lo[0], map.positions.x[i]);
    hi[0] = std::max(hi[0], map.positions.x[i]);
    lo[1] = std::min(lo[1], map.positions.y[i]);
    hi[1] = std::max(hi[1], map.positions.y[i]);
  }
  int n[2];
  double origin[2];
  for (int a = 0; a < 2; ++a) {
    origin[a] = std::floor(lo[a] / tile) * tile;
    n[a] = std::max(1, static_cast<int>(std::floor((hi[a] - origin[a]) / tile)) + 1);
    auto & lines = a == 0 ? xs_ : ys_;
    for (int i = 0; i <= n[a]; ++i) {
      lines.push_back(static_cast<float>(origin[a] + i * tile));
    }
  }
  NavMap cut = std::move(map);   // cut in place
  ::navmap::cut_along_lines(
    cut, {{xs_.begin() + 1, xs_.end() - 1}, {ys_.begin() + 1, ys_.end() - 1}});

  // The finest tiles: the NavCels whose centroid is in each square
  std::map<std::pair<int, int>, std::vector<NavCelId>> squares;
  for (NavCelId c = 0; c < cut.navcels.size(); ++c) {
    const auto q = cut.navcel_centroid(c);
    const int i = std::clamp(static_cast<int>(std::floor((q.x() - origin[0]) / tile)), 0, n[0] - 1);
    const int j = std::clamp(static_cast<int>(std::floor((q.y() - origin[1]) / tile)), 0, n[1] - 1);
    squares[{i, j}].push_back(c);
  }
  const auto surf = surface_of(cut);
  std::vector<std::vector<NavCelId>> cels_of;
  for (auto & [ij, cels] : squares) {
    Node node;
    node.level = 0;
    node.lines = {ij.first, ij.first + 1, ij.second, ij.second + 1};
    nodes_.push_back(std::move(node));
    cels_of.push_back(std::move(cels));
  }
  squares.clear();
  st.tiles = nodes_.size();
  // each tile taken from the cut map just before it is simplified: the unsimplified tiles are
  // never all in memory at once
  parallel_for(
    nodes_.size(), threads, [this, &cut, &cels_of, &surf](std::size_t i) {
      nodes_[i].mesh = subset(cut, cels_of[i], surf);
      cels_of[i] = {};
      simplify_node(nodes_[i]);
    });
  cut = NavMap();

  // Each level from the one below: four tiles put together, simplified with twice the error
  std::size_t level_begin = 0;
  for (int k = 1; k <= top_; ++k) {
    const std::size_t level_end = nodes_.size();
    std::map<std::pair<int, int>, std::vector<std::uint32_t>> parents;
    for (std::size_t i = level_begin; i < level_end; ++i) {
      const auto & c = nodes_[i];
      parents[{c.lines[0] >> k, c.lines[2] >> k}].push_back(static_cast<std::uint32_t>(i));
    }
    const std::size_t first = nodes_.size();
    for (const auto & [ij, kids] : parents) {
      Node node;
      node.level = k;
      node.lines = {ij.first << k, std::min((ij.first + 1) << k, n[0]),
          ij.second << k, std::min((ij.second + 1) << k, n[1])};
      for (const auto kid : kids) {
        const auto & c = nodes_[kid];
        const int q = ((c.lines[0] >> (k - 1)) & 1) + 2 * ((c.lines[2] >> (k - 1)) & 1);
        node.child[q] = static_cast<int>(kid);
      }
      nodes_.push_back(std::move(node));
    }
    parallel_for(
      nodes_.size() - first, threads, [this, first](std::size_t i) {
        auto & node = nodes_[first + i];
        std::vector<std::uint32_t> kids;
        for (const int c : node.child) {
          if (c >= 0) {kids.push_back(static_cast<std::uint32_t>(c));}
        }
        node.mesh = stitch(kids, false);
        simplify_node(node);
      });
    level_begin = level_end;
  }
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    st.navcels[nodes_[i].level] += nodes_[i].mesh.navcels.size();
    if (nodes_[i].level == top_) {roots_.push_back(static_cast<std::uint32_t>(i));}
  }
  st.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return st;
}

std::vector<PointId> TiledLod::border_locks(const Node & n) const
{
  const auto & m = n.mesh;
  const float x0 = xs_[n.lines[0]], x1 = xs_[n.lines[1]];
  const float y0 = ys_[n.lines[2]], y1 = ys_[n.lines[3]];
  auto sides = [&](PointId v) {
      const float x = m.positions.x[v], y = m.positions.y[v];
      return (x == x0 ? 1 : 0) | (x == x1 ? 2 : 0) | (y == y0 ? 4 : 0) | (y == y1 ? 8 : 0);
    };
  std::vector<char> lock(m.positions.size(), 0);
  for (PointId v = 0; v < m.positions.size(); ++v) {
    const int s = sides(v);
    if (s && (s & (s - 1))) {lock[v] = 1;}   // a corner
  }
  // Where the map's border leaves a side of the tile
  std::unordered_map<std::uint64_t, int> edges;
  auto key = [](PointId a, PointId b) {
      return a < b ? (static_cast<std::uint64_t>(a) << 32) | b :
             (static_cast<std::uint64_t>(b) << 32) | a;
    };
  for (const auto & c : m.navcels) {
    for (int k = 0; k < 3; ++k) {
      edges[key(c.v[k], c.v[(k + 1) % 3])]++;
    }
  }
  for (const auto & [e, count] : edges) {
    if (count != 1) {continue;}
    const PointId a = static_cast<PointId>(e >> 32), b = static_cast<PointId>(e);
    const int sa = sides(a), sb = sides(b);
    if (sa && !(sa & sb)) {lock[a] = 1;}
    if (sb && !(sa & sb)) {lock[b] = 1;}
  }
  std::vector<PointId> out;
  for (PointId v = 0; v < lock.size(); ++v) {
    if (lock[v]) {out.push_back(v);}
  }
  return out;
}

void TiledLod::simplify_node(Node & n) const
{
  ::navmap::SimplifyParams sp;
  sp.max_error = static_cast<float>(cell(n.level) / 2.0);
  sp.max_edge_length = static_cast<float>(p_.cell_max);
  sp.keep_vertices = border_locks(n);
  ::navmap::simplify(n.mesh, sp);
  n.mesh = slim(std::move(n.mesh));
}

double TiledLod::distance(const Node & n, double x, double y) const
{
  const double dx = std::max({static_cast<double>(xs_[n.lines[0]]) - x, 0.0,
        x - static_cast<double>(xs_[n.lines[1]])});
  const double dy = std::max({static_cast<double>(ys_[n.lines[2]]) - y, 0.0,
        y - static_cast<double>(ys_[n.lines[3]])});
  return std::hypot(dx, dy);
}

std::vector<std::uint32_t> TiledLod::select(double x, double y) const
{
  std::vector<std::uint32_t> out, stack(roots_.rbegin(), roots_.rend());
  while (!stack.empty()) {
    const std::uint32_t i = stack.back();
    stack.pop_back();
    const auto & n = nodes_[i];
    if (n.level == 0 || distance(n, x, y) >= radius(n.level)) {
      out.push_back(i);
      continue;
    }
    for (int q = 3; q >= 0; --q) {
      if (n.child[q] >= 0) {stack.push_back(static_cast<std::uint32_t>(n.child[q]));}
    }
  }
  return out;
}

NavMap TiledLod::assemble(const std::vector<std::uint32_t> & tiles, bool accels) const
{
  return stitch(tiles, accels);
}

void TiledLod::detail(
  NavMap & nm, double x, double y, const std::vector<Eigen::Vector3f> & obstacles,
  double margin) const
{
  if (nm.navcels.empty()) {return;}
  const float fine = static_cast<float>(p_.cell_min);
  auto near_robot = [&](const Eigen::Vector2f & q, double extra) {
      return std::hypot(q.x() - x, q.y() - y) <= p_.min_radius + extra;
    };

  // Seeds near the robot: the borders of the map (points every cell_min along them) and the
  // sensed obstacles. Farther obstacles keep the cell size of their place.
  std::unordered_map<std::uint64_t, int> edges;
  auto key = [](PointId a, PointId b) {
      return a < b ? (static_cast<std::uint64_t>(a) << 32) | b :
             (static_cast<std::uint64_t>(b) << 32) | a;
    };
  const float cell_max = static_cast<float>(p_.cell_max);
  for (const auto & c : nm.navcels) {
    // only near the robot: a NavCel left out has all its vertices farther than min_radius plus
    // cell_max (no side is longer), so no edge it shares within min_radius looks open
    if (!near_robot(nm.positions.at(c.v[0]).head<2>(), cell_max) &&
      !near_robot(nm.positions.at(c.v[1]).head<2>(), cell_max) &&
      !near_robot(nm.positions.at(c.v[2]).head<2>(), cell_max))
    {
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      edges[key(c.v[k], c.v[(k + 1) % 3])]++;
    }
  }
  std::vector<Eigen::Vector2f> seeds;
  for (const auto & [e, use] : edges) {
    if (use != 1) {continue;}
    const Eigen::Vector2f a = nm.positions.at(static_cast<PointId>(e >> 32)).head<2>();
    const Eigen::Vector2f b = nm.positions.at(static_cast<PointId>(e & 0xffffffffu)).head<2>();
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / fine)));
    for (int i = 0; i <= n; ++i) {
      const Eigen::Vector2f q = a + (b - a) * (static_cast<float>(i) / n);
      if (near_robot(q, 0.0)) {seeds.push_back(q);}
    }
  }
  struct Far
  {
    Eigen::Vector2f p;
    float size;
  };
  std::vector<Far> far;
  for (const auto & q : obstacles) {
    if (near_robot(q.head<2>(), 0.0)) {
      seeds.push_back(q.head<2>());
    } else {
      far.push_back(
        {q.head<2>(), static_cast<float>(cell(level_at(std::hypot(q.x() - x, q.y() - y))))});
    }
  }

  // Every NavCel within the margin (the inflation) of a seed fine: the cost of a NavCel is that
  // of its centroid, so a large one next to an obstacle would read as free. A NavCel holding an
  // obstacle farther away, as fine as the cell size of its place. One refinement for all.
  const double band = std::max(margin, p_.cell_min);
  auto grid_key = [](std::int64_t i, std::int64_t j) {return (i << 32) ^ (j & 0xffffffff);};
  auto cell_of = [](float v, double cell) {return static_cast<std::int64_t>(std::floor(v / cell));};
  std::unordered_map<std::int64_t, std::vector<Eigen::Vector2f>> seed_grid;
  for (const auto & q : seeds) {
    seed_grid[grid_key(cell_of(q.x(), band), cell_of(q.y(), band))].push_back(q);
  }
  const double far_cell = p_.cell_max;
  std::unordered_map<std::int64_t, std::vector<Far>> far_grid;
  for (const auto & f : far) {
    far_grid[grid_key(cell_of(f.p.x(), far_cell), cell_of(f.p.y(), far_cell))].push_back(f);
  }
  auto cross = [](const Eigen::Vector2f & u, const Eigen::Vector2f & v) {
      return u.x() * v.y() - u.y() * v.x();
    };
  auto size_at = [&](
    const Eigen::Vector3f & a3, const Eigen::Vector3f & b3, const Eigen::Vector3f & c3) {
      const Eigen::Vector2f a = a3.head<2>(), b = b3.head<2>(), c = c3.head<2>();
      const Eigen::Vector2f q = (a + b + c) / 3.0f;
      const float reach = std::max({(a - q).norm(), (b - q).norm(), (c - q).norm()});
      if (!seed_grid.empty() && near_robot(q, band + reach)) {
        const double r = band + reach;
        const int span = static_cast<int>(std::ceil(r / band));
        const auto ci = cell_of(q.x(), band), cj = cell_of(q.y(), band);
        for (std::int64_t i = ci - span; i <= ci + span; ++i) {
          for (std::int64_t j = cj - span; j <= cj + span; ++j) {
            const auto it = seed_grid.find(grid_key(i, j));
            if (it == seed_grid.end()) {continue;}
            for (const auto & sd : it->second) {
              if ((sd - q).norm() < r) {return fine;}
            }
          }
        }
      }
      float out = std::numeric_limits<float>::infinity();
      if (far_grid.empty()) {return out;}
      const Eigen::Vector2f lo = a.cwiseMin(b).cwiseMin(c), hi = a.cwiseMax(b).cwiseMax(c);
      const float area = cross(b - a, c - a);
      const float eps = 1e-6f * std::abs(area) + 1e-9f;
      for (auto i = cell_of(lo.x(), far_cell); i <= cell_of(hi.x(), far_cell); ++i) {
        for (auto j = cell_of(lo.y(), far_cell); j <= cell_of(hi.y(), far_cell); ++j) {
          const auto it = far_grid.find(grid_key(i, j));
          if (it == far_grid.end()) {continue;}
          for (const auto & f : it->second) {
            const float s1 = cross(b - a, f.p - a), s2 = cross(c - b, f.p - b),
              s3 = cross(a - c, f.p - c);
            const bool in = area > 0.0f ? (s1 >= -eps && s2 >= -eps && s3 >= -eps) :
              (s1 <= eps && s2 <= eps && s3 <= eps);
            if (in) {out = std::min(out, f.size);}
          }
        }
      }
      return out;
    };
  const auto st = ::navmap::refine(nm, size_at);
  if (st.navcels_after == st.navcels_before) {nm.rebuild_geometry_accels();}  // refine did not

  auto layer = nm.add_layer<std::uint8_t>(
    "obstacles", "obstacles: the map's and the sensed ones", "", ::navmap::FREE_SPACE);
  for (const auto & q : obstacles) {
    std::size_t s;
    NavCelId c;
    Eigen::Vector3f bary;
    if (nm.locate_navcel(q, s, c, bary, nullptr)) {
      layer->data()[c] = ::navmap::LETHAL_OBSTACLE;
      layer->mark_changed(c);
    }
  }
}

NavMap TiledLod::stitch(const std::vector<std::uint32_t> & nodes, bool accels) const
{
  // Vertices: those on a grid line are shared by the tiles on both sides (same exact position)
  std::vector<Eigen::Vector3f> P;
  std::vector<std::array<std::uint8_t, 4>> C;
  bool colored = !nodes.empty();
  std::size_t surfaces = 0;
  for (const auto i : nodes) {
    const auto & m = nodes_[i].mesh;
    colored &= m.colors && m.colors->r.size() == m.positions.size();
    surfaces = std::max(surfaces, m.surfaces.size());
  }
  std::unordered_map<Key, std::uint32_t, KeyHash> shared;
  struct OnLine
  {
    float s;              // coordinate along the line
    std::uint32_t v;
  };
  std::vector<std::vector<OnLine>> vlines(xs_.size()), hlines(ys_.size());
  struct Src
  {
    std::uint32_t node;
    NavCelId cel;
  };
  std::vector<std::array<std::uint32_t, 3>> tris;
  std::vector<Src> src;
  std::vector<std::vector<std::size_t>> surf_of(nodes.size());
  for (std::size_t ni = 0; ni < nodes.size(); ++ni) {
    const auto & m = nodes_[nodes[ni]].mesh;
    surf_of[ni] = surface_of(m);
    std::vector<std::uint32_t> g(m.positions.size());
    for (PointId v = 0; v < m.positions.size(); ++v) {
      const Eigen::Vector3f p = m.positions.at(v);
      const int ix = line_index(xs_, p.x()), iy = line_index(ys_, p.y());
      auto add = [&]() {
          P.push_back(p);
          if (colored) {
            C.push_back({m.colors->r[v], m.colors->g[v], m.colors->b[v], m.colors->a[v]});
          }
          return static_cast<std::uint32_t>(P.size() - 1);
        };
      if (ix < 0 && iy < 0) {g[v] = add(); continue;}
      auto [it, fresh] = shared.try_emplace(Key{p.x(), p.y(), p.z()}, 0u);
      if (fresh) {
        it->second = add();
        if (ix >= 0) {vlines[ix].push_back({p.y(), it->second});}
        if (iy >= 0) {hlines[iy].push_back({p.x(), it->second});}
      }
      g[v] = it->second;
    }
    for (NavCelId c = 0; c < m.navcels.size(); ++c) {
      const auto & v = m.navcels[c].v;
      tris.push_back({g[v[0]], g[v[1]], g[v[2]]});
      src.push_back({static_cast<std::uint32_t>(ni), c});
    }
  }
  for (auto * lines : {&vlines, &hlines}) {
    for (auto & l : *lines) {
      std::sort(l.begin(), l.end(), [](const OnLine & a, const OnLine & b) {return a.s < b.s;});
    }
  }

  // Stitching: an edge on a grid line gets the vertices that the tiles on the other side of the
  // line have on it, between its ends. All the vertices are the source's own (a simplified side
  // only loses vertices along it), and along a line the ground of the source is made of chains of
  // edges whose ends (where a border crosses the line) and tile corners are kept at every level.
  // Floors over each other have edges on the same stretch of a line: a vertex goes into an edge
  // of another chain (a chain whose ends were simplified away on one side) only if it is within
  // kSameFloor of it in height, so floors, or a floor that ends at the line above another, are
  // never joined; on each side it goes into the covering edge nearest in height.
  constexpr float kSameFloor = 0.5f;   // m: floors are farther apart than this
  struct LineEdge
  {
    std::uint32_t a, b;   // ends
    float sa, sb;         // their coordinates along the line
    const std::vector<OnLine> * line;
    std::uint64_t lid;    // the line: vertical ones first, then horizontal
    int side;             // 0 / 1: the side of the line its NavCel is on
  };
  std::vector<LineEdge> ledges;
  std::vector<std::array<int, 3>> tri_edge(tris.size(), {-1, -1, -1});
  for (std::size_t t = 0; t < tris.size(); ++t) {
    const auto & g = tris[t];
    for (int k = 0; k < 3; ++k) {
      const std::uint32_t a = g[k], b = g[(k + 1) % 3];
      const Eigen::Vector3f pa = P[a], pb = P[b], po = P[g[(k + 2) % 3]];
      LineEdge e{a, b, 0.0f, 0.0f, nullptr, 0, 0};
      if (pa.x() == pb.x()) {
        const int ix = line_index(xs_, pa.x());
        if (ix >= 0) {
          e = {a, b, pa.y(), pb.y(), &vlines[ix], static_cast<std::uint64_t>(ix),
            po.x() > pa.x() ? 1 : 0};
        }
      }
      if (!e.line && pa.y() == pb.y()) {
        const int iy = line_index(ys_, pa.y());
        if (iy >= 0) {
          e = {a, b, pa.x(), pb.x(), &hlines[iy], xs_.size() + static_cast<std::uint64_t>(iy),
            po.y() > pa.y() ? 1 : 0};
        }
      }
      if (!e.line || e.sa == e.sb) {continue;}
      tri_edge[t][k] = static_cast<int>(ledges.size());
      ledges.push_back(e);
    }
  }
  // The chains: the edges on each line, joined at their shared vertices (a vertex at a crossing
  // of two lines is on two: keyed by the line too)
  std::unordered_map<std::uint64_t, std::uint32_t> on_line;   // (line, vertex) -> index
  auto line_vertex = [](std::uint64_t lid, std::uint32_t v) {
      return (lid << 32) | static_cast<std::uint64_t>(v);
    };
  std::vector<std::uint32_t> chain;
  auto index_of = [&](std::uint64_t lid, std::uint32_t v) {
      auto [it, fresh] = on_line.try_emplace(line_vertex(lid, v), 0u);
      if (fresh) {
        it->second = static_cast<std::uint32_t>(chain.size());
        chain.push_back(it->second);
      }
      return it->second;
    };
  std::function<std::uint32_t(std::uint32_t)> find = [&](std::uint32_t x) {
      while (chain[x] != x) {
        chain[x] = chain[chain[x]];
        x = chain[x];
      }
      return x;
    };
  for (const auto & e : ledges) {
    const auto ia = index_of(e.lid, e.a), ib = index_of(e.lid, e.b);
    chain[find(ia)] = find(ib);
  }
  auto chain_of = [&](std::uint64_t lid, std::uint32_t v) -> std::int64_t {
      const auto it = on_line.find(line_vertex(lid, v));
      return it == on_line.end() ? -1 : static_cast<std::int64_t>(find(it->second));
    };
  // For each vertex and side of a line, the edge of its chain on that side that covers it
  struct Best
  {
    float dz;
    int edge;   // -1: an edge on that side ends at the vertex, nothing to do
  };
  std::unordered_map<std::uint64_t, Best> best;
  auto best_key = [](const LineEdge & e, std::uint32_t v) {
      return (e.lid << 33) | (static_cast<std::uint64_t>(v) << 1) |
             static_cast<std::uint64_t>(e.side);
    };
  auto offer = [&](std::uint64_t k, float dz, int edge) {
      auto [it, fresh] = best.try_emplace(k, Best{dz, edge});
      if (!fresh && dz < it->second.dz) {it->second = Best{dz, edge};}
    };
  for (std::size_t i = 0; i < ledges.size(); ++i) {
    const auto & e = ledges[i];
    offer(best_key(e, e.a), 0.0f, -1);
    offer(best_key(e, e.b), 0.0f, -1);
    const std::int64_t own = chain_of(e.lid, e.a);
    const float lo = std::min(e.sa, e.sb), hi = std::max(e.sa, e.sb);
    auto it = std::upper_bound(
      e.line->begin(), e.line->end(), lo, [](float s, const OnLine & o) {return s < o.s;});
    for (; it != e.line->end() && it->s < hi; ++it) {
      const float t = (it->s - e.sa) / (e.sb - e.sa);
      const float dz = std::abs(P[it->v].z() - (P[e.a].z() + t * (P[e.b].z() - P[e.a].z())));
      if (dz > kSameFloor && chain_of(e.lid, it->v) != own) {continue;}
      offer(best_key(e, it->v), dz, static_cast<int>(i));
    }
  }
  std::vector<std::vector<std::uint32_t>> inserted(ledges.size());
  for (std::size_t i = 0; i < ledges.size(); ++i) {
    const auto & e = ledges[i];
    const float lo = std::min(e.sa, e.sb), hi = std::max(e.sa, e.sb);
    auto it = std::upper_bound(
      e.line->begin(), e.line->end(), lo, [](float s, const OnLine & o) {return s < o.s;});
    for (; it != e.line->end() && it->s < hi; ++it) {
      const auto b = best.find(best_key(e, it->v));
      if (b != best.end() && b->second.edge == static_cast<int>(i)) {inserted[i].push_back(it->v);}
    }
    if (e.sa > e.sb) {std::reverse(inserted[i].begin(), inserted[i].end());}   // from a to b
  }
  auto between = [&](std::size_t t, int k, std::vector<std::uint32_t> & out) {
      out.clear();
      if (tri_edge[t][k] >= 0) {out = inserted[tri_edge[t][k]];}
    };

  std::vector<std::array<std::uint32_t, 3>> out_tris;
  std::vector<Src> out_src;
  out_tris.reserve(tris.size());
  out_src.reserve(tris.size());
  std::array<std::vector<std::uint32_t>, 3> ins;
  for (std::size_t t = 0; t < tris.size(); ++t) {
    const auto & g = tris[t];
    int edges_with = 0, which = -1;
    for (int k = 0; k < 3; ++k) {
      between(t, k, ins[k]);
      if (!ins[k].empty()) {++edges_with; which = k;}
    }
    if (edges_with == 0) {
      out_tris.push_back(g);
      out_src.push_back(src[t]);
    } else if (edges_with == 1) {
      // a fan from the opposite vertex, along the edge (same winding)
      const std::uint32_t a = g[which], b = g[(which + 1) % 3], o = g[(which + 2) % 3];
      std::uint32_t prev = a;
      for (const auto v : ins[which]) {
        out_tris.push_back({prev, v, o});
        out_src.push_back(src[t]);
        prev = v;
      }
      out_tris.push_back({prev, b, o});
      out_src.push_back(src[t]);
    } else {
      // vertices on more than one edge: a fan from the centroid
      std::vector<std::uint32_t> poly;
      for (int k = 0; k < 3; ++k) {
        poly.push_back(g[k]);
        poly.insert(poly.end(), ins[k].begin(), ins[k].end());
      }
      P.push_back((P[g[0]] + P[g[1]] + P[g[2]]) / 3.0f);
      if (colored) {
        std::array<std::uint8_t, 4> c;
        for (int ch = 0; ch < 4; ++ch) {
          c[ch] = static_cast<std::uint8_t>((C[g[0]][ch] + C[g[1]][ch] + C[g[2]][ch]) / 3);
        }
        C.push_back(c);
      }
      const std::uint32_t mid = static_cast<std::uint32_t>(P.size() - 1);
      for (std::size_t i = 0; i < poly.size(); ++i) {
        out_tris.push_back({poly[i], poly[(i + 1) % poly.size()], mid});
        out_src.push_back(src[t]);
      }
    }
  }

  // The NavMap: geometry, surfaces, layers, colours
  NavMap out;
  for (std::size_t s = 0; s < surfaces; ++s) {
    std::string frame;
    for (const auto i : nodes) {
      if (s < nodes_[i].mesh.surfaces.size()) {frame = nodes_[i].mesh.surfaces[s].frame_id; break;}
    }
    out.create_surface(frame);
  }
  out.positions.x.reserve(P.size());
  out.positions.y.reserve(P.size());
  out.positions.z.reserve(P.size());
  for (const auto & p : P) {
    out.positions.x.push_back(p.x());
    out.positions.y.push_back(p.y());
    out.positions.z.push_back(p.z());
  }
  if (colored) {
    out.colors.emplace();
    for (const auto & c : C) {
      out.colors->r.push_back(c[0]);
      out.colors->g.push_back(c[1]);
      out.colors->b.push_back(c[2]);
      out.colors->a.push_back(c[3]);
    }
  }
  out.navcels.resize(out_tris.size());
  for (std::size_t i = 0; i < out_tris.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      out.navcels[i].v[k] = out_tris[i][k];
    }
    const auto & s = out_src[i];
    out.surfaces[surf_of[s.node][s.cel]].navcels.push_back(static_cast<NavCelId>(i));
  }
  std::map<std::string, std::uint32_t> layer_owner;   // a tile that has each layer
  for (std::size_t ni = 0; ni < nodes.size(); ++ni) {
    for (const auto & name : nodes_[nodes[ni]].mesh.list_layers()) {
      layer_owner.emplace(name, static_cast<std::uint32_t>(ni));
    }
  }
  for (const auto & [name, owner] : layer_owner) {
    with_view(
      nodes_[nodes[owner]].mesh.layers.get(name), [&](const auto & first) {
        using T = std::decay_t<decltype(first.data()[0])>;
        auto dst = out.add_layer<T>(name);
        std::vector<const std::vector<T> *> per_node(nodes.size(), nullptr);
        for (std::size_t ni = 0; ni < nodes.size(); ++ni) {
          const auto & m = nodes_[nodes[ni]].mesh;
          auto v = std::dynamic_pointer_cast<::navmap::LayerView<T>>(m.layers.get(name));
          if (v && v->size() == m.navcels.size()) {per_node[ni] = &v->data();}
        }
        for (std::size_t i = 0; i < out_src.size(); ++i) {
          const auto * d = per_node[out_src[i].node];
          if (d) {dst->data()[i] = (*d)[out_src[i].cel];}
        }
        const auto meta = nodes_[nodes[owner]].mesh.layer_meta.find(name);
        if (meta != nodes_[nodes[owner]].mesh.layer_meta.end()) {
          out.layer_meta[name] = meta->second;
        }
      });
  }
  if (accels) {out.rebuild_geometry_accels();}
  return out;
}

}  // namespace bigmap
}  // namespace easynav
