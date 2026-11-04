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

#include "easynav_bigmap_maps_manager/surface_lod.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace bigmap_maps_manager
{

namespace
{
struct WeldKey
{
  std::int32_t a, b, c;
  bool operator==(const WeldKey & o) const {return a == o.a && b == o.b && c == o.c;}
};
struct WeldHash
{
  std::size_t operator()(const WeldKey & k) const
  {
    std::size_t h = static_cast<std::size_t>(k.a) * 73856093u;
    h ^= static_cast<std::size_t>(k.b) * 19349663u;
    h ^= static_cast<std::size_t>(k.c) * 83492791u;
    return h;
  }
};
struct TriHash
{
  std::size_t operator()(const std::array<std::uint32_t, 3> & t) const
  {
    std::size_t h = t[0] * 73856093u;
    h ^= t[1] * 19349663u;
    h ^= t[2] * 83492791u;
    return h;
  }
};

// The cell index of a coordinate. An absurd one (a robot pose of 1e12 m, a vertex of 3e38 m) is
// clamped first: its cast to int64 is undefined and the index arithmetic around it overflows.
std::int64_t cell_index(double v, double cell)
{
  return static_cast<std::int64_t>(std::floor(std::clamp(v / cell, -1e15, 1e15)));
}

// (shifted as unsigned: a negative cell index shifted left is undefined before C++20)
std::int64_t cell_key_of(std::int64_t ix, std::int64_t iy)
{
  return static_cast<std::int64_t>(static_cast<std::uint64_t>(ix) << 32) ^ (iy & 0xffffffffLL);
}

std::int64_t cell_key(double x, double y, double cell)
{
  return cell_key_of(cell_index(x, cell), cell_index(y, cell));
}

std::int64_t key_x(std::int64_t key) {return key >> 32;}
std::int64_t key_y(std::int64_t key)
{
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(key & 0xffffffffLL));
}

// Slopes are kept as hundredths of a degree, rounded up: never gentler than the source's
std::uint16_t slope_code(double degrees)
{
  return static_cast<std::uint16_t>(std::clamp(std::ceil(degrees * 100.0 - 1e-6), 0.0, 65535.0));
}

// An upper bound of the effective distances at which the policy splits a node of size `s` and
// error `err` (negative: it never does). Beyond it the node's decision is the one of a robot
// infinitely far, the far cut's: the `dynamic radius` of the node.
double split_radius_of(double s, double err, const LodParams & p)
{
  if (!(s > p.min_size)) {return -1.0;}
  const double hr = p.high_res_radius, lr = p.low_res_distance;
  // the size cap reaches s here (it is min_size up to high_res_radius, max_size from
  // low_res_distance on, geometric in between)
  double r = std::max(hr, lr);
  if (lr > hr && p.max_size > p.min_size && s <= p.max_size) {
    r = hr + (lr - hr) * std::log(s / p.min_size) / std::log(p.max_size / p.min_size);
  }
  // the tolerance reaches err here
  if (err > p.base_error && p.error_growth > 0.0) {
    r = std::max(r, hr + (err - p.base_error) / p.error_growth);
  }
  return r * (1.0 + 1e-6) + 1e-3;
}

bool finite_params(const LodParams & p)
{
  for (const double v : {p.high_res_radius, p.low_res_distance, p.min_size, p.max_size,
      p.base_error, p.error_growth, p.known_margin, p.known_falloff})
  {
    if (!std::isfinite(v)) {return false;}
  }
  return true;
}
}  // namespace

struct SurfaceLodMesher::Query
{
  const LodParams & p;
  LodParams po;    // the policy for the observed ground
  double rx, ry, rz;
};

// ---------------------------------------------------------------------------------------------
// CellIndex

void SurfaceLodMesher::CellIndex::build(
  double cell_size, const std::vector<float> & x, const std::vector<float> & y)
{
  cell = cell_size;
  const std::size_t n = x.size();
  std::vector<std::pair<std::int64_t, std::uint32_t>> kv(n);
  for (std::size_t i = 0; i < n; ++i) {
    kv[i] = {cell_key(x[i], y[i], cell), static_cast<std::uint32_t>(i)};
  }
  std::sort(kv.begin(), kv.end());
  keys.clear();
  start.clear();
  items.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (i == 0 || kv[i].first != kv[i - 1].first) {
      keys.push_back(kv[i].first);
      start.push_back(static_cast<std::uint32_t>(i));
    }
    items[i] = kv[i].second;
  }
  start.push_back(static_cast<std::uint32_t>(n));
  keys.shrink_to_fit();
  start.shrink_to_fit();
}

template<class F>
void SurfaceLodMesher::CellIndex::visit_cell(std::int64_t key, F && f) const
{
  const auto it = std::lower_bound(keys.begin(), keys.end(), key);
  if (it == keys.end() || *it != key) {return;}
  const auto i = static_cast<std::size_t>(it - keys.begin());
  for (std::uint32_t m = start[i]; m < start[i + 1]; ++m) {
    f(items[m]);
  }
}

template<class F>
void SurfaceLodMesher::CellIndex::visit_disc(double x, double y, double radius, F && f) const
{
  if (keys.empty() || !(radius >= 0.0)) {return;}
  const auto span = static_cast<std::int64_t>(std::ceil(std::min(radius, 1e12) / cell));
  const auto cx = cell_index(x, cell), cy = cell_index(y, cell);
  const double cells = static_cast<double>(2 * span + 1) * static_cast<double>(2 * span + 1);
  if (cells > 4.0 * static_cast<double>(keys.size())) {
    // a disc wider than the map: every occupied cell in the square
    for (std::size_t i = 0; i < keys.size(); ++i) {
      if (std::abs(key_x(keys[i]) - cx) > span || std::abs(key_y(keys[i]) - cy) > span) {continue;}
      for (std::uint32_t m = start[i]; m < start[i + 1]; ++m) {
        f(items[m]);
      }
    }
    return;
  }
  for (std::int64_t j = cy - span; j <= cy + span; ++j) {
    for (std::int64_t i = cx - span; i <= cx + span; ++i) {
      visit_cell(cell_key_of(i, j), f);
    }
  }
}

std::size_t SurfaceLodMesher::CellIndex::memory_bytes() const
{
  return keys.capacity() * sizeof(std::int64_t) +
         (start.capacity() + items.capacity()) * sizeof(std::uint32_t);
}

// ---------------------------------------------------------------------------------------------
// Construction

SurfaceLodMesher::SurfaceLodMesher(const SourceMesh & map, const Config & config)
: cfg_(config)
{
  init(map);
}

SurfaceLodMesher::SurfaceLodMesher(const ::navmap::NavMap & map, const Config & config)
: cfg_(config)
{
  if (map.navcels.empty() || map.positions.x.empty()) {
    throw std::invalid_argument("the NavMap has no NavCels");
  }
  init(SourceMesh::from_navmap(map));
}

void SurfaceLodMesher::init(const SourceMesh & map)
{
  if (map.triangle_count() == 0 || map.x.empty()) {
    throw std::invalid_argument("the NavMap has no NavCels");
  }
  if (!(cfg_.min_size > 0.0) || !(cfg_.max_size >= cfg_.min_size) ||
    !(cfg_.cluster_height > 0.0) || !(cfg_.hole_error >= 0.0) || !std::isfinite(cfg_.max_size) ||
    !std::isfinite(cfg_.hole_error))
  {
    throw std::invalid_argument("invalid surface LOD configuration");
  }
  // (a NaN here would be welded to one vertex, cast to a cell index (undefined) and come out in
  // every mesh; an index out of range read anywhere)
  map.validate();
  const auto & px = map.x;
  const auto & py = map.y;
  const auto & pz = map.z;
  coloured_ = map.coloured();

  // Weld the vertices that share a position (to a millimetre): the clusters need a connected mesh
  Level & l0 = levels_.emplace_back();
  std::vector<std::uint32_t> remap(px.size(), 0);
  {
    std::unordered_map<WeldKey, std::uint32_t, WeldHash> welded;
    welded.reserve(px.size());
    for (std::size_t v = 0; v < px.size(); ++v) {
      const WeldKey k{static_cast<std::int32_t>(std::lround(px[v] * 1000.0)),
        static_cast<std::int32_t>(std::lround(py[v] * 1000.0)),
        static_cast<std::int32_t>(std::lround(pz[v] * 1000.0))};
      const auto it = welded.emplace(k, static_cast<std::uint32_t>(l0.x.size()));
      remap[v] = it.first->second;
      if (!it.second) {continue;}
      l0.x.push_back(px[v]);
      l0.y.push_back(py[v]);
      l0.z.push_back(pz[v]);
      if (coloured_) {
        l0.r.push_back(map.r[v]);
        l0.g.push_back(map.g[v]);
        l0.b.push_back(map.b[v]);
      }
    }
  }
  const std::size_t V = l0.x.size();
  known_.assign(V, 0);

  had_traversed_ = map.has_traversed_layer;
  const std::size_t nt_src = map.triangle_count();
  tri_.reserve(nt_src * 3);
  tri_slope_.reserve(nt_src);
  for (std::size_t t = 0; t < nt_src; ++t) {
    const std::uint32_t a = remap[map.v0[t]], b = remap[map.v1[t]], d = remap[map.v2[t]];
    if (a == b || b == d || a == d) {continue;}
    tri_.insert(tri_.end(), {a, b, d});
    {
      const double ux = l0.x[b] - l0.x[a], uy = l0.y[b] - l0.y[a], uz = l0.z[b] - l0.z[a];
      const double vx = l0.x[d] - l0.x[a], vy = l0.y[d] - l0.y[a], vz = l0.z[d] - l0.z[a];
      const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
      tri_slope_.push_back(slope_code(len < 1e-12 ? 90.0 :
        std::acos(std::min(1.0, std::abs(nz) / len)) * 180.0 / M_PI));
    }
    if (!map.traversed.empty() && map.traversed[t] > 0) {
      for (const auto v : {a, b, d}) {
        known_[v] = 1;
      }
    }
  }
  remap = {};
  if (tri_.empty()) {throw std::invalid_argument("the NavMap has no usable NavCels");}

  if (cfg_.drop_unreachable_ground) {
    // union-find over the triangles that share an edge
    const std::size_t nt = tri_.size() / 3;
    std::vector<std::uint32_t> root(nt);
    for (std::uint32_t t = 0; t < nt; ++t) {
      root[t] = t;
    }
    const auto find = [&root](std::uint32_t t) {
        while (root[t] != t) {root[t] = root[root[t]]; t = root[t];}
        return t;
      };
    {
      std::unordered_map<std::uint64_t, std::uint32_t> first;
      first.reserve(tri_.size());
      for (std::uint32_t t = 0; t < nt; ++t) {
        for (int k = 0; k < 3; ++k) {
          auto a = tri_[3 * t + k], b = tri_[3 * t + (k + 1) % 3];
          if (a > b) {std::swap(a, b);}
          const auto key = (static_cast<std::uint64_t>(a) << 32) | b;
          const auto it = first.emplace(key, t);
          if (!it.second) {root[find(t)] = find(it.first->second);}
        }
      }
    }
    std::unordered_map<std::uint32_t, std::size_t> size;
    for (std::uint32_t t = 0; t < nt; ++t) {
      ++size[find(t)];
    }
    std::uint32_t best = find(0);
    for (const auto & kv : size) {
      if (kv.second > size[best]) {best = kv.first;}
    }
    std::size_t kept = 0;
    for (std::uint32_t t = 0; t < nt; ++t) {
      if (find(t) != best) {continue;}
      for (int k = 0; k < 3; ++k) {
        tri_[3 * kept + k] = tri_[3 * t + k];
      }
      tri_slope_[kept] = tri_slope_[t];
      ++kept;
    }
    n_dropped_ = nt - kept;
    tri_.resize(3 * kept);
    tri_slope_.resize(kept);
    // what was driven over on the dropped pieces does not count
    std::vector<std::uint8_t> used(V, 0);
    for (const auto v : tri_) {
      used[v] = 1;
    }
    for (std::size_t v = 0; v < V; ++v) {
      if (!used[v]) {known_[v] = 0;}
    }
  }
  tri_.shrink_to_fit();
  tri_slope_.shrink_to_fit();
  for (std::uint32_t v = 0; v < V; ++v) {
    if (known_[v]) {known_list_.push_back(v);}
  }
  n_known_ = known_list_.size();
  observed_.assign(V, 0);

  // The source triangles of each vertex (CSR)
  vtx_tri_start_.assign(V + 1, 0);
  for (const auto v : tri_) {
    ++vtx_tri_start_[v + 1];
  }
  for (std::size_t v = 0; v < V; ++v) {
    vtx_tri_start_[v + 1] += vtx_tri_start_[v];
  }
  vtx_tri_.resize(tri_.size());
  {
    std::vector<std::uint32_t> fill(vtx_tri_start_.begin(), vtx_tri_start_.end() - 1);
    for (std::size_t i = 0; i < tri_.size(); ++i) {
      vtx_tri_[fill[tri_[i]]++] = static_cast<std::uint32_t>(i / 3);
    }
  }

  // The border of the mesh: the ends of the edges that belong to one triangle only
  l0.flags.assign(V, 0);
  {
    std::unordered_map<std::uint64_t, int> uses;
    uses.reserve(tri_.size());
    const auto edge_key = [](std::uint32_t a, std::uint32_t b) {
        if (a > b) {std::swap(a, b);}
        return (static_cast<std::uint64_t>(a) << 32) | b;
      };
    for (std::size_t t = 0; t < tri_.size(); t += 3) {
      for (int k = 0; k < 3; ++k) {
        ++uses[edge_key(tri_[t + k], tri_[t + (k + 1) % 3])];
      }
    }
    for (std::size_t t = 0; t < tri_.size(); t += 3) {
      for (int k = 0; k < 3; ++k) {
        const auto a = tri_[t + k], b = tri_[t + (k + 1) % 3];
        if (uses[edge_key(a, b)] == 1) {l0.flags[a] |= kBoundary; l0.flags[b] |= kBoundary;}
      }
    }
  }
  grid_.build(kGridCell, l0.x, l0.y);
  if (cfg_.keep_narrow_width > 0.0) {pin_narrow_passages();}
  // Typical edge of the source: clustering finer than two of them only blurs the mesh (holes, steps
  // across a narrow ramp). The first cluster level is never smaller, and wherever the policy asks
  // for less than that the mesh is the source itself.
  {
    std::vector<float> edge;
    edge.reserve(tri_.size() / 3);
    for (std::size_t t = 0; t < tri_.size(); t += 3) {
      const auto a = tri_[t], b = tri_[t + 1], c = tri_[t + 2];
      const auto len = [&l0](std::uint32_t p, std::uint32_t q) {
          return std::hypot(l0.x[p] - l0.x[q], l0.y[p] - l0.y[q], l0.z[p] - l0.z[q]);
        };
      edge.push_back(static_cast<float>((len(a, b) + len(b, c) + len(c, a)) / 3.0));
    }
    std::nth_element(edge.begin(), edge.begin() + edge.size() / 2, edge.end());
    source_edge_ = edge[edge.size() / 2];
  }
  build_levels();

  offset_.assign(levels_.size() + 1, 0);
  for (std::size_t k = 0; k < levels_.size(); ++k) {
    offset_[k + 1] = offset_[k] + levels_[k].x.size();
  }
  if (offset_.back() >= kNone) {throw std::invalid_argument("the NavMap is too large");}
  node_grid_.resize(levels_.size());
  for (std::size_t k = 1; k < levels_.size(); ++k) {
    node_grid_[k].build(std::max(4.0 * levels_[k].size, kGridCell), levels_[k].x, levels_[k].y);
  }
}

void SurfaceLodMesher::pin_narrow_passages()
{
  // Distance of every vertex to the nearest border vertex (same floor: within a metre in z); the
  // half-width of the passage around a vertex is the largest such distance in its surroundings. A
  // vertex whose surroundings are all closer to a border than half the width is in a narrow one.
  Level & l0 = levels_[0];
  const std::size_t n = l0.x.size();
  const double half = 0.5 * cfg_.keep_narrow_width;
  const double reach = half + 0.5;  // the borders that matter and the surroundings to look at
  std::vector<float> to_border(n, static_cast<float>(reach + 1.0));
  for (std::uint32_t v = 0; v < n; ++v) {
    if (l0.flags[v] & kBoundary) {to_border[v] = 0.0f; continue;}
    grid_.visit_disc(l0.x[v], l0.y[v], reach, [&](std::uint32_t u) {
        if (!(l0.flags[u] & kBoundary) || std::abs(l0.z[u] - l0.z[v]) > 1.0) {return;}
        const double d = std::hypot(l0.x[u] - l0.x[v], l0.y[u] - l0.y[v]);
        if (d < to_border[v]) {to_border[v] = static_cast<float>(d);}
      });
  }
  for (std::uint32_t v = 0; v < n; ++v) {
    float widest = 0.0f;
    const double look = half + 0.25;  // far enough to reach the middle of a wide passage
    grid_.visit_disc(l0.x[v], l0.y[v], look, [&](std::uint32_t u) {
        if (std::abs(l0.z[u] - l0.z[v]) > 1.0 ||
        std::hypot(l0.x[u] - l0.x[v], l0.y[u] - l0.y[v]) > look) {return;}
        widest = std::max(widest, to_border[u]);
      });
    if (widest < half) {
      l0.flags[v] |= kPinned;
      ++n_pinned_;
    }
  }
}

void SurfaceLodMesher::build_levels()
{
  // The edges of the source mesh (each once): two nodes are merged only if the mesh joins them
  // inside the cell. Without it the two sides of a gap, of a wall's footprint or of a ledge
  // falling in one cell become one node, and the coarse mesh bridges what cannot be driven.
  std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
  {
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(tri_.size());
    for (std::size_t t = 0; t < tri_.size(); t += 3) {
      for (int k = 0; k < 3; ++k) {
        auto a = tri_[t + k], b = tri_[t + (k + 1) % 3];
        if (a > b) {std::swap(a, b);}
        if (seen.insert((static_cast<std::uint64_t>(a) << 32) | b).second) {
          edges.emplace_back(a, b);
        }
      }
    }
  }
  // What only the construction needs, per node of the level being grouped
  struct Build
  {
    std::vector<float> zmin, zmax, r, g, b;
    std::vector<std::uint32_t> count;
  };
  Build prev_b;
  {
    const Level & l0 = levels_[0];
    prev_b.zmin = l0.z;
    prev_b.zmax = l0.z;
    prev_b.count.assign(l0.x.size(), 1);
    if (coloured_) {
      prev_b.r.assign(l0.r.begin(), l0.r.end());
      prev_b.g.assign(l0.g.begin(), l0.g.end());
      prev_b.b.assign(l0.b.begin(), l0.b.end());
    }
  }
  // owner[v]: the node of the previous level that holds source vertex v
  std::vector<std::uint32_t> owner(levels_[0].x.size());
  for (std::uint32_t v = 0; v < owner.size(); ++v) {
    owner[v] = v;
  }
  std::vector<std::uint32_t> uf;   // union-find over the nodes of the previous level
  const auto find = [&uf](std::uint32_t a) {
      while (uf[a] != a) {
        uf[a] = uf[uf[a]];
        a = uf[a];
      }
      return a;
    };

  double size = std::max(cfg_.min_size, 2.0 * source_edge_);
  while (true) {
    Level & prev = levels_.back();
    const std::size_t n = prev.x.size();
    Level next;
    Build next_b;
    const auto key_of = [&](std::uint32_t i) {
        // border nodes and inner ones are never mixed
        // (shifted as unsigned: the key is negative for cells west or south of the origin)
        return static_cast<std::int64_t>(
          (static_cast<std::uint64_t>(cell_key(prev.x[i], prev.y[i], size)) << 1) |
          static_cast<std::uint64_t>(prev.flags[i] & kBoundary));
      };
    uf.resize(n);
    for (std::uint32_t i = 0; i < n; ++i) {
      uf[i] = i;
    }
    for (const auto & [a, b] : edges) {
      const auto na = owner[a], nb = owner[b];
      if (na != nb && key_of(na) == key_of(nb)) {uf[find(na)] = find(nb);}
    }
    // The groups of this level. `locked` nodes of the previous level stay alone (see the fold-over
    // check below).
    const auto group = [&](const std::vector<std::uint8_t> & locked) {
        next = Level();
        next_b = Build();
        next.size = size;
        // the nodes of the previous level by cell
        std::unordered_map<std::int64_t, std::vector<std::uint32_t>> cells;
        for (std::uint32_t i = 0; i < n; ++i) {
          cells[key_of(i)].push_back(i);
        }
        prev.parent.assign(n, 0);
        for (auto & kv : cells) {
          auto & ids = kv.second;
          // by connected piece, and inside each piece by height
          std::vector<std::uint32_t> piece(ids.size());
          for (std::size_t m = 0; m < ids.size(); ++m) {
            piece[m] = find(ids[m]);
          }
          std::vector<std::size_t> order(ids.size());
          for (std::size_t m = 0; m < order.size(); ++m) {
            order[m] = m;
          }
          std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
              return piece[a] != piece[b] ? piece[a] < piece[b] : prev.z[ids[a]] < prev.z[ids[b]];
            });
          {
            std::vector<std::uint32_t> sorted_ids(ids.size()), sorted_piece(ids.size());
            for (std::size_t m = 0; m < order.size(); ++m) {
              sorted_ids[m] = ids[order[m]];
              sorted_piece[m] = piece[order[m]];
            }
            ids.swap(sorted_ids);
            piece.swap(sorted_piece);
          }
          std::size_t i = 0;
          while (i < ids.size()) {
            // a group: consecutive heights of one piece that together span at most cluster_height
            float lo = prev_b.zmin[ids[i]], hi = prev_b.zmax[ids[i]];
            std::size_t j = i + 1;
            while (j < ids.size() && piece[j] == piece[i] && !locked[ids[i]] && !locked[ids[j]]) {
              const float nlo = std::min(lo, prev_b.zmin[ids[j]]);
              const float nhi = std::max(hi, prev_b.zmax[ids[j]]);
              if (nhi - nlo > cfg_.cluster_height) {break;}
              lo = nlo;
              hi = nhi;
              ++j;
            }
            const auto id = static_cast<std::uint32_t>(next.x.size());
            double sx = 0, sy = 0, sz = 0, sr = 0, sg = 0, sb = 0, w = 0;
            std::uint8_t pin = 0;
            for (std::size_t m = i; m < j; ++m) {
              const auto c = ids[m];
              const double cw = prev_b.count[c];
              sx += cw * prev.x[c];
              sy += cw * prev.y[c];
              sz += cw * prev.z[c];
              if (coloured_) {
                sr += cw * prev_b.r[c];
                sg += cw * prev_b.g[c];
                sb += cw * prev_b.b[c];
              }
              w += cw;
              pin = pin || (prev.flags[c] & kPinned);
              prev.parent[c] = id;
            }
            next.x.push_back(static_cast<float>(sx / w));
            next.y.push_back(static_cast<float>(sy / w));
            next.z.push_back(static_cast<float>(sz / w));
            next_b.zmin.push_back(lo);
            next_b.zmax.push_back(hi);
            next_b.count.push_back(static_cast<std::uint32_t>(w));
            next.flags.push_back(static_cast<std::uint8_t>(
                (prev.flags[ids[i]] & kBoundary) | (pin ? kPinned : 0)));
            if (coloured_) {
              next_b.r.push_back(static_cast<float>(sr / w));
              next_b.g.push_back(static_cast<float>(sg / w));
              next_b.b.push_back(static_cast<float>(sb / w));
            }
            i = j;
          }
        }
      };
    // A merge must not fold the surface over itself: a source triangle whose three corners end up
    // in three different nodes keeps its orientation (seen from above). Where one flips, its
    // corners' groups are undone and the level is built again (a few rounds at most).
    std::vector<std::uint8_t> locked(n, 0);
    for (int round = 0; round < 4; ++round) {
      group(locked);
      bool flipped = false;
      for (std::size_t t = 0; t < tri_.size(); t += 3) {
        const std::uint32_t a = tri_[t], b = tri_[t + 1], c = tri_[t + 2];
        const auto na = prev.parent[owner[a]], nb = prev.parent[owner[b]],
          nc = prev.parent[owner[c]];
        if (na == nb || nb == nc || na == nc) {continue;}
        const auto & l0 = levels_[0];
        const auto cross = [](double ax, double ay, double bx, double by, double cx, double cy) {
            return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
          };
        const double src = cross(l0.x[a], l0.y[a], l0.x[b], l0.y[b], l0.x[c], l0.y[c]);
        const double now = cross(next.x[na], next.y[na], next.x[nb], next.y[nb], next.x[nc],
            next.y[nc]);
        if (std::abs(src) > 1e-9 && src * now < 0.0) {
          for (const auto v : {a, b, c}) {
            locked[owner[v]] = 1;
          }
          flipped = true;
        }
      }
      if (!flipped) {break;}
    }
    for (auto & o : owner) {
      o = prev.parent[o];
    }
    // the node's error (the height it spans around its mean), its colour, its children
    const std::size_t m = next.x.size();
    next.err.resize(m);
    for (std::size_t i = 0; i < m; ++i) {
      float e = std::max(next_b.zmax[i] - next.z[i], next.z[i] - next_b.zmin[i]);
      if (cfg_.hole_error > 0.0 && (next.flags[i] & kBoundary)) {
        e = std::max(e, static_cast<float>(cfg_.hole_error));
      }
      next.err[i] = e;
      next.max_err = std::max(next.max_err, e);
    }
    if (coloured_) {
      const auto to_u8 = [m](const std::vector<float> & in, std::vector<std::uint8_t> & out) {
          out.resize(m);
          for (std::size_t i = 0; i < m; ++i) {
            out[i] = static_cast<std::uint8_t>(std::clamp(std::lround(in[i]), 0L, 255L));
          }
        };
      to_u8(next_b.r, next.r);
      to_u8(next_b.g, next.g);
      to_u8(next_b.b, next.b);
    }
    next.child_start.assign(m + 1, 0);
    for (std::size_t c = 0; c < n; ++c) {
      ++next.child_start[prev.parent[c] + 1];
    }
    for (std::size_t i = 0; i < m; ++i) {
      next.child_start[i + 1] += next.child_start[i];
    }
    next.child.resize(n);
    {
      std::vector<std::uint32_t> fill(next.child_start.begin(), next.child_start.end() - 1);
      for (std::uint32_t c = 0; c < n; ++c) {
        next.child[fill[prev.parent[c]]++] = c;
      }
    }
    prev_b = std::move(next_b);
    levels_.push_back(std::move(next));
    if (size >= cfg_.max_size || levels_.size() > 12) {break;}
    size *= 2.0;
  }
}

// ---------------------------------------------------------------------------------------------
// Driven-over and observed ground

int SurfaceLodMesher::mark(double x, double y, double z, double radius)
{
  // (a NaN cast to a cell index is undefined; nothing is there anyway)
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !(radius >= 0.0)) {return 0;}
  const auto & l0 = levels_[0];
  int added = 0;
  grid_.visit_disc(x, y, std::min(radius, 1e6), [&](std::uint32_t v) {
      if (known_[v] || std::hypot(l0.x[v] - x, l0.y[v] - y) > radius ||
      std::abs(l0.z[v] - z) > cfg_.vertical_tolerance)
      {
        return;
      }
      known_[v] = 1;
      known_list_.push_back(v);
      ++added;
    });
  if (added > 0) {
    n_known_ = known_list_.size();
    kd_dirty_ = true;
  }
  return added;
}

std::size_t SurfaceLodMesher::forget_outside(double x, double y, double z, double radius)
{
  if (known_list_.empty() || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return 0;
  }
  const auto & l0 = levels_[0];
  std::size_t kept = 0;
  for (const auto v : known_list_) {
    const double dx = l0.x[v] - x, dy = l0.y[v] - y, dz = l0.z[v] - z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > radius) {
      known_[v] = 0;
    } else {
      known_list_[kept++] = v;
    }
  }
  const std::size_t forgotten = known_list_.size() - kept;
  if (forgotten > 0) {
    known_list_.resize(kept);
    n_known_ = kept;
    kd_dirty_ = true;
  }
  return forgotten;
}

int SurfaceLodMesher::mark_observed(double x, double y, double z, double radius)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !(radius >= 0.0)) {return 0;}
  const auto & l0 = levels_[0];
  int added = 0;
  grid_.visit_disc(x, y, std::min(radius, 1e6), [&](std::uint32_t v) {
      if (observed_[v] || !(l0.flags[v] & kBoundary) ||
      std::hypot(l0.x[v] - x, l0.y[v] - y) > radius ||
      std::abs(l0.z[v] - z) > cfg_.vertical_tolerance)
      {
        return;
      }
      observed_[v] = 1;
      observed_list_.push_back(v);
      ++added;
    });
  if (added > 0) {kd_dirty_ = true;}
  return added;
}

std::size_t SurfaceLodMesher::forget_observed_outside(
  double x, double y, double z, double radius)
{
  if (observed_list_.empty() || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return 0;
  }
  const auto & l0 = levels_[0];
  std::size_t kept = 0;
  for (const auto v : observed_list_) {
    const double dx = l0.x[v] - x, dy = l0.y[v] - y, dz = l0.z[v] - z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > radius) {
      observed_[v] = 0;
    } else {
      observed_list_[kept++] = v;
    }
  }
  const std::size_t forgotten = observed_list_.size() - kept;
  if (forgotten > 0) {
    observed_list_.resize(kept);
    kd_dirty_ = true;
  }
  return forgotten;
}

// The driven-over and the observed vertices as balanced 2-D trees over (x, y): the nearest one
// (on the same floor: within vertical_tolerance in z) within `reach` of a node is found in
// O(log n). Scanning every 2 m cell within reach instead (a reach of 50 m: 2600 cells per node)
// took up to a second per re-mesh on a big building.
void SurfaceLodMesher::build_kd() const
{
  if (!kd_dirty_) {return;}
  kd_dirty_ = false;
  const auto & l0 = levels_[0];
  for (auto [list, kd] : {std::pair{&known_list_, &kd_known_},
      std::pair{&observed_list_, &kd_observed_}})
  {
    kd->clear();
    kd->reserve(list->size());
    for (const auto v : *list) {
      kd->push_back({l0.x[v], l0.y[v], l0.z[v], true});
    }
    kd_build(*kd, 0, kd->size());
  }
}

void SurfaceLodMesher::kd_build(std::vector<KdNode> & kd, std::size_t lo, std::size_t hi)
{
  // each node is the median of its range along the wider axis: [lo, mid) left, (mid, hi) right
  if (hi - lo <= 1) {return;}
  float min_x = kd[lo].x, max_x = min_x, min_y = kd[lo].y, max_y = min_y;
  for (std::size_t i = lo + 1; i < hi; ++i) {
    min_x = std::min(min_x, kd[i].x);
    max_x = std::max(max_x, kd[i].x);
    min_y = std::min(min_y, kd[i].y);
    max_y = std::max(max_y, kd[i].y);
  }
  const bool by_x = max_x - min_x >= max_y - min_y;
  const std::size_t mid = lo + (hi - lo) / 2;
  std::nth_element(
    kd.begin() + lo, kd.begin() + mid, kd.begin() + hi,
    [by_x](const KdNode & a, const KdNode & b) {return by_x ? a.x < b.x : a.y < b.y;});
  kd[mid].by_x = by_x;
  kd_build(kd, lo, mid);
  kd_build(kd, mid + 1, hi);
}

void SurfaceLodMesher::kd_nearest(
  const std::vector<KdNode> & kd, std::size_t lo, std::size_t hi, double x, double y, double z,
  double & best) const
{
  if (hi <= lo) {return;}
  const std::size_t mid = lo + (hi - lo) / 2;
  const KdNode & n = kd[mid];
  const double dx = x - n.x, dy = y - n.y, dz = z - n.z;
  if (std::abs(dz) <= cfg_.vertical_tolerance) {best = std::min(best, dx * dx + dy * dy + dz * dz);}
  const double delta = n.by_x ? dx : dy;      // signed distance to the splitting line
  if (delta < 0.0) {
    kd_nearest(kd, lo, mid, x, y, z, best);
    if (delta * delta < best) {kd_nearest(kd, mid + 1, hi, x, y, z, best);}
  } else {
    kd_nearest(kd, mid + 1, hi, x, y, z, best);
    if (delta * delta < best) {kd_nearest(kd, lo, mid, x, y, z, best);}
  }
}

double SurfaceLodMesher::near_distance(
  const std::vector<KdNode> & kd, double x, double y, double z, double reach) const
{
  // (beyond `reach` the driven-over ground changes nothing: infinity)
  double best = reach * reach;
  kd_nearest(kd, 0, kd.size(), x, y, z, best);
  return best < reach * reach ? std::sqrt(best) : std::numeric_limits<double>::infinity();
}

// ---------------------------------------------------------------------------------------------
// The policy

int SurfaceLodMesher::level_of(std::uint32_t global) const
{
  return static_cast<int>(
    std::upper_bound(offset_.begin(), offset_.end(), static_cast<std::size_t>(global)) -
    offset_.begin()) - 1;
}

// May the nodes of level k - 1 collapse into node c of level k, for this robot and ground?
bool SurfaceLodMesher::allowed_now(int k, std::uint32_t c, const Query & q) const
{
  const Level & lv = levels_[k];
  if (lv.flags[c] & kPinned) {return false;}
  const LodParams & p = q.p;
  const double s = lv.size, hyp = s * M_SQRT2;
  const double dx = lv.x[c] - q.rx, dy = lv.y[c] - q.ry;
  const double dz = cfg_.vertical_weight * (lv.z[c] - q.rz);
  const double d_eff = effective_distance(std::sqrt(dx * dx + dy * dy + dz * dz), hyp);
  const double err = lv.err[c];
  bool split = should_split_at(p, err, s, d_eff);
  // (the smallest cells are always allowed: nothing to ask the driven-over ground)
  if (!split && s > p.min_size && (!kd_known_.empty() || !kd_observed_.empty())) {
    double d = d_eff;
    for (const auto & [kd, pp] : {std::pair<const std::vector<KdNode> &, const LodParams &>{
          kd_known_, p}, {kd_observed_, q.po}})
    {
      if (kd.empty()) {continue;}
      // beyond this the ground cannot ask for more than the robot already does
      // (known_effective_distance >= low_res_distance)
      const double reach = kReach * hyp + pp.known_margin +
        (pp.low_res_distance - pp.high_res_radius) / std::max(pp.known_falloff, 1e-6);
      const double dk = near_distance(kd, lv.x[c], lv.y[c], lv.z[c], reach);
      if (std::isfinite(dk)) {d = std::min(d, known_effective_distance(pp, dk, hyp));}
    }
    split = should_split_at(p, err, s, d);
  }
  return !split;
}

bool SurfaceLodMesher::allowed_memo(int k, std::uint32_t c, const Query & q) const
{
  const std::size_t g = offset_[k] + c;
  if (node_stamp_[g] != epoch_) {
    node_stamp_[g] = epoch_;
    node_memo_[g] = allowed_now(k, c, q) ? 1 : 0;
  }
  return node_memo_[g] != 0;
}

// The decision of a robot infinitely far with no driven-over or observed ground. The policy only
// splits less with the distance (tolerance and size cap grow with it), so a node allowed now is
// allowed here: the far cut is never finer than a real one.
bool SurfaceLodMesher::static_allowed(int k, std::uint32_t c) const
{
  const Level & lv = levels_[k];
  if (lv.flags[c] & kPinned) {return false;}
  const double s = lv.size;
  if (!(s > far_min_)) {return true;}
  return !(s > far_max_ || (far_growth_ <= 0.0 && lv.err[c] > far_base_));
}

std::uint32_t SurfaceLodMesher::static_leaf(std::uint32_t v) const
{
  std::uint32_t c = v;
  std::size_t g = v;
  for (std::size_t k = 1; k < levels_.size(); ++k) {
    const std::uint32_t parent = levels_[k - 1].parent[c];
    if (!static_allowed(static_cast<int>(k), parent)) {break;}
    c = parent;
    g = offset_[k] + c;
  }
  return static_cast<std::uint32_t>(g);
}

std::uint32_t SurfaceLodMesher::climb(std::uint32_t v, const Query & q) const
{
  std::uint32_t c = v;
  std::size_t g = v;
  for (std::size_t k = 1; k < levels_.size(); ++k) {
    const std::uint32_t parent = levels_[k - 1].parent[c];
    if (!allowed_memo(static_cast<int>(k), parent, q)) {break;}
    c = parent;
    g = offset_[k] + c;
  }
  return static_cast<std::uint32_t>(g);
}

void SurfaceLodMesher::next_epoch() const
{
  const std::size_t nodes = offset_.back(), V = levels_[0].x.size();
  const std::size_t nt = tri_.size() / 3, nf = far_slope_.size();
  if (node_stamp_.size() != nodes) {
    node_stamp_.assign(nodes, 0);
    node_memo_.assign(nodes, 0);
    walk_stamp_.assign(nodes, 0);
    slot_stamp_.assign(nodes, 0);
    slot_.assign(nodes, 0);
    known_stamp_.assign(nodes, 0);
    vtx_stamp_.assign(V, 0);
    cur_node_.assign(V, 0);
    src_stamp_.assign(nt, 0);
  }
  if (far_stamp_.size() != nf) {far_stamp_.assign(nf, 0);}
  if (++epoch_ == 0) {
    for (auto * a : {&node_stamp_, &walk_stamp_, &slot_stamp_, &known_stamp_, &vtx_stamp_,
        &far_stamp_, &src_stamp_})
    {
      std::fill(a->begin(), a->end(), 0u);
    }
    epoch_ = 1;
  }
}

bool SurfaceLodMesher::far_cut_matches(const LodParams & p) const
{
  return far_built_ && far_min_ == p.min_size && far_max_ == p.max_size &&
         far_base_ == p.base_error && far_growth_ == p.error_growth;
}

void SurfaceLodMesher::build_far_cut(const LodParams & p) const
{
  far_built_ = true;
  far_min_ = p.min_size;
  far_max_ = p.max_size;
  far_base_ = p.base_error;
  far_growth_ = p.error_growth;
  const std::size_t V = levels_[0].x.size(), nt = tri_.size() / 3;
  std::vector<std::uint32_t> leaf(V);
  for (std::uint32_t v = 0; v < V; ++v) {
    leaf[v] = static_leaf(v);
  }
  far_tri_.clear();
  far_slope_.clear();
  far_size_.clear();
  far_of_src_.assign(nt, kNone);
  std::unordered_map<std::array<std::uint32_t, 3>, std::uint32_t, TriHash> seen;
  for (std::size_t t = 0; t < nt; ++t) {
    const std::array<std::uint32_t, 3> g{leaf[tri_[3 * t]], leaf[tri_[3 * t + 1]],
      leaf[tri_[3 * t + 2]]};
    if (g[0] == g[1] || g[1] == g[2] || g[0] == g[2]) {continue;}
    auto key = g;
    std::sort(key.begin(), key.end());
    const auto found = seen.emplace(key, static_cast<std::uint32_t>(far_slope_.size()));
    const std::uint32_t f = found.first->second;
    far_of_src_[t] = f;
    if (!found.second) {
      far_slope_[f] = std::max(far_slope_[f], tri_slope_[t]);
      continue;
    }
    far_tri_.insert(far_tri_.end(), g.begin(), g.end());
    far_slope_.push_back(tri_slope_[t]);
    double size = 0.0;
    for (const auto n : g) {
      size = std::max(size, levels_[level_of(n)].size);
    }
    far_size_.push_back(static_cast<float>(size));
  }
  const std::size_t nf = far_slope_.size();
  far_src_start_.assign(nf + 1, 0);
  for (std::size_t t = 0; t < nt; ++t) {
    if (far_of_src_[t] != kNone) {++far_src_start_[far_of_src_[t] + 1];}
  }
  for (std::size_t f = 0; f < nf; ++f) {
    far_src_start_[f + 1] += far_src_start_[f];
  }
  far_src_.resize(far_src_start_[nf]);
  std::vector<std::uint32_t> fill(far_src_start_.begin(), far_src_start_.end() - 1);
  for (std::size_t t = 0; t < nt; ++t) {
    if (far_of_src_[t] != kNone) {far_src_[fill[far_of_src_[t]]++] = static_cast<std::uint32_t>(t);}
  }
  far_tri_.shrink_to_fit();
  far_slope_.shrink_to_fit();
  far_size_.shrink_to_fit();
}

// ---------------------------------------------------------------------------------------------
// Extraction

LodMesh SurfaceLodMesher::extract_reference(
  double rx, double ry, double rz, const LodParams & p) const
{
  const std::size_t V = levels_[0].x.size(), nt = tri_.size() / 3;
  build_kd();
  const Query q{p, observed_params(p), rx, ry, rz};
  next_epoch();
  const bool use_known = n_known_ > 0;

  // the node every vertex ends in: a cut of the tree
  std::vector<std::uint32_t> leaf(V);
  for (std::uint32_t v = 0; v < V; ++v) {
    leaf[v] = climb(v, q);
  }
  if (use_known) {
    for (const auto v : known_list_) {
      known_stamp_[leaf[v]] = epoch_;
    }
  }
  LodMesh out;
  const auto push_node = [&](std::uint32_t g) {
      const int k = level_of(g);
      const std::uint32_t c = g - static_cast<std::uint32_t>(offset_[k]);
      const Level & lv = levels_[k];
      out.x.push_back(lv.x[c]);
      out.y.push_back(lv.y[c]);
      out.z.push_back(lv.z[c]);
      if (coloured_) {
        out.r.push_back(lv.r[c]);
        out.g.push_back(lv.g[c]);
        out.b.push_back(lv.b[c]);
      }
    };
  std::unordered_map<std::array<std::uint32_t, 3>, std::uint32_t, TriHash> seen;
  for (std::size_t t = 0; t < nt; ++t) {
    const std::array<std::uint32_t, 3> g{leaf[tri_[3 * t]], leaf[tri_[3 * t + 1]],
      leaf[tri_[3 * t + 2]]};
    if (g[0] == g[1] || g[1] == g[2] || g[0] == g[2]) {continue;}
    auto key = g;
    std::sort(key.begin(), key.end());
    const float slope = tri_slope_[t] / 100.0f;
    const auto found = seen.emplace(key, static_cast<std::uint32_t>(out.leg.size()));
    if (!found.second) {
      // the same triangle again (several source triangles end in it): the steepest one counts
      out.slope[found.first->second] = std::max(out.slope[found.first->second], slope);
      continue;
    }
    double leg = p.min_size;
    for (const auto n : g) {
      if (slot_stamp_[n] != epoch_) {
        slot_stamp_[n] = epoch_;
        slot_[n] = static_cast<std::uint32_t>(out.x.size());
        push_node(n);
      }
      out.tri.push_back(slot_[n]);
      leg = std::max(leg, levels_[level_of(n)].size);
    }
    out.leg.push_back(static_cast<float>(leg));
    out.slope.push_back(slope);
    out.traversed.push_back(use_known && known_stamp_[g[0]] == epoch_ &&
      known_stamp_[g[1]] == epoch_ && known_stamp_[g[2]] == epoch_);
  }
  return out;
}

// The dynamic radius: a node (k, c) splits only where the effective distance to the robot, or to
// the driven-over / observed ground, is below split_radius_of(size, err). Outside those discs its
// decision is the far cut's. So only the vertices under the nodes inside them climb again, and
// only the source triangles of those vertices are mapped again; the rest of the mesh is the far
// cut's, as precomputed. The result is the reference's (see extract_reference and the tests).
LodMesh SurfaceLodMesher::extract(double rx, double ry, double rz, const LodParams & p) const
{
  if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz) || !finite_params(p) ||
    p.error_growth < 0.0)
  {
    // (a policy that is not monotone in the distance has no far cut)
    return extract_reference(rx, ry, rz, p);
  }
  if (!far_cut_matches(p)) {build_far_cut(p);}
  build_kd();
  const Query q{p, observed_params(p), rx, ry, rz};
  next_epoch();
  const int L = static_cast<int>(levels_.size());
  const bool use_known = n_known_ > 0;

  // 1. The nodes that may split now (near the robot, near the driven-over / observed ground)
  // and do, among those the far cut lets merge
  std::vector<std::uint32_t> split;
  const auto consider = [&](int k, std::uint32_t c) {
      if (node_stamp_[offset_[k] + c] == epoch_) {return;}   // already decided
      if (static_allowed(k, c) && !allowed_memo(k, c, q)) {
        split.push_back(static_cast<std::uint32_t>(offset_[k] + c));
      }
    };
  std::vector<std::int64_t> near_cells, visit;
  for (int k = 1; k < L; ++k) {
    const Level & lv = levels_[k];
    const double s = lv.size, hyp = s * M_SQRT2;
    const double r = split_radius_of(s, lv.max_err, p);
    if (r < 0.0) {continue;}
    const CellIndex & cells = node_grid_[k];
    // robot: d_eff < r  =>  3-D distance < r + kReach * hyp  =>  same in xy
    const double rr = r + kReach * hyp;
    cells.visit_disc(rx, ry, rr, [&](std::uint32_t c) {
        if (std::hypot(lv.x[c] - rx, lv.y[c] - ry) <= rr) {consider(k, c);}
      });
    for (const auto & [kd, pp] : {std::pair<const std::vector<KdNode> &, const LodParams &>{
          kd_known_, p}, {kd_observed_, q.po}})
    {
      if (kd.empty()) {continue;}
      // the ground: known_effective_distance(dk) < r, and dk within the reach the policy looks at
      const double reach = kReach * hyp + pp.known_margin +
        (pp.low_res_distance - pp.high_res_radius) / std::max(pp.known_falloff, 1e-6);
      double kr = reach;
      if (pp.known_falloff > 0.0) {
        kr = std::min(reach, pp.known_margin + kReach * hyp +
            std::max(0.0, r - pp.high_res_radius) / pp.known_falloff);
      }
      kr += 1e-3;
      near_cells.clear();
      for (const auto & n : kd) {
        near_cells.push_back(cell_key(n.x, n.y, cells.cell));
      }
      std::sort(near_cells.begin(), near_cells.end());
      near_cells.erase(std::unique(near_cells.begin(), near_cells.end()), near_cells.end());
      const auto span = static_cast<std::int64_t>(std::ceil(std::min(kr, 1e12) / cells.cell));
      const double per = static_cast<double>(2 * span + 1) * static_cast<double>(2 * span + 1);
      if (per * static_cast<double>(near_cells.size()) > static_cast<double>(lv.x.size())) {
        // (cheaper to look at every node of the level)
        for (std::uint32_t c = 0; c < lv.x.size(); ++c) {
          consider(k, c);
        }
        continue;
      }
      visit.clear();
      for (const auto key : near_cells) {
        const auto ix = key_x(key), iy = key_y(key);
        for (std::int64_t j = iy - span; j <= iy + span; ++j) {
          for (std::int64_t i = ix - span; i <= ix + span; ++i) {
            visit.push_back(cell_key_of(i, j));
          }
        }
      }
      std::sort(visit.begin(), visit.end());
      visit.erase(std::unique(visit.begin(), visit.end()), visit.end());
      for (const auto key : visit) {
        cells.visit_cell(key, [&](std::uint32_t c) {consider(k, c);});
      }
    }
  }

  // 2. The vertices under them (through the nodes the far cut merges: the others stop below
  // anyway), each subtree once, from the top
  std::sort(split.begin(), split.end(), std::greater<>());
  std::vector<std::uint32_t> affected, stack;
  for (const auto top : split) {
    if (walk_stamp_[top] == epoch_) {continue;}
    walk_stamp_[top] = epoch_;
    stack.push_back(top);
    while (!stack.empty()) {
      const std::uint32_t g = stack.back();
      stack.pop_back();
      const int k = level_of(g);
      const Level & lv = levels_[k];
      const std::uint32_t c = g - static_cast<std::uint32_t>(offset_[k]);
      for (std::uint32_t i = lv.child_start[c]; i < lv.child_start[c + 1]; ++i) {
        const std::uint32_t ch = lv.child[i];
        if (k == 1) {
          if (vtx_stamp_[ch] != epoch_) {vtx_stamp_[ch] = epoch_; affected.push_back(ch);}
          continue;
        }
        const auto gch = static_cast<std::uint32_t>(offset_[k - 1] + ch);
        if (walk_stamp_[gch] == epoch_ || !static_allowed(k - 1, ch)) {continue;}
        walk_stamp_[gch] = epoch_;
        stack.push_back(gch);
      }
    }
  }
  // they climb again; the others end where the far cut does
  for (const auto v : affected) {
    cur_node_[v] = climb(v, q);
  }
  const auto leaf = [&](std::uint32_t v) {
      return vtx_stamp_[v] == epoch_ ? cur_node_[v] : static_leaf(v);
    };
  if (use_known) {
    for (const auto v : known_list_) {
      known_stamp_[leaf(v)] = epoch_;
    }
  }

  // 3. Their source triangles are mapped again; the far triangles they ended in are remade from
  // the source triangles left
  std::vector<std::uint32_t> remap_src, remade;
  for (const auto v : affected) {
    for (std::uint32_t i = vtx_tri_start_[v]; i < vtx_tri_start_[v + 1]; ++i) {
      const std::uint32_t t = vtx_tri_[i];
      if (src_stamp_[t] == epoch_) {continue;}
      src_stamp_[t] = epoch_;
      remap_src.push_back(t);
      const std::uint32_t f = far_of_src_[t];
      if (f != kNone && far_stamp_[f] != epoch_) {far_stamp_[f] = epoch_; remade.push_back(f);}
    }
  }
  std::sort(remap_src.begin(), remap_src.end());
  std::sort(remade.begin(), remade.end());

  LodMesh out;
  const std::size_t nf = far_slope_.size();
  out.tri.reserve(3 * nf);
  out.leg.reserve(nf);
  out.slope.reserve(nf);
  out.traversed.reserve(nf);
  const auto node_vertex = [&](std::uint32_t g) {
      if (slot_stamp_[g] != epoch_) {
        slot_stamp_[g] = epoch_;
        slot_[g] = static_cast<std::uint32_t>(out.x.size());
        const int k = level_of(g);
        const std::uint32_t c = g - static_cast<std::uint32_t>(offset_[k]);
        const Level & lv = levels_[k];
        out.x.push_back(lv.x[c]);
        out.y.push_back(lv.y[c]);
        out.z.push_back(lv.z[c]);
        if (coloured_) {
          out.r.push_back(lv.r[c]);
          out.g.push_back(lv.g[c]);
          out.b.push_back(lv.b[c]);
        }
      }
      return slot_[g];
    };
  const auto emit = [&](const std::array<std::uint32_t, 3> & g, std::uint16_t slope, double size) {
      for (const auto n : g) {
        out.tri.push_back(node_vertex(n));
      }
      out.leg.push_back(static_cast<float>(std::max(p.min_size, size)));
      out.slope.push_back(slope / 100.0f);
      out.traversed.push_back(use_known && known_stamp_[g[0]] == epoch_ &&
        known_stamp_[g[1]] == epoch_ && known_stamp_[g[2]] == epoch_);
    };
  // the far triangles untouched
  for (std::size_t f = 0; f < nf; ++f) {
    if (far_stamp_[f] == epoch_) {continue;}
    emit({far_tri_[3 * f], far_tri_[3 * f + 1], far_tri_[3 * f + 2]}, far_slope_[f], far_size_[f]);
  }
  // the far triangles some of whose source triangles are left: those decide the slope, the
  // first one the orientation
  for (const auto f : remade) {
    std::uint32_t first = kNone;
    std::uint16_t slope = 0;
    for (std::uint32_t i = far_src_start_[f]; i < far_src_start_[f + 1]; ++i) {
      const std::uint32_t t = far_src_[i];
      if (src_stamp_[t] == epoch_) {continue;}
      if (first == kNone) {first = t;}
      slope = std::max(slope, tri_slope_[t]);
    }
    if (first == kNone) {continue;}
    emit({static_leaf(tri_[3 * first]), static_leaf(tri_[3 * first + 1]),
        static_leaf(tri_[3 * first + 2])}, slope, far_size_[f]);
  }
  // the source triangles mapped again: new triangles (each has a node no far triangle has)
  std::unordered_map<std::array<std::uint32_t, 3>, std::uint32_t, TriHash> seen;
  seen.reserve(remap_src.size());
  for (const auto t : remap_src) {
    const std::array<std::uint32_t, 3> g{leaf(tri_[3 * t]), leaf(tri_[3 * t + 1]),
      leaf(tri_[3 * t + 2])};
    if (g[0] == g[1] || g[1] == g[2] || g[0] == g[2]) {continue;}
    auto key = g;
    std::sort(key.begin(), key.end());
    const auto found = seen.emplace(key, static_cast<std::uint32_t>(out.leg.size()));
    if (!found.second) {
      out.slope[found.first->second] =
        std::max(out.slope[found.first->second], tri_slope_[t] / 100.0f);
      continue;
    }
    double size = 0.0;
    for (const auto n : g) {
      size = std::max(size, levels_[level_of(n)].size);
    }
    emit(g, tri_slope_[t], size);
  }
  return out;
}

std::size_t SurfaceLodMesher::memory_bytes() const
{
  std::size_t b = 0;
  for (const auto & lv : levels_) {
    b += (lv.x.capacity() + lv.y.capacity() + lv.z.capacity() + lv.err.capacity()) *
      sizeof(float);
    b += lv.r.capacity() + lv.g.capacity() + lv.b.capacity() + lv.flags.capacity();
    b += (lv.parent.capacity() + lv.child_start.capacity() + lv.child.capacity()) *
      sizeof(std::uint32_t);
  }
  b += (tri_.capacity() + vtx_tri_start_.capacity() + vtx_tri_.capacity() +
    known_list_.capacity() + observed_list_.capacity()) * sizeof(std::uint32_t);
  b += tri_slope_.capacity() * sizeof(std::uint16_t) + known_.capacity() + observed_.capacity();
  b += offset_.capacity() * sizeof(std::size_t) + grid_.memory_bytes();
  for (const auto & g : node_grid_) {
    b += g.memory_bytes();
  }
  b += (kd_known_.capacity() + kd_observed_.capacity()) * sizeof(KdNode);
  b += (far_tri_.capacity() + far_src_start_.capacity() + far_src_.capacity() +
    far_of_src_.capacity()) * sizeof(std::uint32_t);
  b += far_slope_.capacity() * sizeof(std::uint16_t) + far_size_.capacity() * sizeof(float);
  b += (node_stamp_.capacity() + walk_stamp_.capacity() + slot_stamp_.capacity() +
    slot_.capacity() + known_stamp_.capacity() + vtx_stamp_.capacity() + cur_node_.capacity() +
    far_stamp_.capacity() + src_stamp_.capacity()) * sizeof(std::uint32_t) +
    node_memo_.capacity();
  return b;
}

}  // namespace bigmap_maps_manager
