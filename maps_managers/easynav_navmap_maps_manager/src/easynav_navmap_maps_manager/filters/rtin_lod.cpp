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

#include "easynav_navmap_maps_manager/filters/rtin_lod.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <unordered_map>
#include <queue>
#include <string>
#include <vector>

namespace easynav
{
namespace navmap
{

namespace
{

using LayerU8 = ::navmap::LayerView<std::uint8_t>;

bool fail(std::string * error, const std::string & msg)
{
  if (error) {*error = msg;}
  return false;
}

}  // namespace

RtinLod::Tri RtinLod::child(const Tri & t, int side)
{
  // the right angle c is joined to the hypotenuse midpoint m: two right triangles with their
  // right angle at m, and the legs of t as their hypotenuses
  const int mx = (t.ax + t.bx) / 2, my = (t.ay + t.by) / 2;
  if (side == 0) {return {t.cx, t.cy, t.ax, t.ay, mx, my};}
  return {t.bx, t.by, t.cx, t.cy, mx, my};
}

float RtinLod::zat(int gx, int gy) const
{
  return z_[static_cast<std::size_t>(gy) * (n_ + 1) + gx];
}

bool RtinLod::build(
  const ::navmap::NavMap & src, const std::string & layer, const Params & params,
  std::string * error)
{
  built_ = false;
  out_.reset();  // a new tree: the next extract() writes a new output, not over the old one
  layer_ = layer;
  params_ = params;
  const auto values = std::dynamic_pointer_cast<LayerU8>(src.layers.get(layer));
  if (!values) {return fail(error, "the NavMap has no uint8 layer '" + layer + "'");}
  const std::size_t nc = src.navcels.size();
  if (nc == 0 || values->size() != nc) {return fail(error, "empty NavMap or layer");}

  // Resolution: the shortest edge of the first NavCel; origin: the lowest corner
  auto P = [&src](std::uint32_t v) {return src.positions.at(v);};
  {
    const auto & c = src.navcels[0];
    double r = 1e30;
    for (int e = 0; e < 3; ++e) {
      r = std::min(r, static_cast<double>((P(c.v[e]) - P(c.v[(e + 1) % 3])).head<2>().norm()));
    }
    res_ = r;
  }
  if (!(res_ > 1e-6)) {return fail(error, "degenerate NavCels");}
  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  for (std::size_t i = 0; i < src.positions.size(); ++i) {
    minx = std::min(minx, static_cast<double>(src.positions.x[i]));
    miny = std::min(miny, static_cast<double>(src.positions.y[i]));
    maxx = std::max(maxx, static_cast<double>(src.positions.x[i]));
    maxy = std::max(maxy, static_cast<double>(src.positions.y[i]));
  }
  ox_ = minx;
  oy_ = miny;
  const int cells = std::max(
    1, static_cast<int>(std::lround(std::max(maxx - minx, maxy - miny) / res_)));
  n_ = 1;
  int log2n = 0;
  while (n_ < cells) {n_ *= 2; ++log2n;}
  leaf_depth_ = 1 + 2 * log2n;
  if (leaf_depth_ > 29) {return fail(error, "the map is too big for the tree");}
  const std::size_t ncells = static_cast<std::size_t>(n_) * n_;

  // Grid indices of a point, checked against the grid
  const double tol = 0.02 * res_;
  auto grid = [&](const Eigen::Vector3f & p, int & gx, int & gy) {
      const double fx = (p.x() - ox_) / res_, fy = (p.y() - oy_) / res_;
      gx = static_cast<int>(std::lround(fx));
      gy = static_cast<int>(std::lround(fy));
      return std::abs(fx - gx) * res_ < tol && std::abs(fy - gy) * res_ < tol;
    };

  z_.assign(static_cast<std::size_t>(n_ + 1) * (n_ + 1), std::nanf(""));
  cell_src_.assign(2 * ncells, kNone);
  cell_diag_.assign(ncells, 0);
  src_cell_.assign(nc, kNone);
  src_value_ = values->data();
  src_count_ = nc;
  pending_.clear();
  cursor_ = 0;

  for (std::uint32_t i = 0; i < nc; ++i) {
    const auto & c = src.navcels[i];
    int g[3][2];
    for (int k = 0; k < 3; ++k) {
      if (!grid(P(c.v[k]), g[k][0], g[k][1])) {
        return fail(error, "NavCel " + std::to_string(i) + " is not on a regular grid");
      }
    }
    // the right angle: the vertex whose two edges are one cell long, along x and y
    int ra = -1;
    for (int k = 0; k < 3 && ra < 0; ++k) {
      const int * p = g[k], * q = g[(k + 1) % 3], * r = g[(k + 2) % 3];
      const int d1x = q[0] - p[0], d1y = q[1] - p[1], d2x = r[0] - p[0], d2y = r[1] - p[1];
      if (std::abs(d1x) + std::abs(d1y) == 1 && std::abs(d2x) + std::abs(d2y) == 1 &&
        d1x * d2x + d1y * d2y == 0)
      {
        ra = k;
      }
    }
    if (ra < 0) {
      return fail(error, "NavCel " + std::to_string(i) + " is not a cell half (" +
               std::to_string(res_) + " m legs)");
    }
    const int cx = std::min({g[0][0], g[1][0], g[2][0]});
    const int cy = std::min({g[0][1], g[1][1], g[2][1]});
    const int rx = g[ra][0] - cx, ry = g[ra][1] - cy;
    const std::uint8_t diag = (rx != ry) ? 1 : 0;   // right angle off the main diagonal
    const std::size_t cell = static_cast<std::size_t>(cy) * n_ + cx;
    const std::size_t slot = 2 * cell + ry;
    if (cell_src_[slot] != kNone ||
      (cell_src_[2 * cell + 1 - ry] != kNone && cell_diag_[cell] != diag))
    {
      return fail(error, "overlapping NavCels in cell (" + std::to_string(cx) + ", " +
               std::to_string(cy) + "): several levels are not supported");
    }
    cell_src_[slot] = i;
    cell_diag_[cell] = diag;
    src_cell_[i] = static_cast<std::uint32_t>(cell);
    for (int k = 0; k < 3; ++k) {
      float & z = z_[static_cast<std::size_t>(g[k][1]) * (n_ + 1) + g[k][0]];
      const float pz = P(c.v[k]).z();
      if (std::isnan(z)) {
        z = pz;
      } else if (std::abs(z - pz) > 0.01f) {
        return fail(error, "two heights at one vertex: several levels are not supported");
      }
    }
  }

  const std::size_t nodes = std::size_t{1} << (leaf_depth_ + 1);
  value_.assign(nodes, 0);
  state_.assign(nodes, 0);
  partner_.assign(nodes, kNone);
  cell_leaf_.assign(2 * ncells, kNone);
  cell_leaf_main_.assign(ncells, 0);

  // Pass 1: leaves, merges and the pairs sharing a hypotenuse (by its midpoint)
  std::vector<std::uint32_t> first_at(z_.size(), kNone);
  const int T = 2 * n_;
  // recursive: the depth is at most 29
  // corner of the doubled grid -> point; the normal of a node's triangle
  auto point = [this](int dx, int dy) {
      return Eigen::Vector3f(
        static_cast<float>(dx / 2 * res_), static_cast<float>(dy / 2 * res_), zat(dx / 2, dy / 2));
    };
  auto normal = [&point](const Tri & t) {
      Eigen::Vector3f n = (point(t.bx, t.by) - point(t.ax, t.ay)).cross(
        point(t.cx, t.cy) - point(t.ax, t.ay));
      if (n.z() < 0.0f) {n = -n;}
      const float len = n.norm();
      return len > 0.0f ? Eigen::Vector3f(n / len) : Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    };
  const float cos_max = std::cos(params_.max_angle_deg * static_cast<float>(M_PI) / 180.0f);
  // Geometry of a subtree: the normal of its triangle and a bound of how far the source vertices
  // under it are from its plane. Two children merge if both fit the parent's plane: their
  // normals within max_angle_deg of it, and the bound within height_tolerance.
  struct Geo
  {
    Eigen::Vector3f n;
    float dev;
  };
  // recursive: the depth is at most 29
  std::function<Geo(std::uint32_t, const Tri &, int)> rec;
  rec = [&](std::uint32_t k, const Tri & t, int depth) -> Geo {
      if (depth == leaf_depth_) {
        leaf_state(k, t);
        return {(state_[k] & kExists) ? normal(t) : Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.0f};
      }
      const std::size_t mv = static_cast<std::size_t>((t.ay + t.by) / 4) * (n_ + 1) +
        (t.ax + t.bx) / 4;
      if (first_at[mv] == kNone) {
        first_at[mv] = k;
      } else {
        partner_[k] = first_at[mv];
        partner_[first_at[mv]] = k;
      }
      const Geo g0 = rec(2 * k, child(t, 0), depth + 1);
      const Geo g1 = rec(2 * k + 1, child(t, 1), depth + 1);
      const std::uint8_t s0 = state_[2 * k], s1 = state_[2 * k + 1];
      std::uint8_t s = 0;
      if ((s0 & kEmpty) && (s1 & kEmpty)) {s |= kEmpty;}
      Geo g{Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.0f};
      if ((s0 & kComplete) && (s1 & kComplete)) {
        s |= kComplete;
        g.n = normal(t);
        // the only vertex of the children off the parent's corners: the hypotenuse midpoint
        const float off = std::abs(
          g.n.dot(point((t.ax + t.bx) / 2, (t.ay + t.by) / 2) - point(t.ax, t.ay)));
        g.dev = off + std::max(g0.dev, g1.dev);
        if ((s0 & kFlat) && (s1 & kFlat) && g.n.dot(g0.n) >= cos_max - 1e-6f &&
          g.n.dot(g1.n) >= cos_max - 1e-6f && g.dev <= params_.height_tolerance)
        {
          s |= kFlat;
        }
      }
      state_[k] = s;
      recompute_merge(k);
      return g;
    };
  rec(2, Tri{0, 0, T, T, T, 0}, 1);
  rec(3, Tri{T, T, 0, 0, 0, T}, 1);

  // Pass 2: splits, deepest first; a split splits the pair sharing the hypotenuse too
  for (std::uint32_t k = (std::uint32_t{1} << leaf_depth_) - 1; k >= 2; --k) {
    if (need_split(k)) {
      state_[k] |= kSplit;
      if (partner_[k] != kNone) {state_[partner_[k]] |= kSplit;}
    }
  }
  // Tiles: subtrees of about tile_size^2 m^2 (a root covers (n res)^2 / 2)
  const double root_area = 0.5 * (n_ * res_) * (n_ * res_);
  const double tile_area = std::max(1e-6, static_cast<double>(params_.tile_size) *
      params_.tile_size);
  tile_depth_ = 1 + std::max(0, static_cast<int>(std::ceil(std::log2(root_area / tile_area))));
  tile_depth_ = std::min(tile_depth_, leaf_depth_);
  tiles_.assign((std::size_t{1} << tile_depth_) + 1, Tile{});
  vmap_.assign(z_.size(), kNone);
  edge_.assign(static_cast<std::size_t>(2 * n_ + 1) * (2 * n_ + 1), kNone);
  built_ = true;
  return true;
}

void RtinLod::leaf_state(std::uint32_t k, const Tri & t)
{
  const int cx = std::min({t.ax, t.bx, t.cx}) / 2, cy = std::min({t.ay, t.by, t.cy}) / 2;
  const std::size_t cell = static_cast<std::size_t>(cy) * n_ + cx;
  const int slot = t.cy / 2 - cy;   // the half: the row of its right angle in the cell
  cell_leaf_[2 * cell + slot] = k;
  cell_leaf_main_[cell] = (t.ax - t.bx) * (t.ay - t.by) > 0 ? 1 : 0;
  refresh_leaf(k, cell, slot);
}

void RtinLod::refresh_leaf(std::uint32_t k, std::size_t cell, int slot)
{
  // the source half it is, or (other diagonal) the two halves it overlaps: the highest value,
  // and it exists only if both do
  const std::uint32_t s0 = cell_src_[2 * cell], s1 = cell_src_[2 * cell + 1];
  bool exists = false;
  std::uint8_t v = 0;
  if (cell_diag_[cell] == cell_leaf_main_[cell]) {
    const std::uint32_t s = cell_src_[2 * cell + slot];
    exists = s != kNone;
    if (exists) {v = src_value_[s];}
  } else {
    exists = s0 != kNone && s1 != kNone;
    if (exists) {v = std::max(src_value_[s0], src_value_[s1]);}
  }
  value_[k] = v;
  state_[k] = exists ? (kExists | kComplete | kFlat | kUniform) : kEmpty;
}

bool RtinLod::recompute_merge(std::uint32_t k)
{
  const std::uint8_t s = state_[k], s0 = state_[2 * k], s1 = state_[2 * k + 1];
  const bool u = (s & kComplete) && (s & kFlat) && (s0 & kUniform) && (s1 & kUniform) &&
    value_[2 * k] == value_[2 * k + 1];
  const std::uint8_t v = u ? value_[2 * k] : 0;
  const bool changed = u != static_cast<bool>(s & kUniform) || v != value_[k];
  state_[k] = u ? (s | kUniform) : (s & ~kUniform);
  value_[k] = v;
  return changed;
}

bool RtinLod::need_split(std::uint32_t k) const
{
  const std::uint8_t s = state_[k];
  if (s & kEmpty) {return false;}
  return !(s & kUniform) || (state_[2 * k] & kSplit) || (state_[2 * k + 1] & kSplit);
}

const std::vector<std::uint8_t> * RtinLod::layer_data(const ::navmap::NavMap & src) const
{
  if (!built_) {return nullptr;}
  const auto values = std::dynamic_pointer_cast<LayerU8>(src.layers.get(layer_));
  if (!values || values->size() != src_count_) {return nullptr;}
  return &values->data();
}

bool RtinLod::read(std::uint32_t i, const std::vector<std::uint8_t> & v)
{
  if (v[i] == src_value_[i]) {return false;}
  src_value_[i] = v[i];
  const std::size_t cell = src_cell_[i];
  for (int s = 0; s < 2; ++s) {
    if (cell_leaf_[2 * cell + s] != kNone) {pending_.push_back((cell << 1) | s);}
  }
  return true;
}

std::size_t RtinLod::update(const ::navmap::NavMap & src)
{
  const auto * v = layer_data(src);
  if (!v) {return 0;}
  std::size_t changed = 0;
  for (std::uint32_t i = 0; i < src_count_; ++i) {
    changed += read(i, *v);
  }
  return apply(changed);
}

std::size_t RtinLod::update_window(
  const ::navmap::NavMap & src, double x, double y, double radius, std::size_t sweep)
{
  const auto * v = layer_data(src);
  if (!v) {return 0;}
  std::size_t changed = 0;
  // the cells around the robot, where the sensors change the costs
  const int i0 = std::max(0, static_cast<int>(std::floor((x - radius - ox_) / res_)));
  const int j0 = std::max(0, static_cast<int>(std::floor((y - radius - oy_) / res_)));
  const int i1 = std::min(n_ - 1, static_cast<int>(std::floor((x + radius - ox_) / res_)));
  const int j1 = std::min(n_ - 1, static_cast<int>(std::floor((y + radius - oy_) / res_)));
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      const std::size_t cell = static_cast<std::size_t>(j) * n_ + i;
      for (int s = 0; s < 2; ++s) {
        if (cell_src_[2 * cell + s] != kNone) {changed += read(cell_src_[2 * cell + s], *v);}
      }
    }
  }
  // and a slice of the whole map, in turns: what changes far away (an obstacle the memory
  // forgets after the robot left) is caught within src_count / sweep cycles
  for (std::size_t k = 0; k < std::min(sweep, src_count_); ++k) {
    changed += read(static_cast<std::uint32_t>(cursor_), *v);
    cursor_ = (cursor_ + 1) % src_count_;
  }
  return apply(changed);
}

std::size_t RtinLod::apply(std::size_t changed)
{
  std::vector<std::uint64_t> leaves;
  leaves.swap(pending_);
  if (changed == 0) {return 0;}
  std::sort(leaves.begin(), leaves.end());
  leaves.erase(std::unique(leaves.begin(), leaves.end()), leaves.end());

  // merges, from the leaves up while something changes
  std::priority_queue<std::uint32_t> todo;   // deepest (largest id) first
  for (const auto cs : leaves) {
    const std::size_t cell = cs >> 1;
    const int slot = static_cast<int>(cs & 1);
    const std::uint32_t leaf = cell_leaf_[2 * cell + slot];
    const std::uint8_t old_v = value_[leaf];
    refresh_leaf(leaf, cell, slot);
    if (value_[leaf] == old_v) {continue;}
    mark(leaf, false);
    for (std::uint32_t k = leaf >> 1; k >= 2; k >>= 1) {
      if (!recompute_merge(k)) {break;}
      mark(k, false);
      todo.push(k);
    }
  }
  // splits: a change in a node's split changes its parent's and its pair's parent's
  std::uint32_t last = 0;
  while (!todo.empty()) {
    const std::uint32_t k = todo.top();
    todo.pop();
    if (k == last) {continue;}
    last = k;
    const std::uint32_t p = partner_[k];
    const bool split = need_split(k) || (p != kNone && need_split(p));
    if (split == static_cast<bool>(state_[k] & kSplit)) {continue;}
    auto set = [this, split](std::uint32_t x) {
        state_[x] = split ? (state_[x] | kSplit) : (state_[x] & ~kSplit);
      };
    set(k);
    mark(k, true);
    if (k >= 4) {todo.push(k >> 1);}
    if (p != kNone) {
      set(p);
      mark(p, true);
      if (p >= 4) {todo.push(p >> 1);}
    }
  }
  return changed;
}

void RtinLod::mark(std::uint32_t k, bool split_changed)
{
  const int depth = 31 - __builtin_clz(k);
  const std::size_t first = std::size_t{1} << tile_depth_;
  if (depth >= tile_depth_) {
    tiles_[(k >> (depth - tile_depth_)) - first].dirty = true;
    return;
  }
  tiles_.back().dirty = true;   // the top of the tree
  if (split_changed) {          // what it covered (or now uncovers) are whole tiles
    const int shift = tile_depth_ - depth;
    const std::size_t t0 = (static_cast<std::size_t>(k) << shift) - first;
    for (std::size_t t = t0; t < t0 + (std::size_t{1} << shift); ++t) {
      tiles_[t].dirty = true;
    }
  }
}

RtinLod::Tri RtinLod::node_tri(std::uint32_t k, int depth) const
{
  // below the leading bit: the root (2 or 3), then the side taken at each level
  const int T = 2 * n_;
  Tri t = ((k >> (depth - 1)) & 1) ? Tri{T, T, 0, 0, 0, T} : Tri{0, 0, T, T, T, 0};
  for (int b = depth - 2; b >= 0; --b) {
    t = child(t, (k >> b) & 1);
  }
  return t;
}

namespace
{
constexpr std::uint32_t kPaired = std::numeric_limits<std::uint32_t>::max() - 1;
}  // namespace

int RtinLod::extract_node(
  std::uint32_t k, const Tri & t, int depth, int stop_depth, Tile & tile)
{
  if (depth == stop_depth) {return -1;}
  const std::uint8_t s = state_[k];
  if (s & kEmpty) {return -1;}
  const bool leaf = depth == leaf_depth_;
  if (!leaf && (s & kSplit)) {
    // the tree is the BVH: a split node bounds its two children
    const int l = extract_node(2 * k, child(t, 0), depth + 1, stop_depth, tile);
    const int r = extract_node(2 * k + 1, child(t, 1), depth + 1, stop_depth, tile);
    if (l < 0 || r < 0) {return l < 0 ? r : l;}
    ::navmap::BVHNode node;
    node.box = tile.bvh[l].box;
    node.box.expand(tile.bvh[r].box);
    node.left = l;
    node.right = r;
    tile.bvh.push_back(node);
    return static_cast<int>(tile.bvh.size()) - 1;
  }
  if (leaf && !(s & kExists)) {return -1;}

  auto vertex = [&](int dx, int dy) {
      const int gx = dx / 2, gy = dy / 2;
      const std::size_t vi = static_cast<std::size_t>(gy) * (n_ + 1) + gx;
      std::uint32_t & id = vmap_[vi];
      if (id == kNone) {
        id = static_cast<std::uint32_t>(tile.grid.size());
        tile.grid.push_back(static_cast<std::uint32_t>(vi));
        touched_v_.push_back(static_cast<std::uint32_t>(vi));
      }
      return id;
    };
  int px[3] = {t.ax, t.bx, t.cx}, py[3] = {t.ay, t.by, t.cy};
  // counter-clockwise seen from above: the normal points up
  if ((t.bx - t.ax) * (t.cy - t.ay) - (t.by - t.ay) * (t.cx - t.ax) < 0) {
    std::swap(px[0], px[1]);
    std::swap(py[0], py[1]);
  }
  ::navmap::NavCel cell;
  cell.v[0] = vertex(px[0], py[0]);
  cell.v[1] = vertex(px[1], py[1]);
  cell.v[2] = vertex(px[2], py[2]);
  const auto cid = static_cast<std::uint32_t>(tile.cells.size());
  tile.cells.push_back(cell);

  // neighbours: without T-junctions, the midpoint of an edge (doubled grid) is the edge
  const int T = 2 * n_;
  for (int e = 0; e < 3; ++e) {
    const std::size_t m = static_cast<std::size_t>((py[e] + py[(e + 1) % 3]) / 2) * (T + 1) +
      (px[e] + px[(e + 1) % 3]) / 2;
    std::uint32_t & slot = edge_[m];
    if (slot == kNone) {
      slot = cid * 3 + e;
      touched_e_.push_back(static_cast<std::uint32_t>(m));
    } else {
      tile.cells[cid].neighbor[e] = slot / 3;
      tile.cells[slot / 3].neighbor[slot % 3] = cid;
      slot = kPaired;
    }
  }

  ::navmap::BVHNode node;
  for (int v = 0; v < 3; ++v) {
    const int gx = px[v] / 2, gy = py[v] / 2;
    node.box.expand(
      Eigen::Vector3f(
        static_cast<float>(ox_ + gx * res_), static_cast<float>(oy_ + gy * res_), zat(gx, gy)));
  }
  node.start = static_cast<int>(tile.prim.size());
  node.count = 1;
  tile.prim.push_back(static_cast<int>(cid));
  tile.bvh.push_back(node);

  const int level = leaf_depth_ - depth;
  tile.values.push_back(value_[k]);
  tile.levels.push_back(static_cast<std::uint8_t>(level));
  if (level == 0) {++tile.finest;}
  tile.max_level = std::max(tile.max_level, level);
  return static_cast<int>(tile.bvh.size()) - 1;
}

void RtinLod::extract_tile(std::size_t i)
{
  Tile & tile = tiles_[i];
  const std::size_t keep = tile.cells.size();
  tile = Tile{};
  tile.dirty = false;
  tile.cells.reserve(keep);
  tile.grid.reserve(keep);
  tile.bvh.reserve(2 * keep);
  tile.prim.reserve(keep);
  tile.values.reserve(keep);
  tile.levels.reserve(keep);

  int root = -1;
  if (i + 1 == tiles_.size()) {
    // the top: what is merged above the tiles' depth
    const int T = 2 * n_;
    const int l = extract_node(2, Tri{0, 0, T, T, T, 0}, 1, tile_depth_, tile);
    const int r = extract_node(3, Tri{T, T, 0, 0, 0, T}, 1, tile_depth_, tile);
    root = l < 0 ? r : l;
    if (l >= 0 && r >= 0) {
      ::navmap::BVHNode node;
      node.box = tile.bvh[l].box;
      node.box.expand(tile.bvh[r].box);
      node.left = l;
      node.right = r;
      tile.bvh.push_back(node);
      root = static_cast<int>(tile.bvh.size()) - 1;
    }
  } else {
    const auto k = static_cast<std::uint32_t>((std::size_t{1} << tile_depth_) + i);
    // inside a NavCel merged above it (or nothing under it): no NavCels of its own
    bool covered = false;
    for (std::uint32_t a = k >> 1; a >= 2 && !covered; a >>= 1) {
      covered = !(state_[a] & kSplit);
    }
    if (!covered) {root = extract_node(k, node_tri(k, tile_depth_), tile_depth_, -1, tile);}
  }

  if (root >= 0) {
    // NavMap starts its traversals at node 0: the root (built last) moves to the front
    const int last = static_cast<int>(tile.bvh.size()) - 1;
    auto renum = [last](int n) {return n < 0 ? n : (n == last ? 0 : n + 1);};
    std::vector<::navmap::BVHNode> bvh(tile.bvh.size());
    for (int n = 0; n <= last; ++n) {
      auto b = tile.bvh[n];
      b.left = renum(b.left);
      b.right = renum(b.right);
      bvh[renum(n)] = b;
    }
    tile.bvh.swap(bvh);
  }
  // the edges left unpaired are the tile's border; the scratch tables back to empty
  for (const auto m : touched_e_) {
    if (edge_[m] != kPaired) {tile.border.emplace_back(m, edge_[m]);}
    edge_[m] = kNone;
  }
  for (const auto v : touched_v_) {
    vmap_[v] = kNone;
  }
  touched_e_.clear();
  touched_v_.clear();
}

void RtinLod::reset_output()
{
  out_ = std::make_shared<::navmap::NavMap>();
  auto & out = *out_;
  // vertex 0: the anchor of the dead NavCels (degenerate: no area, no neighbours, in no BVH)
  out.add_vertex(Eigen::Vector3f(static_cast<float>(ox_), static_cast<float>(oy_), 0.0f));
  for (std::size_t i = 0; i < tiles_.size(); ++i) {
    out.create_surface("map");
  }
  values_l_ = out.add_layer<std::uint8_t>(layer_, "value of the source layer", "", 0);
  levels_l_ = out.add_layer<std::uint8_t>(
    "lod_level", "merges of this NavCel (0: source resolution; each one doubles the area)", "", 0);
  modified_l_ = out.add_layer<std::uint8_t>(
    "modified", "1: not at the source resolution (merged)", "", 0);
  slots_.assign(tiles_.size(), Slot{});
  border_edges_.clear();
  gvert_.assign(z_.size(), kNone);
  dead_ = 0;
  changes_full_ = true;
  changed_ranges_.clear();
  changed_surfaces_.clear();
}

RtinLod::Changes RtinLod::take_changes()
{
  Changes ch;
  ch.full = changes_full_ || !out_;
  ch.first_vertex = changes_first_vertex_;
  if (!ch.full) {
    std::sort(changed_ranges_.begin(), changed_ranges_.end());
    const std::uint32_t n = static_cast<std::uint32_t>(out_->navcels.size());
    std::uint32_t next = 0;   // ranges overlap: each NavCel once
    for (const auto & [from, to] : changed_ranges_) {
      for (std::uint32_t c = std::max(from, next); c < std::min(to, n); ++c) {
        ch.navcels.push_back(c);
      }
      next = std::max(next, to);
    }
    std::sort(changed_surfaces_.begin(), changed_surfaces_.end());
    changed_surfaces_.erase(
      std::unique(changed_surfaces_.begin(), changed_surfaces_.end()), changed_surfaces_.end());
    ch.surfaces = changed_surfaces_;
  }
  changes_full_ = false;
  changes_first_vertex_ = out_ ? static_cast<std::uint32_t>(out_->positions.x.size()) : 0;
  changed_ranges_.clear();
  changed_surfaces_.clear();
  return ch;
}

void RtinLod::kill(std::uint32_t from, std::uint32_t to)
{
  if (from < to) {changed_ranges_.emplace_back(from, to);}
  auto & cells = out_->navcels;
  for (std::uint32_t c = from; c < to; ++c) {
    cells[c] = ::navmap::NavCel{};   // vertices 0, 0, 0; no neighbours
    cells[c].area = 0.0f;
    values_l_->data()[c] = 254;     // lethal, should anything look at it
    levels_l_->data()[c] = 0;
    modified_l_->data()[c] = 0;
  }
  dead_ += to - from;
}

void RtinLod::write_tile(std::size_t i)
{
  const Tile & t = tiles_[i];
  Slot & s = slots_[i];
  auto & out = *out_;
  auto & cells = out.navcels;

  // 1. its border edges let go of the neighbour tiles' NavCels
  for (const auto & [m, g] : s.border) {
    auto it = border_edges_.find(m);
    if (it == border_edges_.end()) {continue;}
    auto & own = it->second;
    if (own[0] == g) {own[0] = own[1]; own[1] = kNone;} else if (own[1] == g) {own[1] = kNone;}
    if (own[0] == kNone) {
      border_edges_.erase(it);
    } else {
      cells[own[0] / 3].neighbor[own[0] % 3] = kNone;   // it pointed at us
    }
  }
  s.border.clear();

  // 2. its range: the same one if it fits, else a new one at the end (the old one dies)
  const auto n = static_cast<std::uint32_t>(t.cells.size());
  if (n > s.cap) {
    kill(s.base, s.base + s.count);
    s.base = static_cast<std::uint32_t>(cells.size());
    s.cap = n + n / 4 + 16;
    const std::size_t size = cells.size() + s.cap;
    cells.resize(size);
    values_l_->data().resize(size, 254);
    levels_l_->data().resize(size, 0);
    modified_l_->data().resize(size, 0);
    dead_ += s.cap;   // a new NavCel is dead (vertices 0, 0, 0) until written
    s.count = 0;
    changed_ranges_.emplace_back(s.base, s.base + s.cap);   // all of it is new
  }
  changed_ranges_.emplace_back(s.base, s.base + std::max(n, s.count));
  changed_surfaces_.push_back(i);

  // 3. the NavCels, on the shared vertices, with their normals
  local_to_global_.resize(t.grid.size());
  for (std::size_t v = 0; v < t.grid.size(); ++v) {
    const std::uint32_t gi = t.grid[v];
    std::uint32_t & id = gvert_[gi];
    if (id == kNone) {
      const int gx = static_cast<int>(gi % (n_ + 1)), gy = static_cast<int>(gi / (n_ + 1));
      id = out.add_vertex(
        Eigen::Vector3f(
          static_cast<float>(ox_ + gx * res_), static_cast<float>(oy_ + gy * res_), zat(gx, gy)));
    }
    local_to_global_[v] = id;
  }
  const std::uint32_t base = s.base;
  for (std::uint32_t c = 0; c < n; ++c) {
    auto cell = t.cells[c];
    for (int e = 0; e < 3; ++e) {
      cell.v[e] = local_to_global_[cell.v[e]];
      if (cell.neighbor[e] != kNone) {cell.neighbor[e] += base;}
    }
    const Eigen::Vector3f p0 = out.positions.at(cell.v[0]), p1 = out.positions.at(cell.v[1]),
      p2 = out.positions.at(cell.v[2]);
    const Eigen::Vector3f nn = (p1 - p0).cross(p2 - p0);
    const float len = nn.norm();
    cell.area = 0.5f * len;
    cell.normal = len > 0.0f ? Eigen::Vector3f(nn / len) : Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    cells[base + c] = cell;
    values_l_->data()[base + c] = t.values[c];
    levels_l_->data()[base + c] = t.levels[c];
    modified_l_->data()[base + c] = t.levels[c] > 0 ? 1 : 0;
  }
  if (n < s.count) {kill(base + n, base + s.count);}
  if (n > s.count) {dead_ -= n - s.count;}
  s.count = n;

  // 4. its surface: the NavCels and the BVH (prim ids made global)
  auto & surf = out.surfaces[i];
  surf.navcels.resize(n);
  for (std::uint32_t c = 0; c < n; ++c) {
    surf.navcels[c] = base + c;
  }
  surf.prim_indices.resize(t.prim.size());
  for (std::size_t p = 0; p < t.prim.size(); ++p) {
    surf.prim_indices[p] = t.prim[p] + static_cast<int>(base);
  }
  surf.bvh = t.bvh;
  surf.aabb = t.bvh.empty() ? ::navmap::AABB{} : t.bvh.front().box;

  // 5. its border edges meet the neighbour tiles'
  for (const auto & [m, ce] : t.border) {
    const std::uint32_t g = (base + ce / 3) * 3 + ce % 3;
    s.border.emplace_back(m, g);
    auto & own = border_edges_.try_emplace(m, std::array<std::uint32_t, 2>{kNone, kNone})
      .first->second;
    if (own[0] == kNone) {
      own[0] = g;
    } else {
      own[1] = g;
      cells[g / 3].neighbor[g % 3] = own[0] / 3;
      cells[own[0] / 3].neighbor[own[0] % 3] = g / 3;
    }
  }
}

std::shared_ptr<::navmap::NavMap> RtinLod::extract(Stats * stats)
{
  Stats st;
  if (!built_) {
    if (stats) {*stats = st;}
    return std::make_shared<::navmap::NavMap>();
  }
  using clock = std::chrono::steady_clock;
  auto ms = [](clock::time_point a) {
      return std::chrono::duration<double, std::milli>(clock::now() - a).count();
    };

  auto t0 = clock::now();
  std::vector<std::size_t> changed;
  for (std::size_t i = 0; i < tiles_.size(); ++i) {
    if (tiles_[i].dirty) {extract_tile(i); changed.push_back(i);}
  }
  st.tiles_rebuilt = changed.size();
  st.ms_mesh = ms(t0);

  t0 = clock::now();
  std::size_t live = 0;
  for (const auto & s : slots_) {
    live += s.count;
  }
  if (!out_ || (dead_ > live && dead_ > 65536)) {
    // first time, or more dead NavCels than live ones: every tile again, from its cache
    reset_output();
    for (std::size_t i = 0; i < tiles_.size(); ++i) {
      write_tile(i);
    }
  } else {
    for (const auto i : changed) {
      write_tile(i);
    }
  }
  st.ms_assemble = ms(t0);

  t0 = clock::now();
  // neighbours, BVHs, normals are already there: only the surface indices
  out_->rebuild_geometry_accels(false, false, false);
  st.ms_finish = ms(t0);

  for (std::size_t i = 0; i < tiles_.size(); ++i) {
    const auto & t = tiles_[i];
    if (t.cells.empty()) {continue;}
    ++st.tiles;
    st.triangles += t.cells.size();
    st.finest += t.finest;
    st.max_level = std::max(st.max_level, t.max_level);
  }
  st.dead = dead_;
  if (stats) {*stats = st;}
  return out_;
}

}  // namespace navmap
}  // namespace easynav
