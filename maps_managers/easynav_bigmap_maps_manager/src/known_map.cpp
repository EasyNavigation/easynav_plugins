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

#include "easynav_bigmap_maps_manager/known_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bigmap_maps_manager
{

KnownMap::KnownMap(int side, double resolution, int /*grid*/)
: side_(side), res_(resolution), center_((side - 1) / 2),
  mark_(static_cast<std::size_t>(side_) * side_, 0)
{
  // The marks cover the DEM's own nodes only: distance() answers for any node (the padding of the
  // RTIN grid too) from the tree of the marked ones, so a mark per padded node (2.7 times the
  // DEM's nodes for a side of 5001) bought nothing
}

int KnownMap::mark(double x, double y, double radius)
{
  const double fx = x / res_ + center_, fy = y / res_ + center_;
  const int reach = static_cast<int>(std::ceil(radius / res_));
  const int cx = static_cast<int>(std::lround(fx)), cy = static_cast<int>(std::lround(fy));
  int added = 0;
  for (int j = cy - reach; j <= cy + reach; ++j) {
    for (int i = cx - reach; i <= cx + reach; ++i) {
      if (i < 0 || j < 0 || i >= side_ || j >= side_) {continue;}
      if (std::hypot((i - fx) * res_, (j - fy) * res_) > radius) {continue;}
      auto & m = mark_[static_cast<std::size_t>(j) * side_ + i];
      if (!m) {
        m = 1;
        nodes_.push_back(static_cast<std::uint32_t>(static_cast<std::size_t>(j) * side_ + i));
        ++added;
      }
    }
  }
  count_ += added;
  dirty_ = dirty_ || added > 0;
  return added;
}

bool KnownMap::mark_node(int ix, int iy)
{
  if (ix < 0 || iy < 0 || ix >= side_ || iy >= side_) {return false;}
  auto & m = mark_[static_cast<std::size_t>(iy) * side_ + ix];
  if (m) {return false;}
  m = 1;
  nodes_.push_back(static_cast<std::uint32_t>(static_cast<std::size_t>(iy) * side_ + ix));
  ++count_;
  dirty_ = true;
  return true;
}

void KnownMap::update()
{
  if (!dirty_) {return;}
  dirty_ = false;
  compute(nodes_);
}

void KnownMap::update_within(double x, double y, double radius)
{
  const double fx = x / res_ + center_, fy = y / res_ + center_;
  std::vector<std::uint32_t> src;
  for (const auto k : nodes_) {
    const std::uint32_t side = static_cast<std::uint32_t>(side_);
    const double i = k % side, j = k / side;
    if (std::hypot((i - fx) * res_, (j - fy) * res_) <= radius) {src.push_back(k);}
  }
  dirty_ = true;  // the next update() starts from all of them again
  compute(src);
}

std::size_t KnownMap::forget_outside(double x, double y, double radius)
{
  const double fx = x / res_ + center_, fy = y / res_ + center_;
  const std::uint32_t side = static_cast<std::uint32_t>(side_);
  std::vector<std::uint32_t> kept;
  kept.reserve(nodes_.size());
  std::size_t forgotten = 0;
  for (const auto k : nodes_) {
    const double i = k % side, j = k / side;
    if (std::hypot((i - fx) * res_, (j - fy) * res_) <= radius) {
      kept.push_back(k);
    } else {
      mark_[k] = 0;
      ++forgotten;
    }
  }
  if (forgotten > 0) {
    nodes_ = std::move(kept);
    count_ = nodes_.size();
    dirty_ = true;
  }
  return forgotten;
}

bool KnownMap::near_marked(int ix, int iy, int reach) const
{
  for (int j = iy - reach; j <= iy + reach; ++j) {
    for (int i = ix - reach; i <= ix + reach; ++i) {
      if (known(i, j)) {return true;}
    }
  }
  return false;
}

// A balanced 2-D tree of the source nodes, built in place over `tree_` (the median along the
// wider axis of each half, recursively): the exact nearest source of any node is found in
// O(log n) on average, so a re-mesh costs the few tens of thousands of queries the mesher makes
// instead of an exact distance transform of the whole grid (60 ms of a 75 ms re-mesh at 4 M
// nodes; the mesher visits a few hundred nodes per metre of path, not the grid)
void KnownMap::compute(const std::vector<std::uint32_t> & sources)
{
  sources_ = sources.size();
  tree_.clear();
  tree_.reserve(sources.size());
  const auto side = static_cast<std::uint32_t>(side_);
  for (const auto k : sources) {
    tree_.push_back({static_cast<int>(k % side), static_cast<int>(k / side)});
  }
  build(0, tree_.size());
}

void KnownMap::build(std::size_t lo, std::size_t hi)
{
  // each node of the tree is the median of its range: [lo, mid) left, (mid, hi) right
  if (hi - lo <= 1) {return;}
  int min_x = tree_[lo].x, max_x = min_x, min_y = tree_[lo].y, max_y = min_y;
  for (std::size_t i = lo + 1; i < hi; ++i) {
    min_x = std::min(min_x, tree_[i].x);
    max_x = std::max(max_x, tree_[i].x);
    min_y = std::min(min_y, tree_[i].y);
    max_y = std::max(max_y, tree_[i].y);
  }
  const bool by_x = max_x - min_x >= max_y - min_y;
  const std::size_t mid = lo + (hi - lo) / 2;
  std::nth_element(
    tree_.begin() + lo, tree_.begin() + mid, tree_.begin() + hi,
    [by_x](const Node & a, const Node & b) {return by_x ? a.x < b.x : a.y < b.y;});
  tree_[mid].by_x = by_x;
  build(lo, mid);
  build(mid + 1, hi);
}

void KnownMap::nearest(std::size_t lo, std::size_t hi, int x, int y, double & best) const
{
  if (hi <= lo) {return;}
  const std::size_t mid = lo + (hi - lo) / 2;
  const Node & n = tree_[mid];
  const double dx = static_cast<double>(x - n.x), dy = static_cast<double>(y - n.y);
  best = std::min(best, dx * dx + dy * dy);
  const double delta = n.by_x ? dx : dy;      // signed distance to the splitting line
  if (delta < 0.0) {
    nearest(lo, mid, x, y, best);
    if (delta * delta < best) {nearest(mid + 1, hi, x, y, best);}
  } else {
    nearest(mid + 1, hi, x, y, best);
    if (delta * delta < best) {nearest(lo, mid, x, y, best);}
  }
}

double KnownMap::distance(int ix, int iy) const
{
  if (tree_.empty()) {return std::numeric_limits<double>::infinity();}
  double best = std::numeric_limits<double>::infinity();
  nearest(0, tree_.size(), ix, iy, best);
  return std::sqrt(best) * res_;
}

bool KnownMap::known(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= side_ || iy >= side_) {return false;}
  return mark_[static_cast<std::size_t>(iy) * side_ + ix] != 0;
}

}  // namespace bigmap_maps_manager
