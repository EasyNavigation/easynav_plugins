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

#include "easynav_bonxai_planner/Planner3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace easynav
{
namespace bonxai_planner
{

namespace
{

constexpr double kInf = 1e20;

/// Squared distance transform of a sampled function (Felzenszwalb & Huttenlocher).
void transform_1d(
  const std::vector<double> & f, int n, std::vector<double> & d, std::vector<int> & v,
  std::vector<double> & z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  auto intersection = [&](int q, int p) {
      return ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) /
             (2.0 * (q - p));
    };
  for (int q = 1; q < n; ++q) {
    double s = intersection(q, v[k]);
    while (s <= z[k]) {  // z[0] is -inf: it stops there
      --k;
      s = intersection(q, v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    const double dq = q - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

constexpr int kGapFillPasses = 8;  // a gap up to 16 cells across is filled

}  // namespace

bool parse_fill_below(const std::string & name, FillBelow & fill)
{
  if (name == "none") {
    fill = FillBelow::None;
  } else if (name == "lowest") {
    fill = FillBelow::Lowest;
  } else if (name == "highest") {
    fill = FillBelow::Highest;
  } else {
    return false;
  }
  return true;
}

bool ClearanceGrid::build(
  const std::vector<Eigen::Vector3d> & obstacles, double resolution, double margin,
  double z_min, double z_max, FillBelow fill, std::size_t max_cells)
{
  dist_.clear();
  if (obstacles.empty() || !(resolution > 0.0)) {
    return false;
  }
  Eigen::Vector3d lo = obstacles.front(), hi = obstacles.front();
  for (const auto & p : obstacles) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  lo.array() -= margin;
  hi.array() += margin;
  if (std::isfinite(z_min)) {lo.z() = z_min;}
  if (std::isfinite(z_max)) {hi.z() = z_max;}
  if (hi.z() <= lo.z()) {
    return false;
  }
  res_ = resolution;
  origin_ = lo;
  nx_ = static_cast<int>(std::ceil((hi.x() - lo.x()) / res_));
  ny_ = static_cast<int>(std::ceil((hi.y() - lo.y()) / res_));
  nz_ = static_cast<int>(std::ceil((hi.z() - lo.z()) / res_));
  const std::size_t n = static_cast<std::size_t>(nx_) * ny_ * nz_;
  if (nx_ <= 0 || ny_ <= 0 || nz_ <= 0 || n > max_cells) {
    nx_ = ny_ = nz_ = 0;
    return false;
  }

  std::vector<double> sq(n, kInf);
  // The height of each column (cell index), for fill; -1: no points in it
  std::vector<int> top(static_cast<std::size_t>(nx_) * ny_, -1);
  for (const auto & p : obstacles) {
    int x, y, z;
    if (!cell_of(p, x, y, z)) {
      // Below the grid (e.g. z_min above the ground) still counts as the column's bottom
      if (fill == FillBelow::None || x < 0 || y < 0 || x >= nx_ || y >= ny_ || z >= nz_) {
        continue;
      }
      z = -1;
    } else {
      sq[index(x, y, z)] = 0.0;
    }
    int & t = top[static_cast<std::size_t>(y) * nx_ + x];
    if (t < 0 && z < 0) {
      t = 0;  // a column whose only points are below the grid: solid at its bottom cell
    } else if (fill == FillBelow::Highest) {
      t = std::max(t, z);
    } else if (fill == FillBelow::Lowest) {
      t = t < 0 ? std::max(z, 0) : std::min(t, std::max(z, 0));
    }
  }
  if (fill != FillBelow::None) {
    // Gaps in the scans: the highest of the neighbouring columns', a few passes
    for (int pass = 0; pass < kGapFillPasses; ++pass) {
      std::vector<int> next = top;
      bool changed = false;
      for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
          if (top[static_cast<std::size_t>(y) * nx_ + x] >= 0) {continue;}
          int best = -1;
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              const int qx = x + dx, qy = y + dy;
              if (qx >= 0 && qy >= 0 && qx < nx_ && qy < ny_) {
                best = std::max(best, top[static_cast<std::size_t>(qy) * nx_ + qx]);
              }
            }
          }
          if (best >= 0) {
            next[static_cast<std::size_t>(y) * nx_ + x] = best;
            changed = true;
          }
        }
      }
      top.swap(next);
      if (!changed) {break;}
    }
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        for (int z = 0; z <= top[static_cast<std::size_t>(y) * nx_ + x]; ++z) {
          sq[index(x, y, z)] = 0.0;
        }
      }
    }
  }
  // Separable: along x, then y, then z
  const int longest = std::max({nx_, ny_, nz_});
  std::vector<double> f(longest), d(longest), zz(longest + 1);
  std::vector<int> v(longest);
  for (int z = 0; z < nz_; ++z) {
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        f[x] = sq[index(x, y, z)];
      }
      transform_1d(f, nx_, d, v, zz);
      for (int x = 0; x < nx_; ++x) {
        sq[index(x, y, z)] = d[x];
      }
    }
  }
  for (int z = 0; z < nz_; ++z) {
    for (int x = 0; x < nx_; ++x) {
      for (int y = 0; y < ny_; ++y) {
        f[y] = sq[index(x, y, z)];
      }
      transform_1d(f, ny_, d, v, zz);
      for (int y = 0; y < ny_; ++y) {
        sq[index(x, y, z)] = d[y];
      }
    }
  }
  for (int y = 0; y < ny_; ++y) {
    for (int x = 0; x < nx_; ++x) {
      for (int z = 0; z < nz_; ++z) {
        f[z] = sq[index(x, y, z)];
      }
      transform_1d(f, nz_, d, v, zz);
      for (int z = 0; z < nz_; ++z) {
        sq[index(x, y, z)] = d[z];
      }
    }
  }
  dist_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    dist_[i] = static_cast<float>(std::sqrt(std::min(sq[i], 1e12)) * res_);
  }
  return true;
}

bool ClearanceGrid::cell_of(const Eigen::Vector3d & p, int & x, int & y, int & z) const
{
  x = static_cast<int>(std::floor((p.x() - origin_.x()) / res_));
  y = static_cast<int>(std::floor((p.y() - origin_.y()) / res_));
  z = static_cast<int>(std::floor((p.z() - origin_.z()) / res_));
  return inside(x, y, z);
}

Eigen::Vector3d ClearanceGrid::center(int x, int y, int z) const
{
  return origin_ + res_ * Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5);
}

double ClearanceGrid::distance_at(const Eigen::Vector3d & p) const
{
  int x, y, z;
  return cell_of(p, x, y, z) ? dist_[index(x, y, z)] : 0.0;
}

std::vector<Eigen::Vector3d> smooth(
  const ClearanceGrid & grid, const std::vector<Eigen::Vector3d> & path, double step,
  double clearance, const SmootherParams & params)
{
  std::vector<Eigen::Vector3d> dense;
  if (path.empty() || !(step > 0.0)) {
    return path;
  }
  dense.push_back(path.front());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d d = path[i + 1] - path[i];
    const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / step)));
    for (int k = 1; k <= n; ++k) {
      dense.push_back(path[i] + d * (static_cast<double>(k) / n));
    }
  }
  const std::vector<Eigen::Vector3d> original = dense;
  for (int it = 0; it < params.iterations; ++it) {
    for (std::size_t i = 1; i + 1 < dense.size(); ++i) {
      const Eigen::Vector3d moved = dense[i] +
        params.smooth_weight * (0.5 * (dense[i - 1] + dense[i + 1]) - dense[i]) +
        params.data_weight * (original[i] - dense[i]);
      if (grid.distance_at(moved) >= clearance) {
        dense[i] = moved;
      }
    }
  }
  return dense;
}

bool segment_clear(
  const ClearanceGrid & grid, const Eigen::Vector3d & a, const Eigen::Vector3d & b,
  double clearance)
{
  const double len = (b - a).norm();
  const int steps = std::max(1, static_cast<int>(std::ceil(len / (0.5 * grid.resolution()))));
  for (int i = 0; i <= steps; ++i) {
    if (grid.distance_at(a + (b - a) * (static_cast<double>(i) / steps)) < clearance) {
      return false;
    }
  }
  return true;
}

namespace
{

/// The cell nearest to \p p with the clearance, within \p radius; false if none.
bool snap(
  const ClearanceGrid & grid, const Eigen::Vector3d & p, double clearance, double radius,
  int & cx, int & cy, int & cz)
{
  int x, y, z;
  grid.cell_of(p, x, y, z);
  if (grid.inside(x, y, z) && grid.distance(x, y, z) >= clearance) {
    cx = x;
    cy = y;
    cz = z;
    return true;
  }
  const int r = static_cast<int>(std::ceil(radius / grid.resolution()));
  double best = std::numeric_limits<double>::infinity();
  for (int dz = -r; dz <= r; ++dz) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int qx = x + dx, qy = y + dy, qz = z + dz;
        if (!grid.inside(qx, qy, qz) || grid.distance(qx, qy, qz) < clearance) {
          continue;
        }
        const double d = (grid.center(qx, qy, qz) - p).norm();
        if (d < best && d <= radius) {
          best = d;
          cx = qx;
          cy = qy;
          cz = qz;
        }
      }
    }
  }
  return std::isfinite(best);
}

}  // namespace

std::vector<Eigen::Vector3d> plan(
  const ClearanceGrid & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const PlannerParams & params, PlanStatus * status)
{
  std::vector<Eigen::Vector3d> path;
  PlanStatus dummy;
  PlanStatus & st = status ? *status : dummy;
  int sx, sy, sz, gx, gy, gz;
  if (grid.empty()) {
    st = PlanStatus::NoGrid;
    return path;
  }
  if (!snap(grid, start, params.min_clearance, params.snap_radius, sx, sy, sz)) {
    st = PlanStatus::StartBlocked;
    return path;
  }
  if (!snap(grid, goal, params.min_clearance, params.snap_radius, gx, gy, gz)) {
    st = PlanStatus::GoalBlocked;
    return path;
  }
  st = PlanStatus::Unreachable;
  const bool goal_free = grid.distance_at(goal) >= params.min_clearance;

  const std::size_t n = static_cast<std::size_t>(grid.nx()) * grid.ny() * grid.nz();
  std::vector<float> g(n, std::numeric_limits<float>::infinity());
  std::vector<int32_t> parent(n, -1);
  std::vector<uint8_t> closed(n, 0);
  using Entry = std::pair<float, int32_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  const double res = grid.resolution();
  auto h = [&](int x, int y, int z) {
      return static_cast<float>(res * std::sqrt(
               static_cast<double>((x - gx) * (x - gx) + (y - gy) * (y - gy) +
               (z - gz) * (z - gz))));
    };
  auto penalty = [&](double d) {
      if (d >= params.preferred_clearance || params.preferred_clearance <= 0.0) {return 0.0;}
      const double t = (params.preferred_clearance - d) / params.preferred_clearance;
      return params.clearance_weight * t * t;
    };

  const std::size_t s = grid.index(sx, sy, sz), t = grid.index(gx, gy, gz);
  g[s] = 0.0f;
  open.emplace(h(sx, sy, sz), static_cast<int32_t>(s));
  std::size_t expansions = 0;
  bool found = false;
  const int nx = grid.nx(), ny = grid.ny();
  while (!open.empty() && expansions < params.max_expansions) {
    const auto [f, ci] = open.top();
    open.pop();
    const std::size_t c = static_cast<std::size_t>(ci);
    if (closed[c]) {continue;}
    closed[c] = 1;
    ++expansions;
    if (c == t) {
      found = true;
      break;
    }
    const int cx = static_cast<int>(c % nx);
    const int cy = static_cast<int>((c / nx) % ny);
    const int cz = static_cast<int>(c / (static_cast<std::size_t>(nx) * ny));
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!dx && !dy && !dz) {continue;}
          const int qx = cx + dx, qy = cy + dy, qz = cz + dz;
          if (!grid.inside(qx, qy, qz)) {continue;}
          const std::size_t q = grid.index(qx, qy, qz);
          const double d = grid.distance(qx, qy, qz);
          if (closed[q] || d < params.min_clearance) {continue;}
          const double step = res * std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
          const float cost = g[c] + static_cast<float>(step * (1.0 + penalty(d)));
          if (cost < g[q]) {
            g[q] = cost;
            parent[q] = static_cast<int32_t>(c);
            open.emplace(cost + h(qx, qy, qz), static_cast<int32_t>(q));
          }
        }
      }
    }
  }
  if (!found) {
    return path;
  }
  st = PlanStatus::Found;

  std::vector<Eigen::Vector3d> cells;
  for (int32_t c = static_cast<int32_t>(t); c >= 0; c = parent[c]) {
    const std::size_t u = static_cast<std::size_t>(c);
    cells.push_back(
      grid.center(
        static_cast<int>(u % nx), static_cast<int>((u / nx) % ny),
        static_cast<int>(u / (static_cast<std::size_t>(nx) * ny))));
  }
  std::reverse(cells.begin(), cells.end());
  if (cells.size() == 1) {
    cells.push_back(cells.front());  // start and goal in one cell: a path from one to the other
  }
  cells.front() = grid.distance_at(start) >= params.min_clearance ? start : cells.front();
  if (goal_free) {
    cells.back() = goal;
  }

  // Straight segments where the clearance allows: as much as A*'s cells had along that stretch
  // (up to preferred_clearance), so that a shortcut does not undo the preference
  std::vector<double> kept(cells.size());
  for (std::size_t k = 0; k < cells.size(); ++k) {
    kept[k] = std::min(
      grid.distance_at(cells[k]),
      std::max(params.preferred_clearance, params.min_clearance));
  }
  path.push_back(cells.front());
  std::size_t i = 0;
  while (i + 1 < cells.size()) {
    std::size_t j = cells.size() - 1;
    while (j > i + 1) {
      const double needed = std::max(
        params.min_clearance,
        *std::min_element(kept.begin() + i, kept.begin() + j + 1) - 0.5 * grid.resolution());
      if (segment_clear(grid, cells[i], cells[j], needed)) {
        break;
      }
      --j;
    }
    path.push_back(cells[j]);
    i = j;
  }
  return path;
}

}  // namespace bonxai_planner
}  // namespace easynav
