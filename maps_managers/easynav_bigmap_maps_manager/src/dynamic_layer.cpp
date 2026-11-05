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

#include "easynav_bigmap_maps_manager/dynamic_layer.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace bigmap_maps_manager
{

TriangleLocator::TriangleLocator(const LodMesh & mesh, double cell)
: mesh_(&mesh), cell_(cell)
{
  double xmin = 1e300, ymin = 1e300, xmax = -1e300, ymax = -1e300;
  for (std::size_t v = 0; v < mesh.x.size(); ++v) {
    xmin = std::min<double>(xmin, mesh.x[v]); xmax = std::max<double>(xmax, mesh.x[v]);
    ymin = std::min<double>(ymin, mesh.y[v]); ymax = std::max<double>(ymax, mesh.y[v]);
  }
  x0_ = xmin;
  y0_ = ymin;
  nx_ = std::max(1, static_cast<int>(std::ceil((xmax - xmin) / cell_)) + 1);
  ny_ = std::max(1, static_cast<int>(std::ceil((ymax - ymin) / cell_)) + 1);
  bins_.resize(static_cast<std::size_t>(nx_) * ny_);
  for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
    const auto a = mesh.tri[3 * t], b = mesh.tri[3 * t + 1], c = mesh.tri[3 * t + 2];
    const double tx0 = std::min({mesh.x[a], mesh.x[b], mesh.x[c]});
    const double tx1 = std::max({mesh.x[a], mesh.x[b], mesh.x[c]});
    const double ty0 = std::min({mesh.y[a], mesh.y[b], mesh.y[c]});
    const double ty1 = std::max({mesh.y[a], mesh.y[b], mesh.y[c]});
    const int i0 = static_cast<int>((tx0 - x0_) / cell_),
      i1 = static_cast<int>((tx1 - x0_) / cell_);
    const int j0 = static_cast<int>((ty0 - y0_) / cell_),
      j1 = static_cast<int>((ty1 - y0_) / cell_);
    for (int j = j0; j <= std::min(j1, ny_ - 1); ++j) {
      for (int i = i0; i <= std::min(i1, nx_ - 1); ++i) {
        bins_[static_cast<std::size_t>(j) * nx_ + i].push_back(static_cast<std::uint32_t>(t));
      }
    }
  }
}

int TriangleLocator::locate(double x, double y) const
{
  const int i = static_cast<int>(std::floor((x - x0_) / cell_));
  const int j = static_cast<int>(std::floor((y - y0_) / cell_));
  if (i < 0 || j < 0 || i >= nx_ || j >= ny_) {return -1;}
  for (const auto t : bins_[static_cast<std::size_t>(j) * nx_ + i]) {
    const auto a = mesh_->tri[3 * t], b = mesh_->tri[3 * t + 1], c = mesh_->tri[3 * t + 2];
    const double x0 = mesh_->x[a], y0 = mesh_->y[a], x1 = mesh_->x[b], y1 = mesh_->y[b];
    const double x2 = mesh_->x[c], y2 = mesh_->y[c];
    const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    if (std::abs(det) < 1e-12) {continue;}
    const double l0 = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / det;
    const double l1 = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / det;
    if (l0 >= -1e-9 && l1 >= -1e-9 && 1.0 - l0 - l1 >= -1e-9) {return static_cast<int>(t);}
  }
  return -1;
}

namespace
{
double plane_z(const LodMesh & m, std::uint32_t t, double x, double y);
}  // namespace

int TriangleLocator::locate_at(double x, double y, double z) const
{
  if (!std::isfinite(z)) {return locate(x, y);}
  const int i = static_cast<int>(std::floor((x - x0_) / cell_));
  const int j = static_cast<int>(std::floor((y - y0_) / cell_));
  if (i < 0 || j < 0 || i >= nx_ || j >= ny_) {return -1;}
  int below = -1, above = -1;
  double below_z = -1e300, above_z = 1e300;
  for (const auto t : bins_[static_cast<std::size_t>(j) * nx_ + i]) {
    const auto a = mesh_->tri[3 * t], b = mesh_->tri[3 * t + 1], c = mesh_->tri[3 * t + 2];
    const double x0 = mesh_->x[a], y0 = mesh_->y[a], x1 = mesh_->x[b], y1 = mesh_->y[b];
    const double x2 = mesh_->x[c], y2 = mesh_->y[c];
    const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    if (std::abs(det) < 1e-12) {continue;}
    const double l0 = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / det;
    const double l1 = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / det;
    if (!(l0 >= -1e-9 && l1 >= -1e-9 && 1.0 - l0 - l1 >= -1e-9)) {continue;}
    const double pz = l0 * mesh_->z[a] + l1 * mesh_->z[b] + (1.0 - l0 - l1) * mesh_->z[c];
    if (pz <= z + 0.25) {
      if (pz > below_z) {below_z = pz; below = static_cast<int>(t);}
    } else if (pz < above_z) {
      above_z = pz;
      above = static_cast<int>(t);
    }
  }
  return below >= 0 ? below : above;
}

namespace
{
// Height of the plane of triangle t at (x, y), and the z component of its unit normal
double plane_z(const LodMesh & m, std::uint32_t t, double x, double y)
{
  const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
  const double det = (m.y[b] - m.y[c]) * (m.x[a] - m.x[c]) + (m.x[c] - m.x[b]) * (m.y[a] - m.y[c]);
  if (std::abs(det) < 1e-12) {return m.z[a];}
  const double l0 = ((m.y[b] - m.y[c]) * (x - m.x[c]) + (m.x[c] - m.x[b]) * (y - m.y[c])) / det;
  const double l1 = ((m.y[c] - m.y[a]) * (x - m.x[c]) + (m.x[a] - m.x[c]) * (y - m.y[c])) / det;
  return l0 * m.z[a] + l1 * m.z[b] + (1.0 - l0 - l1) * m.z[c];
}

double normal_z(const LodMesh & m, std::uint32_t t)
{
  const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
  const double ux = m.x[b] - m.x[a], uy = m.y[b] - m.y[a], uz = m.z[b] - m.z[a];
  const double vx = m.x[c] - m.x[a], vy = m.y[c] - m.y[a], vz = m.z[c] - m.z[a];
  const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
  return len < 1e-12 ? 1.0 : std::abs(nz) / len;
}
}  // namespace

bool DynamicLayer::ingest(
  const std::vector<std::array<float, 3>> & points, double robot_x, double robot_y,
  double robot_yaw, double now, const Surface & terrain, const DynamicParams & p,
  const LodMesh * mesh, const TriangleLocator * locator)
{
  return ingest(
    points, robot_x, robot_y,
    std::array<double, 4>{0.0, 0.0, std::sin(robot_yaw / 2.0), std::cos(robot_yaw / 2.0)}, now,
    terrain, p, mesh, locator);
}

bool DynamicLayer::ingest(
  const std::vector<std::array<float, 3>> & points, double robot_x, double robot_y,
  const std::array<double, 4> & q_xyzw, double now, const Surface & terrain,
  const DynamicParams & p, const LodMesh * mesh, const TriangleLocator * locator,
  double robot_z)
{
  // rotation matrix of the robot's orientation (body -> map)
  double qx = q_xyzw[0], qy = q_xyzw[1], qz = q_xyzw[2], qw = q_xyzw[3];
  const double qn = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (qn < 1e-9) {qx = qy = qz = 0.0; qw = 1.0;} else {qx /= qn; qy /= qn; qz /= qn; qw /= qn;}
  const double r00 = 1 - 2 * (qy * qy + qz * qz), r01 = 2 * (qx * qy - qz * qw);
  const double r02 = 2 * (qx * qz + qy * qw);
  const double r10 = 2 * (qx * qy + qz * qw), r11 = 1 - 2 * (qx * qx + qz * qz);
  const double r12 = 2 * (qy * qz - qx * qw);
  const double r20 = 2 * (qx * qz - qy * qw), r21 = 2 * (qy * qz + qx * qw);
  const double r22 = 1 - 2 * (qx * qx + qy * qy);

  const bool planes = mesh && locator;
  double ground = terrain(robot_x, robot_y);  // terrain under the robot: the heights' reference
  if (planes) {
    const int rt = locator->locate_at(robot_x, robot_y, robot_z);
    if (rt >= 0) {ground = plane_z(*mesh, static_cast<std::uint32_t>(rt), robot_x, robot_y);}
  }

  // 1) Put every point in the map and group them by cell: an obstacle is told from ground that
  //    the terrain model puts too low by its vertical extent, which needs all the points.
  struct Cell
  {
    double x, y, zmin, zmax;                // map frame; x, y of the highest point
    double bx, by, bz_min, bz_max, bx_lo, by_lo;  // body frame (z of the lowest / highest point)
  };
  std::unordered_map<std::int64_t, Cell> cells;
  for (const auto & q : points) {
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2])) {continue;}
    if (std::hypot(q[0], q[1]) > p.max_range || q[2] > p.max_height) {continue;}
    // the point in the map: the robot's ground contact plus the point rotated by its orientation
    const double wx = robot_x + r00 * q[0] + r01 * q[1] + r02 * q[2];
    const double wy = robot_y + r10 * q[0] + r11 * q[1] + r12 * q[2];
    const double wz = ground + r20 * q[0] + r21 * q[1] + r22 * q[2];
    const std::int64_t key =
      static_cast<std::int64_t>(  // shifted as unsigned: negative indices are fine
      static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(wx / p.cell))) << 32) ^
      (static_cast<std::int64_t>(std::floor(wy / p.cell)) & 0xffffffffLL);
    auto it = cells.find(key);
    if (it == cells.end()) {
      cells[key] = {wx, wy, wz, wz, q[0], q[1], q[2], q[2], q[0], q[1]};
    } else {
      Cell & cc = it->second;
      if (wz > cc.zmax) {cc.zmax = wz; cc.x = wx; cc.y = wy; cc.bx = q[0]; cc.by = q[1];}
      cc.zmin = std::min(cc.zmin, wz);
      if (q[2] > cc.bz_max) {cc.bz_max = q[2];}
      if (q[2] < cc.bz_min) {cc.bz_min = q[2]; cc.bx_lo = q[0]; cc.by_lo = q[1];}
    }
  }

  // 1b) Local ground plane in the window ahead, from the lowest point of each cell
  //     (z = a x + b y + c, robot frame). RANSAC over 3 cells, tilt limited; the best plane is
  //     refitted on its inliers.
  struct Plane
  {
    double a{0}, b{0}, c{0};
    bool ok{false};
  } gp;
  auto in_window = [&p](double bx, double by) {
      return bx >= 0.0 && bx <= p.window_length && std::abs(by) <= 0.5 * p.window_width;
    };
  if (p.ground_fit) {
    std::vector<std::array<double, 3>> low;
    for (const auto & kv : cells) {
      const Cell & cc = kv.second;
      if (in_window(cc.bx_lo, cc.by_lo) && cc.bz_min >= p.window_z_min &&
        cc.bz_min <= p.window_z_max)
      {
        low.push_back({cc.bx_lo, cc.by_lo, cc.bz_min});
      }
    }
    if (static_cast<int>(low.size()) >= p.ground_min_cells) {
      const double max_tilt = std::tan(p.max_ground_tilt_deg * M_PI / 180.0);
      std::uint32_t seed = 12345u;  // deterministic
      auto rnd = [&seed, &low]() {
          seed = seed * 1664525u + 1013904223u;
          return static_cast<std::size_t>(seed >> 8) % low.size();
        };
      std::size_t best = 0;
      Plane bp;
      for (int it = 0; it < 80; ++it) {
        const auto & P0 = low[rnd()], & P1 = low[rnd()], & P2 = low[rnd()];
        const double ux = P1[0] - P0[0], uy = P1[1] - P0[1], uz = P1[2] - P0[2];
        const double vx = P2[0] - P0[0], vy = P2[1] - P0[1], vz = P2[2] - P0[2];
        const double nz = ux * vy - uy * vx;
        if (std::abs(nz) < 1e-6) {continue;}
        const double a = -(uy * vz - uz * vy) / nz, b = -(uz * vx - ux * vz) / nz;
        if (std::hypot(a, b) > max_tilt) {continue;}
        const double c0 = P0[2] - a * P0[0] - b * P0[1];
        std::size_t n = 0;
        for (const auto & q : low) {
          if (std::abs(q[2] - (a * q[0] + b * q[1] + c0)) <= p.ground_tolerance) {++n;}
        }
        if (n > best) {best = n; bp = {a, b, c0, true};}
      }
      if (bp.ok && static_cast<int>(best) >= p.ground_min_cells) {
        // least squares on the inliers (normal equations, 3x3)
        double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, sn = 0, sxz = 0, syz = 0, sz = 0;
        for (const auto & q : low) {
          if (std::abs(q[2] - (bp.a * q[0] + bp.b * q[1] + bp.c)) > p.ground_tolerance) {continue;}
          sxx += q[0] * q[0]; sxy += q[0] * q[1]; sx += q[0]; syy += q[1] * q[1]; sy += q[1];
          sn += 1; sxz += q[0] * q[2]; syz += q[1] * q[2]; sz += q[2];
        }
        const double m[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, sn}};
        const double rhs[3] = {sxz, syz, sz};
        auto det3 = [](const double A[3][3]) {
            return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                   A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                   A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
          };
        const double d = det3(m);
        if (std::abs(d) > 1e-9) {
          double sol[3];
          for (int k = 0; k < 3; ++k) {
            double mk[3][3];
            for (int r = 0; r < 3; ++r) {
              for (int cc = 0; cc < 3; ++cc) {
                mk[r][cc] = (cc == k) ? rhs[r] : m[r][cc];
              }
            }
            sol[k] = det3(mk) / d;
          }
          if (std::hypot(sol[0], sol[1]) <= max_tilt) {bp = {sol[0], sol[1], sol[2], true};}
        }
        gp = bp;
      }
    }
  }

  // 2) A cell is an obstacle if its highest point is at least min_height above the plane of the
  //    NavCel it falls in (along the normal) and its points span at least min_vertical_extent
  bool changed = false;
  for (const auto & kv : cells) {
    const Cell & c = kv.second;
    if (c.zmax - c.zmin < p.min_vertical_extent) {continue;}
    // the sensor's own ground plane, where there is one: the highest point has to rise above it
    // (along its normal) too, or the cell is just the slope the robot is looking at
    if (gp.ok && in_window(c.bx, c.by)) {
      const double norm = std::sqrt(1.0 + gp.a * gp.a + gp.b * gp.b);
      const double above = (c.bz_max - (gp.a * c.bx + gp.b * c.by + gp.c)) / norm;
      if (above < p.min_height) {continue;}
    }
    double surface, cos_n = 1.0;
    if (planes) {
      const int t = locator->locate_at(c.x, c.y, c.zmax);
      if (t < 0) {continue;}
      surface = plane_z(*mesh, static_cast<std::uint32_t>(t), c.x, c.y);
      cos_n = normal_z(*mesh, static_cast<std::uint32_t>(t));
    } else {
      surface = terrain(c.x, c.y);
    }
    // distance to the NavCel's plane along its normal = vertical offset * cos(slope)
    if ((c.zmax - surface) * cos_n < p.min_height) {continue;}
    auto it = entries_.find(kv.first);
    if (it == entries_.end()) {
      entries_[kv.first] = {static_cast<float>(c.x), static_cast<float>(c.y),
        static_cast<float>(c.zmax), static_cast<float>(surface), now, now, p.confirm_time <= 0.0,
        now, 0};
      changed = changed || p.confirm_time <= 0.0;
    } else {
      if (c.zmax > it->second.z + 0.05 && it->second.confirmed) {changed = true;}
      it->second.z = std::max(it->second.z, static_cast<float>(c.zmax));
      constexpr double kStreakGap = 1.0;   // s: a longer break starts the run again
      if (now - it->second.stamp > kStreakGap) {
        it->second.streak = now;
        it->second.periods = 0;
      }
      it->second.stamp = now;
    }
  }
  return changed;
}

bool DynamicLayer::expire(double now, const DynamicParams & p)
{
  // s: an unconfirmed detection that is gone was a glitch
  constexpr double kUnconfirmedTimeout = 0.5;
  bool changed = false;
  for (auto it = entries_.begin(); it != entries_.end(); ) {
    Entry & e = it->second;
    const double unseen = now - e.stamp;
    if (unseen > p.forget_time || (!e.confirmed && unseen > kUnconfirmedTimeout)) {
      changed = changed || e.confirmed;
      it = entries_.erase(it);
      continue;
    }
    if (!e.confirmed && e.stamp - e.first >= p.confirm_time) {
      e.confirmed = true;
      changed = true;
    }
    ++it;
  }
  return changed;
}

std::vector<SustainedHit> DynamicLayer::sustained(double now, double min_time)
{
  std::vector<SustainedHit> out;
  if (min_time <= 0.0) {return out;}
  for (auto & kv : entries_) {
    Entry & e = kv.second;
    if (!e.confirmed || now - e.stamp > 1.0) {continue;}   // still being seen
    const int due = static_cast<int>((e.stamp - e.streak) / min_time);
    if (due > e.periods) {
      e.periods = due;
      out.push_back({e.x, e.y, e.z, e.surface});
    }
  }
  return out;
}

std::vector<RaisedTile> DynamicLayer::tiles(
  const LodMesh & mesh, const TriangleLocator & locator, double min_age) const
{
  std::map<std::uint32_t, float> top;
  for (const auto & kv : entries_) {
    if (!kv.second.confirmed) {continue;}
    if (min_age > 0.0 && kv.second.stamp - kv.second.streak < min_age) {continue;}
    const int t = locator.locate_at(kv.second.x, kv.second.y, kv.second.surface);
    if (t < 0) {continue;}
    // (a map's default float is 0, which would clip every height below the DEM's reference)
    const auto it = top.emplace(static_cast<std::uint32_t>(t), kv.second.z);
    if (!it.second) {it.first->second = std::max(it.first->second, kv.second.z);}
  }
  std::vector<RaisedTile> out;
  for (const auto & kv : top) {
    const auto t = kv.first;
    const auto a = mesh.tri[3 * t], b = mesh.tri[3 * t + 1], c = mesh.tri[3 * t + 2];
    const float highest = std::max({mesh.z[a], mesh.z[b], mesh.z[c]});
    const float mean = (mesh.z[a] + mesh.z[b] + mesh.z[c]) / 3.0f;
    const float z_top = std::max(kv.second, highest);
    out.push_back({t, z_top, z_top - mean});
  }
  return out;
}

}  // namespace bigmap_maps_manager
