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

#include "easynav_mpc_controller/Obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace easynav
{
namespace mpc
{

double RobotShape::extent(const Eigen::Vector2d & dir) const
{
  if (footprint.size() < 3) {
    return radius;
  }
  // Where the ray from the frame in dir leaves the polygon
  double best = 0.0;
  const std::size_t n = footprint.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d a = footprint[i], b = footprint[(i + 1) % n];
    const Eigen::Vector2d e = b - a;
    const double den = dir.x() * e.y() - dir.y() * e.x();
    if (std::abs(den) < 1e-12) {
      continue;
    }
    const double t = (a.x() * e.y() - a.y() * e.x()) / den;          // along dir
    const double s = (a.x() * dir.y() - a.y() * dir.x()) / den;      // along the edge
    if (t > 0.0 && s >= -1e-9 && s <= 1.0 + 1e-9) {
      best = std::max(best, t);
    }
  }
  return best > 0.0 ? best : radius;
}

bool RobotShape::contains(const Eigen::Vector2d & p, double margin) const
{
  if (footprint.size() < 3) {
    return p.norm() <= radius + margin;
  }
  // Inside the polygon, or within margin of an edge
  bool inside = false;
  double min_dist = std::numeric_limits<double>::infinity();
  const std::size_t n = footprint.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Eigen::Vector2d & a = footprint[i];
    const Eigen::Vector2d & b = footprint[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
      p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
    {
      inside = !inside;
    }
    const Eigen::Vector2d e = b - a;
    const double t = std::clamp((p - a).dot(e) / std::max(e.squaredNorm(), 1e-12), 0.0, 1.0);
    min_dist = std::min(min_dist, (p - (a + t * e)).norm());
  }
  return inside || min_dist <= margin;
}

std::pair<int, int> Obstacles::key(double x, double y) const
{
  return {static_cast<int>(std::floor(x / cell_)), static_cast<int>(std::floor(y / cell_))};
}

void Obstacles::set(const std::vector<Eigen::Vector3d> & points, bool use_z, double cell)
{
  points_ = points;
  use_z_ = use_z;
  cell_ = cell;
  buckets_.clear();
  for (std::size_t i = 0; i < points_.size(); ++i) {
    buckets_[key(points_[i].x(), points_[i].y())].push_back(static_cast<int>(i));
  }
}

std::vector<int> Obstacles::near(const Eigen::Vector3d & c, double radius) const
{
  std::vector<std::pair<double, int>> found;
  const auto lo = key(c.x() - radius, c.y() - radius);
  const auto hi = key(c.x() + radius, c.y() + radius);
  for (int bx = lo.first; bx <= hi.first; ++bx) {
    for (int by = lo.second; by <= hi.second; ++by) {
      const auto it = buckets_.find({bx, by});
      if (it == buckets_.end()) {
        continue;
      }
      for (int i : it->second) {
        const Eigen::Vector3d d = points_[i] - c;
        const double dist = use_z_ ? d.norm() : d.head<2>().norm();
        if (dist <= radius) {
          found.emplace_back(dist, i);
        }
      }
    }
  }
  std::sort(found.begin(), found.end());
  std::vector<int> out;
  out.reserve(found.size());
  for (const auto & f : found) {
    out.push_back(f.second);
  }
  return out;
}

bool Obstacles::collides(
  const RobotShape & shape, const Eigen::Vector3d & position, double yaw, double margin) const
{
  const double reach = (shape.footprint.size() < 3 ? shape.radius : [&shape] {
        double r = 0.0;
        for (const auto & v : shape.footprint) {
          r = std::max(r, v.norm());
        }
        return r;
      }()) + margin;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int i : near(position, reach)) {
    const Eigen::Vector3d d = points_[i] - position;
    if (use_z_) {
      if (d.norm() <= shape.radius + margin) {
        return true;
      }
      continue;
    }
    const Eigen::Vector2d local(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
    if (shape.contains(local, margin)) {
      return true;
    }
  }
  return false;
}

}  // namespace mpc
}  // namespace easynav
