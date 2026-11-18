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

/**
 * @file sensed_obstacles.hpp
 * @brief The obstacles the sensors have seen that the map does not have, as an occupancy memory
 *        by cells (one point each, on the ground under the obstacle): evidence in log-odds (a
 *        point above the ground is a hit, a ray from the robot through a cell is a miss), so that
 *        what moves away is cleared; forgotten when the robot is far, or unseen for a while.
 */

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__SENSED_OBSTACLES_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__SENSED_OBSTACLES_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"

namespace easynav
{
namespace bigmap
{

/// How the evidence accumulates (as OctoMap): probabilities of a hit and of a miss, the clamp.
struct MemoryParams
{
  double p_hit{0.7};
  double p_miss{0.4};
  double p_min{0.12};
  double p_max{0.97};
  bool clearing{true};      ///< rays from the robot to each point clear the cells they cross
  double decay_time{0.0};   ///< s: a cell not seen again for this long is forgotten; 0: never
};

class SensedObstacles
{
public:
  /// A ray of the sensor, on the map's surface: from the robot's ground to the ground under the
  /// point (`end`); `end_free`: the point is the ground itself, so its cell is free too.
  struct Ray
  {
    Eigen::Vector3f end;
    bool end_free;
  };

  /// Cells of `cell` m on the plane (1 m in height, so floors one over another stay apart)
  explicit SensedObstacles(double cell = 0.5, const MemoryParams & p = {})
  : cell_(cell), p_(p)
  {
    auto logodds = [](double p) {return static_cast<float>(std::log(p / (1.0 - p)));};
    l_hit_ = logodds(p_.p_hit);
    l_miss_ = logodds(p_.p_miss);
    l_min_ = logodds(p_.p_min);
    l_max_ = logodds(p_.p_max);
  }

  /// One cycle of evidence: `hits` (obstacles, on the ground under them) and, with clearing, the
  /// rays from `origin` (the robot's ground). Each cell gets one hit or one miss per cycle (a hit
  /// beats a miss); unknown cells crossed by a ray are not stored. True if the obstacles changed.
  bool update(
    const std::vector<Eigen::Vector3f> & hits, const Eigen::Vector3f & origin,
    const std::vector<Ray> & rays, double now)
  {
    bool changed = false;
    std::unordered_set<std::int64_t> hit;
    for (const auto & q : hits) {
      const auto k = key(q);
      if (!hit.insert(k).second) {continue;}
      auto [it, fresh] = cells_.try_emplace(k, Cell{q, 0.0f, now});
      const bool was = it->second.l > 0.0f;
      it->second.l = std::min(it->second.l + l_hit_, l_max_);
      it->second.q = q;
      it->second.seen = now;
      changed |= fresh || (!was && it->second.l > 0.0f);
    }
    if (p_.clearing && !cells_.empty()) {
      std::unordered_set<std::int64_t> missed;
      // a cell is crossed if the ray passes within half a metre of its height (the ray is
      // straight, the ground under it is not; floors are farther apart than that)
      for (const auto & r : rays) {
        walk(origin, r.end, r.end_free, [&](std::int64_t i, std::int64_t j, double z) {
            const auto k0 = static_cast<std::int64_t>(std::floor(z));
            for (std::int64_t k = k0 - 1; k <= k0 + 1; ++k) {
              const auto c = key(i, j, k);
              const auto it = cells_.find(c);
              if (it != cells_.end() && !hit.count(c) && std::abs(it->second.q.z() - z) <= 0.5) {
                missed.insert(c);
              }
            }
          });
      }
      for (const auto k : missed) {
        auto it = cells_.find(k);
        const bool was = it->second.l > 0.0f;
        it->second.l = std::max(it->second.l + l_miss_, l_min_);
        if (it->second.l <= 0.0f) {   // free now: not stored
          cells_.erase(it);
          changed |= was;
        }
      }
    }
    if (p_.decay_time > 0.0) {
      for (auto it = cells_.begin(); it != cells_.end(); ) {
        if (now - it->second.seen > p_.decay_time) {
          changed |= it->second.l > 0.0f;
          it = cells_.erase(it);
        } else {
          ++it;
        }
      }
    }
    return changed;
  }

  /// Forgets the obstacles farther than d (m, on the plane) from (x, y). How many.
  std::size_t forget_beyond(double x, double y, double d)
  {
    std::size_t n = 0;
    for (auto it = cells_.begin(); it != cells_.end(); ) {
      if (std::hypot(it->second.q.x() - x, it->second.q.y() - y) > d) {
        it = cells_.erase(it);
        ++n;
      } else {
        ++it;
      }
    }
    return n;
  }

  /// The obstacles: the cells more likely occupied than not
  std::vector<Eigen::Vector3f> points() const
  {
    std::vector<Eigen::Vector3f> out;
    out.reserve(cells_.size());
    for (const auto & kv : cells_) {
      if (kv.second.l > 0.0f) {out.push_back(kv.second.q);}
    }
    return out;
  }

  std::size_t size() const
  {
    return static_cast<std::size_t>(std::count_if(
             cells_.begin(), cells_.end(), [](const auto & kv) {return kv.second.l > 0.0f;}));
  }
  void clear() {cells_.clear();}
  const MemoryParams & params() const {return p_;}

private:
  struct Cell
  {
    Eigen::Vector3f q;   // the latest point seen in it
    float l;             // log-odds
    double seen;         // when it was last hit
  };

  std::int64_t key(const Eigen::Vector3f & q) const
  {
    return key(
      static_cast<std::int64_t>(std::floor(q.x() / cell_)),
      static_cast<std::int64_t>(std::floor(q.y() / cell_)),
      static_cast<std::int64_t>(std::floor(q.z())));
  }

  static std::int64_t key(std::int64_t i, std::int64_t j, std::int64_t k)
  {
    return ((i & 0x1fffff) << 42) | ((j & 0x1fffff) << 21) | (k & 0x1fffff);
  }

  /// The cells a segment crosses on the plane (Amanatides-Woo), f(i, j, the segment's height
  /// there); the last one only if `with_end`
  template<class F>
  void walk(const Eigen::Vector3f & a, const Eigen::Vector3f & b, bool with_end, F f) const
  {
    const double ax = a.x() / cell_, ay = a.y() / cell_, bx = b.x() / cell_, by = b.y() / cell_;
    std::int64_t i = static_cast<std::int64_t>(std::floor(ax));
    std::int64_t j = static_cast<std::int64_t>(std::floor(ay));
    const auto ie = static_cast<std::int64_t>(std::floor(bx));
    const auto je = static_cast<std::int64_t>(std::floor(by));
    const double dx = bx - ax, dy = by - ay;
    const int si = dx > 0 ? 1 : -1, sj = dy > 0 ? 1 : -1;
    const double inf = std::numeric_limits<double>::infinity();
    double tx = dx != 0.0 ? ((si > 0 ? i + 1 - ax : ax - i) / std::abs(dx)) : inf;
    double ty = dy != 0.0 ? ((sj > 0 ? j + 1 - ay : ay - j) / std::abs(dy)) : inf;
    const double ddx = dx != 0.0 ? 1.0 / std::abs(dx) : inf;
    const double ddy = dy != 0.0 ? 1.0 / std::abs(dy) : inf;
    const std::size_t steps =
      static_cast<std::size_t>(std::abs(ie - i) + std::abs(je - j));
    double t = 0.0;
    for (std::size_t n = 0; n <= steps; ++n) {
      const bool last = (i == ie && j == je);
      if (last && !with_end) {break;}
      const double z = a.z() + (b.z() - a.z()) * std::min(t, 1.0);
      f(i, j, z);
      if (last) {break;}
      if (tx < ty) {
        t = tx;
        tx += ddx;
        i += si;
      } else {
        t = ty;
        ty += ddy;
        j += sj;
      }
    }
  }

  double cell_;
  MemoryParams p_;
  float l_hit_, l_miss_, l_min_, l_max_;
  std::unordered_map<std::int64_t, Cell> cells_;
};

}  // namespace bigmap
}  // namespace easynav

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__SENSED_OBSTACLES_HPP_
