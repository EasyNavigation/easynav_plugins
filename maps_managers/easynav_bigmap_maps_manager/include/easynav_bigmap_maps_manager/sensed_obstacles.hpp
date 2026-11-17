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
 * @brief The obstacles the sensors have seen that the map does not have: remembered by cells
 *        (one point each, on the ground under the obstacle) until the robot is far from them.
 */

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__SENSED_OBSTACLES_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__SENSED_OBSTACLES_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"

namespace easynav
{
namespace bigmap
{

class SensedObstacles
{
public:
  /// Cells of `cell` m on the plane (1 m in height, so floors one over another stay apart)
  explicit SensedObstacles(double cell = 0.5)
  : cell_(cell) {}

  /// Remembers an obstacle at q (on the ground under it). True if its cell is new.
  bool add(const Eigen::Vector3f & q)
  {
    return cells_.emplace(key(q), q).second;
  }

  /// Forgets the obstacles farther than d (m, on the plane) from (x, y). How many.
  std::size_t forget_beyond(double x, double y, double d)
  {
    std::size_t n = 0;
    for (auto it = cells_.begin(); it != cells_.end(); ) {
      if (std::hypot(it->second.x() - x, it->second.y() - y) > d) {
        it = cells_.erase(it);
        ++n;
      } else {
        ++it;
      }
    }
    return n;
  }

  std::vector<Eigen::Vector3f> points() const
  {
    std::vector<Eigen::Vector3f> out;
    out.reserve(cells_.size());
    for (const auto & kv : cells_) {
      out.push_back(kv.second);
    }
    return out;
  }

  std::size_t size() const {return cells_.size();}
  void clear() {cells_.clear();}

private:
  std::int64_t key(const Eigen::Vector3f & q) const
  {
    const auto i = static_cast<std::int64_t>(std::floor(q.x() / cell_)) & 0x1fffff;
    const auto j = static_cast<std::int64_t>(std::floor(q.y() / cell_)) & 0x1fffff;
    const auto k = static_cast<std::int64_t>(std::floor(q.z())) & 0x1fffff;
    return (i << 42) | (j << 21) | k;
  }

  double cell_;
  std::unordered_map<std::int64_t, Eigen::Vector3f> cells_;
};

}  // namespace bigmap
}  // namespace easynav

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__SENSED_OBSTACLES_HPP_
