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

/// \file
/// \brief The perceived obstacle points and the robot's shape: collisions, nearest points and the
/// robot's extent towards a point.

#ifndef EASYNAV_MPC_CONTROLLER__OBSTACLES_HPP_
#define EASYNAV_MPC_CONTROLLER__OBSTACLES_HPP_

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"

namespace easynav
{
namespace mpc
{

/// @brief The robot's shape around its frame: a polygon (x, y) or, if empty, a circle of
/// \p radius. An aerial robot is a sphere of \p radius.
struct RobotShape
{
  std::vector<Eigen::Vector2d> footprint;
  double radius {0.3};

  /// Distance from the robot's frame to its edge in direction \p dir (robot frame, unit).
  double extent(const Eigen::Vector2d & dir) const;
  /// Whether (x, y), in the robot frame, is inside the robot grown by \p margin.
  bool contains(const Eigen::Vector2d & p, double margin) const;
};

/// @brief Obstacle points in the map frame, bucketed for neighbourhood queries.
class Obstacles
{
public:
  /// @param use_z whether distances are 3D (aerial robots) or in the plane (ground robots,
  /// whose points are already filtered by height).
  void set(const std::vector<Eigen::Vector3d> & points, bool use_z, double cell = 0.5);

  bool empty() const {return points_.empty();}
  std::size_t size() const {return points_.size();}
  const Eigen::Vector3d & point(int i) const {return points_[i];}
  bool use_z() const {return use_z_;}

  /// Indices of the points within \p radius of \p center, nearest first.
  std::vector<int> near(const Eigen::Vector3d & center, double radius) const;

  /// Whether the robot at (\p position, \p yaw), grown by \p margin, touches a point.
  bool collides(
    const RobotShape & shape, const Eigen::Vector3d & position, double yaw, double margin) const;

private:
  struct Hash
  {
    std::size_t operator()(const std::pair<int, int> & k) const
    {
      return std::hash<int64_t>()(
        (static_cast<int64_t>(k.first) << 32) ^ static_cast<uint32_t>(k.second));
    }
  };

  std::pair<int, int> key(double x, double y) const;

  std::vector<Eigen::Vector3d> points_;
  std::unordered_map<std::pair<int, int>, std::vector<int>, Hash> buckets_;
  double cell_ {0.5};
  bool use_z_ {false};
};

}  // namespace mpc
}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__OBSTACLES_HPP_
