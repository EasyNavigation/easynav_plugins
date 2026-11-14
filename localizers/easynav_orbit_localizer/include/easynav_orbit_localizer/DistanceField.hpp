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
/// \brief Distance to the nearest obstacle of a map, with its gradient: the measurement model
/// of the localizer, on a 2D grid (Costmap2D) or a 3D point cloud (Bonxai's occupied voxels).

#ifndef EASYNAV_ORBIT_LOCALIZER__DISTANCEFIELD_HPP_
#define EASYNAV_ORBIT_LOCALIZER__DISTANCEFIELD_HPP_

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "easynav_costmap_common/costmap_2d.hpp"

namespace easynav
{
namespace orbit
{

/// @brief Distance (m) from a world point to the nearest obstacle of the map, and its gradient.
class DistanceField
{
public:
  virtual ~DistanceField() = default;

  /// @brief Distance at p; grad (unit vector away from the obstacle) if asked for. Infinity
  /// outside the known map.
  virtual double distance(const Eigen::Vector3d & p, Eigen::Vector3d * grad) const = 0;

  /// @brief The map's cell size (m).
  virtual double resolution() const = 0;

  /// @brief Whether p is inside the area the map knows.
  virtual bool inside(const Eigen::Vector3d & p) const = 0;
};

/// @brief Exact Euclidean distance transform of a Costmap2D's lethal cells, bilinear between
/// cells. Also the free cells and a ray caster, for the map's virtual views.
class GridDistanceField2D : public DistanceField
{
public:
  explicit GridDistanceField2D(const Costmap2D & map);

  double distance(const Eigen::Vector3d & p, Eigen::Vector3d * grad) const override;
  double resolution() const override {return res_;}
  bool inside(const Eigen::Vector3d & p) const override;

  /// @brief Whether the cell under (x, y) is free (known and not an obstacle).
  bool isFree(double x, double y) const;

  /// @brief Range (m) of a ray from (x, y) heading angle until an obstacle; max_range if nothing
  /// (an unknown cell ends the ray without a return too).
  double raycast(double x, double y, double angle, double max_range) const;

  unsigned int sizeX() const {return w_;}
  unsigned int sizeY() const {return h_;}
  double originX() const {return ox_;}
  double originY() const {return oy_;}

private:
  unsigned int w_ {0}, h_ {0};
  double res_ {0.05}, ox_ {0.0}, oy_ {0.0};
  std::vector<float> dist_;   // m, per cell
  std::vector<uint8_t> cost_;
  float at(int i, int j) const {return dist_[static_cast<std::size_t>(j) * w_ + i];}
};

/// @brief Distance to the nearest point of a cloud (a 3D map's occupied voxels), through a
/// k-d tree. Beyond max_distance the distance saturates and the gradient is zero.
class CloudDistanceField3D : public DistanceField
{
public:
  CloudDistanceField3D(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double resolution,
    double max_distance = 3.0);

  double distance(const Eigen::Vector3d & p, Eigen::Vector3d * grad) const override;
  double resolution() const override {return res_;}
  bool inside(const Eigen::Vector3d & p) const override;

  /// @brief The map's points within radius of p.
  std::vector<Eigen::Vector3d> pointsWithin(const Eigen::Vector3d & p, double radius) const;

  /// @brief Marches a ray through the occupied voxels: the first hit within max_range, if any.
  bool raycast(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction, double max_range,
    Eigen::Vector3d & hit) const;

  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud() const {return cloud_;}
  const Eigen::Vector3d & min() const {return min_;}
  const Eigen::Vector3d & max() const {return max_;}

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  double res_ {0.3};
  double max_distance_ {3.0};
  Eigen::Vector3d min_, max_;
  struct Voxel
  {
    Eigen::Vector3d sum {Eigen::Vector3d::Zero()};
    int n {0};
  };
  std::unordered_map<int64_t, Voxel> occupied_;   // voxels with points, for the ray casts
  int64_t voxelKey(const Eigen::Vector3d & p) const;
};

}  // namespace orbit
}  // namespace easynav

#endif  // EASYNAV_ORBIT_LOCALIZER__DISTANCEFIELD_HPP_
