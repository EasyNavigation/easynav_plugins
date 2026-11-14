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

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_orbit_localizer/DistanceField.hpp"

namespace easynav
{
namespace orbit
{

namespace
{

// Felzenszwalb-Huttenlocher 1D squared distance transform of f (BIG where no obstacle).
constexpr double BIG = 1e20;

void edt1d(const double * f, double * d, int n, std::vector<int> & v, std::vector<double> & z)
{
  const double inf = std::numeric_limits<double>::infinity();
  v.assign(n, 0);
  z.assign(n + 1, 0.0f);
  int k = 0;
  z[0] = -inf;
  z[1] = inf;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + static_cast<double>(q) * q) -
      (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
      (2.0 * (q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + static_cast<double>(q) * q) - (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
        (2.0 * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = inf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) {++k;}
    d[q] = static_cast<double>(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

GridDistanceField2D::GridDistanceField2D(const Costmap2D & map)
: w_(map.getSizeInCellsX()), h_(map.getSizeInCellsY()), res_(map.getResolution()),
  ox_(map.getOriginX()), oy_(map.getOriginY())
{
  const float inf = std::numeric_limits<float>::infinity();
  const std::size_t n = static_cast<std::size_t>(w_) * h_;
  cost_.resize(n);
  std::vector<double> sq(n, BIG);
  for (unsigned int j = 0; j < h_; ++j) {
    for (unsigned int i = 0; i < w_; ++i) {
      const uint8_t c = map.getCost(i, j);
      cost_[static_cast<std::size_t>(j) * w_ + i] = c;
      if (c == LETHAL_OBSTACLE) {sq[static_cast<std::size_t>(j) * w_ + i] = 0.0;}
    }
  }
  // Separable exact EDT: columns, then rows
  std::vector<int> v;
  std::vector<double> z, f(std::max(w_, h_)), d(std::max(w_, h_));
  for (unsigned int i = 0; i < w_; ++i) {
    for (unsigned int j = 0; j < h_; ++j) {
      f[j] = sq[static_cast<std::size_t>(j) * w_ + i];
    }
    edt1d(f.data(), d.data(), static_cast<int>(h_), v, z);
    for (unsigned int j = 0; j < h_; ++j) {
      sq[static_cast<std::size_t>(j) * w_ + i] = d[j];
    }
  }
  for (unsigned int j = 0; j < h_; ++j) {
    double * row = &sq[static_cast<std::size_t>(j) * w_];
    edt1d(row, d.data(), static_cast<int>(w_), v, z);
    std::copy(d.begin(), d.begin() + w_, row);
  }
  dist_.resize(n);
  for (std::size_t k = 0; k < n; ++k) {
    dist_[k] = sq[k] < BIG ? std::sqrt(sq[k]) * static_cast<float>(res_) : inf;
  }
}

bool GridDistanceField2D::inside(const Eigen::Vector3d & p) const
{
  // One cell of margin: a return on a wall at the map's edge lands a little outside
  const double gx = (p.x() - ox_) / res_, gy = (p.y() - oy_) / res_;
  return gx >= -1.0 && gy >= -1.0 && gx < w_ + 1.0 && gy < h_ + 1.0;
}

bool GridDistanceField2D::isFree(double x, double y) const
{
  const int i = static_cast<int>(std::floor((x - ox_) / res_));
  const int j = static_cast<int>(std::floor((y - oy_) / res_));
  if (i < 0 || j < 0 || i >= static_cast<int>(w_) || j >= static_cast<int>(h_)) {return false;}
  return cost_[static_cast<std::size_t>(j) * w_ + i] == FREE_SPACE;
}

double GridDistanceField2D::distance(const Eigen::Vector3d & p, Eigen::Vector3d * grad) const
{
  // Bilinear between the centres of the four cells around p; the outer half cells take the
  // border's values
  if (!inside(p) || w_ < 2 || h_ < 2) {
    if (grad) {grad->setZero();}
    return std::numeric_limits<double>::infinity();
  }
  const double gx = std::min(std::max((p.x() - ox_) / res_ - 0.5, 0.0), w_ - 1.0 - 1e-9);
  const double gy = std::min(std::max((p.y() - oy_) / res_ - 0.5, 0.0), h_ - 1.0 - 1e-9);
  const int i = static_cast<int>(gx), j = static_cast<int>(gy);
  const double fx = gx - i, fy = gy - j;
  const double d00 = at(i, j), d10 = at(i + 1, j), d01 = at(i, j + 1), d11 = at(i + 1, j + 1);
  if (!std::isfinite(d00 + d10 + d01 + d11)) {
    if (grad) {grad->setZero();}
    return std::numeric_limits<double>::infinity();
  }
  const double d = (1 - fx) * (1 - fy) * d00 + fx * (1 - fy) * d10 + (1 - fx) * fy * d01 +
    fx * fy * d11;
  if (grad) {
    const double dx = ((1 - fy) * (d10 - d00) + fy * (d11 - d01)) / res_;
    const double dy = ((1 - fx) * (d01 - d00) + fx * (d11 - d10)) / res_;
    *grad = Eigen::Vector3d(dx, dy, 0.0);
  }
  return d;
}

double GridDistanceField2D::raycast(double x, double y, double angle, double max_range) const
{
  // Steps of half a cell; an unknown cell ends the ray without a return
  const double step = 0.5 * res_;
  const double cx = std::cos(angle), sy = std::sin(angle);
  for (double r = step; r <= max_range; r += step) {
    const double px = x + r * cx, py = y + r * sy;
    const int i = static_cast<int>(std::floor((px - ox_) / res_));
    const int j = static_cast<int>(std::floor((py - oy_) / res_));
    if (i < 0 || j < 0 || i >= static_cast<int>(w_) || j >= static_cast<int>(h_)) {
      return max_range;
    }
    const uint8_t c = cost_[static_cast<std::size_t>(j) * w_ + i];
    if (c == LETHAL_OBSTACLE) {return r;}
    if (c == NO_INFORMATION) {return max_range;}
  }
  return max_range;
}

CloudDistanceField3D::CloudDistanceField3D(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double resolution, double max_distance)
: cloud_(cloud), res_(resolution), max_distance_(max_distance)
{
  min_.setConstant(std::numeric_limits<double>::infinity());
  max_.setConstant(-std::numeric_limits<double>::infinity());
  for (const auto & p : cloud_->points) {
    min_ = min_.cwiseMin(Eigen::Vector3d(p.x, p.y, p.z));
    max_ = max_.cwiseMax(Eigen::Vector3d(p.x, p.y, p.z));
  }
  if (!cloud_->points.empty()) {tree_.setInputCloud(cloud_);}
  occupied_.reserve(cloud_->points.size());
  for (const auto & p : cloud_->points) {
    auto & v = occupied_[voxelKey(Eigen::Vector3d(p.x, p.y, p.z))];
    v.sum += Eigen::Vector3d(p.x, p.y, p.z);
    ++v.n;
  }
}

int64_t CloudDistanceField3D::voxelKey(const Eigen::Vector3d & p) const
{
  const int64_t i = static_cast<int64_t>(std::floor(p.x() / res_));
  const int64_t j = static_cast<int64_t>(std::floor(p.y() / res_));
  const int64_t k = static_cast<int64_t>(std::floor(p.z() / res_));
  return (i & 0x1FFFFF) | ((j & 0x1FFFFF) << 21) | ((k & 0x1FFFFF) << 42);
}

bool CloudDistanceField3D::raycast(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & direction, double max_range,
  Eigen::Vector3d & hit) const
{
  if (occupied_.empty()) {return false;}
  const Eigen::Vector3d d = direction.normalized();
  const double step = 0.5 * res_;
  int64_t last = std::numeric_limits<int64_t>::min();
  for (double r = step; r <= max_range; r += step) {
    const Eigen::Vector3d p = origin + r * d;
    const int64_t key = voxelKey(p);
    if (key == last) {continue;}
    last = key;
    const auto it = occupied_.find(key);
    if (it != occupied_.end()) {
      hit = it->second.sum / it->second.n;   // the map's own point there, not the voxel's face
      return true;
    }
    // Out of the map's box and heading away: nothing left to hit
    if ((p.array() < min_.array() && d.array() < 0.0).any() ||
      (p.array() > max_.array() && d.array() > 0.0).any())
    {
      return false;
    }
  }
  return false;
}

bool CloudDistanceField3D::inside(const Eigen::Vector3d & p) const
{
  const double m = max_distance_;
  return (p.array() >= (min_.array() - m)).all() && (p.array() <= (max_.array() + m)).all();
}

double CloudDistanceField3D::distance(const Eigen::Vector3d & p, Eigen::Vector3d * grad) const
{
  if (cloud_->points.empty() || !inside(p)) {
    if (grad) {grad->setZero();}
    return std::numeric_limits<double>::infinity();
  }
  pcl::PointXYZ q(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
  std::vector<int> idx(1);
  std::vector<float> sq(1);
  if (tree_.nearestKSearch(q, 1, idx, sq) < 1) {
    if (grad) {grad->setZero();}
    return std::numeric_limits<double>::infinity();
  }
  const auto & n = cloud_->points[static_cast<std::size_t>(idx[0])];
  const Eigen::Vector3d diff = p - Eigen::Vector3d(n.x, n.y, n.z);
  const double d = diff.norm();
  if (d >= max_distance_) {
    if (grad) {grad->setZero();}
    return max_distance_;
  }
  if (grad) {*grad = d > 1e-9 ? (diff / d).eval() : Eigen::Vector3d::Zero();}
  return d;
}

std::vector<Eigen::Vector3d> CloudDistanceField3D::pointsWithin(
  const Eigen::Vector3d & p, double radius) const
{
  std::vector<Eigen::Vector3d> out;
  if (cloud_->points.empty()) {return out;}
  pcl::PointXYZ q(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
  std::vector<int> idx;
  std::vector<float> sq;
  tree_.radiusSearch(q, radius, idx, sq);
  out.reserve(idx.size());
  for (int i : idx) {
    const auto & n = cloud_->points[static_cast<std::size_t>(i)];
    out.emplace_back(n.x, n.y, n.z);
  }
  return out;
}

}  // namespace orbit
}  // namespace easynav
