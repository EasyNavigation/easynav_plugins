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

#include "easynav_mppi_controller/NearestGrid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace easynav
{

void
NearestGrid::reserve(int max_cells_per_side)
{
  max_side_ = std::max(1, max_cells_per_side);
  cell_seed_.reserve(static_cast<size_t>(max_side_) * max_side_);
  // Room for a dense scan plus a long path; more only grows once.
  seed_ids_.reserve(8192);
  seed_x_.reserve(8192);
  seed_y_.reserve(8192);
}

void
NearestGrid::reset(double cx, double cy, double half_size, double resolution)
{
  if (max_side_ == 0) {
    reserve(400);
  }
  half_size = std::max(half_size, resolution);
  side_ = static_cast<int>(std::ceil(2.0 * half_size / resolution));
  resolution_ = resolution;
  if (side_ > max_side_) {
    side_ = max_side_;
    resolution_ = 2.0 * half_size / side_;
  }
  origin_x_ = cx - half_size;
  origin_y_ = cy - half_size;
  cell_seed_.assign(static_cast<size_t>(side_) * side_, -1);
  seed_ids_.clear();
  seed_x_.clear();
  seed_y_.clear();
}

void
NearestGrid::add_seed(int id, double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  if (cx < 0 || cy < 0 || cx >= side_ || cy >= side_) {
    return;
  }
  const auto seed = static_cast<int32_t>(seed_x_.size());
  seed_ids_.push_back(id);
  seed_x_.push_back(x);
  seed_y_.push_back(y);
  auto & cell = cell_seed_[static_cast<size_t>(cy) * side_ + cx];
  // Several seeds in a cell: the one nearest its center.
  if (cell < 0 || cell_dist2(cx, cy, seed) < cell_dist2(cx, cy, cell)) {
    cell = seed;
  }
}

double
NearestGrid::cell_dist2(int cx, int cy, int32_t s) const
{
  const double dx = origin_x_ + (cx + 0.5) * resolution_ - seed_x_[s];
  const double dy = origin_y_ + (cy + 0.5) * resolution_ - seed_y_[s];
  return dx * dx + dy * dy;
}

void
NearestGrid::relax(int cx, int cy, int nx, int ny, double & best)
{
  if (nx < 0 || ny < 0 || nx >= side_ || ny >= side_) {
    return;
  }
  const int32_t candidate = cell_seed_[static_cast<size_t>(ny) * side_ + nx];
  if (candidate < 0) {
    return;
  }
  const double d2 = cell_dist2(cx, cy, candidate);
  if (d2 < best) {
    best = d2;
    cell_seed_[static_cast<size_t>(cy) * side_ + cx] = candidate;
  }
}

void
NearestGrid::propagate()
{
  if (seed_x_.empty()) {
    return;
  }
  constexpr double kNone = std::numeric_limits<double>::infinity();
  auto current = [this](int x, int y) {
      const int32_t s = cell_seed_[static_cast<size_t>(y) * side_ + x];
      return s < 0 ? kNone : cell_dist2(x, y, s);
    };
  // Forward pass: neighbours already visited (left, and the row below).
  for (int y = 0; y < side_; ++y) {
    for (int x = 0; x < side_; ++x) {
      double best = current(x, y);
      relax(x, y, x - 1, y, best);
      relax(x, y, x - 1, y - 1, best);
      relax(x, y, x, y - 1, best);
      relax(x, y, x + 1, y - 1, best);
    }
    for (int x = side_ - 1; x >= 0; --x) {
      double best = current(x, y);
      relax(x, y, x + 1, y, best);
    }
  }
  // Backward pass: right, and the row above.
  for (int y = side_ - 1; y >= 0; --y) {
    for (int x = side_ - 1; x >= 0; --x) {
      double best = current(x, y);
      relax(x, y, x + 1, y, best);
      relax(x, y, x + 1, y + 1, best);
      relax(x, y, x, y + 1, best);
      relax(x, y, x - 1, y + 1, best);
    }
    for (int x = 0; x < side_; ++x) {
      double best = current(x, y);
      relax(x, y, x - 1, y, best);
    }
  }
}

int
NearestGrid::nearest(double x, double y) const
{
  if (seed_x_.empty() || side_ == 0 || !std::isfinite(x) || !std::isfinite(y)) {
    return -1;
  }
  const int cx = std::clamp(
    static_cast<int>(std::floor((x - origin_x_) / resolution_)), 0, side_ - 1);
  const int cy = std::clamp(
    static_cast<int>(std::floor((y - origin_y_) / resolution_)), 0, side_ - 1);
  return cell_seed_[static_cast<size_t>(cy) * side_ + cx];
}

double
NearestGrid::distance(double x, double y, double none) const
{
  const int seed = nearest(x, y);
  if (seed < 0) {
    return none;
  }
  return std::hypot(x - seed_x_[seed], y - seed_y_[seed]);
}

}  // namespace easynav
