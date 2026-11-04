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

#include "easynav_mppi_controller/PointBuckets.hpp"

namespace easynav
{

void
PointBuckets::reset(double cx, double cy, double half_size, double cell)
{
  cell_ = std::max(cell, 1e-3);
  side_ = std::max(1, static_cast<int>(std::ceil(2.0 * half_size / cell_)));
  origin_x_ = cx - half_size;
  origin_y_ = cy - half_size;
  xs_.clear();
  ys_.clear();
  cells_.clear();
}

void
PointBuckets::add(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  const int cx = static_cast<int>(std::floor((x - origin_x_) / cell_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / cell_));
  if (cx < 0 || cy < 0 || cx >= side_ || cy >= side_) {
    return;
  }
  xs_.push_back(x);
  ys_.push_back(y);
  cells_.push_back(cy * side_ + cx);
}

void
PointBuckets::build()
{
  // Counting sort by cell.
  start_.assign(static_cast<size_t>(side_) * side_ + 1, 0);
  for (const auto c : cells_) {
    ++start_[c + 1];
  }
  for (size_t c = 1; c < start_.size(); ++c) {
    start_[c] += start_[c - 1];
  }
  sorted_x_.resize(xs_.size());
  sorted_y_.resize(ys_.size());
  fill_.assign(start_.begin(), start_.end() - 1);
  for (size_t i = 0; i < xs_.size(); ++i) {
    const int32_t at = fill_[cells_[i]]++;
    sorted_x_[at] = xs_[i];
    sorted_y_[at] = ys_[i];
  }
}

}  // namespace easynav
