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
/// \brief Points binned in square cells, to visit those near a position.

#ifndef EASYNAV_MPPI_CONTROLLER__POINTBUCKETS_HPP_
#define EASYNAV_MPPI_CONTROLLER__POINTBUCKETS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace easynav
{

/// \brief Points binned in square cells (built once per cycle, then queried many times).
class PointBuckets
{
public:
  /// \brief Empties it: cells of \p cell meters, centered on (\p cx, \p cy), \p half_size to
  /// each side (points outside are dropped).
  void reset(double cx, double cy, double half_size, double cell);

  /// \brief Adds a point. Call build() after the last one.
  void add(double x, double y);

  /// \brief Sorts the points into their cells.
  void build();

  /// \brief Calls \p visit(x, y) for every point within \p radius of (\p x, \p y) (and some
  /// beyond, in the same cells).
  template<typename Visit>
  void for_each_near(double x, double y, double radius, Visit visit) const
  {
    if (side_ == 0 || xs_.empty()) {
      return;
    }
    const int x0 = std::max(0, static_cast<int>(std::floor((x - radius - origin_x_) / cell_)));
    const int x1 =
      std::min(side_ - 1, static_cast<int>(std::floor((x + radius - origin_x_) / cell_)));
    const int y0 = std::max(0, static_cast<int>(std::floor((y - radius - origin_y_) / cell_)));
    const int y1 =
      std::min(side_ - 1, static_cast<int>(std::floor((y + radius - origin_y_) / cell_)));
    for (int cy = y0; cy <= y1; ++cy) {
      for (int cx = x0; cx <= x1; ++cx) {
        const size_t c = static_cast<size_t>(cy) * side_ + cx;
        for (int32_t i = start_[c]; i < start_[c + 1]; ++i) {
          visit(sorted_x_[i], sorted_y_[i]);
        }
      }
    }
  }

private:
  int side_ {0};
  double cell_ {0.25};
  double origin_x_ {0.0};
  double origin_y_ {0.0};
  std::vector<double> xs_;
  std::vector<double> ys_;
  std::vector<int32_t> cells_;     ///< Cell of each point.
  std::vector<int32_t> start_;     ///< First sorted point of each cell (side^2 + 1).
  std::vector<int32_t> fill_;      ///< Scratch: next free position of each cell.
  std::vector<double> sorted_x_;
  std::vector<double> sorted_y_;
};

}  // namespace easynav

#endif  // EASYNAV_MPPI_CONTROLLER__POINTBUCKETS_HPP_
