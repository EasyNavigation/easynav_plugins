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
/// \brief Implementation of MapMatcher.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <tuple>
#include <vector>
#include <utility>

#include "easynav_costmap_common/cost_values.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "easynav_mhamcl_localizer/MapMatcher.hpp"
#include "easynav_mhamcl_localizer/ParticlesDistribution.hpp"

namespace easynav
{
namespace mhamcl
{

MapMatcher::MapMatcher(
  const easynav::Costmap2D & map, int num_levels, double angle_step, double sigma,
  std::size_t beam_width)
: angle_step_(angle_step), sigma_(sigma), beam_width_(std::max<std::size_t>(beam_width, 1))
{
  levels_.push_back(std::make_shared<easynav::Costmap2D>(map));
  for (int i = 1; i < num_levels; ++i) {
    // Stop before the map degenerates
    if (levels_.back()->getSizeInCellsX() < 4 || levels_.back()->getSizeInCellsY() < 4) {break;}
    levels_.push_back(half_scale(*levels_.back()));
  }
  build_distance_field(map);
}

void
MapMatcher::build_distance_field(const easynav::Costmap2D & map)
{
  // Two-pass chamfer distance (3x3, 1 and sqrt(2) cells) to the nearest obstacle.
  const unsigned int w = map.getSizeInCellsX();
  const unsigned int h = map.getSizeInCellsY();
  const float inf = std::numeric_limits<float>::infinity();
  const float res = static_cast<float>(map.getResolution());
  const float diag = res * static_cast<float>(M_SQRT2);
  distance_.assign(static_cast<std::size_t>(w) * h, inf);
  if (w == 0 || h == 0) {return;}

  auto at = [&](unsigned int i, unsigned int j) -> float & {
      return distance_[static_cast<std::size_t>(j) * w + i];
    };
  for (unsigned int j = 0; j < h; ++j) {
    for (unsigned int i = 0; i < w; ++i) {
      if (map.getCost(i, j) == easynav::LETHAL_OBSTACLE) {at(i, j) = 0.0f;}
    }
  }
  for (unsigned int j = 0; j < h; ++j) {
    for (unsigned int i = 0; i < w; ++i) {
      float & d = at(i, j);
      if (i > 0) {d = std::min(d, at(i - 1, j) + res);}
      if (j > 0) {
        d = std::min(d, at(i, j - 1) + res);
        if (i > 0) {d = std::min(d, at(i - 1, j - 1) + diag);}
        if (i + 1 < w) {d = std::min(d, at(i + 1, j - 1) + diag);}
      }
    }
  }
  for (unsigned int jj = h; jj-- > 0; ) {
    for (unsigned int ii = w; ii-- > 0; ) {
      float & d = at(ii, jj);
      if (ii + 1 < w) {d = std::min(d, at(ii + 1, jj) + res);}
      if (jj + 1 < h) {
        d = std::min(d, at(ii, jj + 1) + res);
        if (ii + 1 < w) {d = std::min(d, at(ii + 1, jj + 1) + diag);}
        if (ii > 0) {d = std::min(d, at(ii - 1, jj + 1) + diag);}
      }
    }
  }
}

double
MapMatcher::distance_to_obstacle(double x, double y) const
{
  unsigned int mx, my;
  if (distance_.empty() || !levels_[0]->worldToMap(x, y, mx, my)) {
    return std::numeric_limits<double>::infinity();
  }
  return distance_[static_cast<std::size_t>(my) * levels_[0]->getSizeInCellsX() + mx];
}

double
MapMatcher::angle_step(int level) const
{
  return angle_step_ / static_cast<double>(1 << (num_levels() - 1 - level));
}

double
MapMatcher::sigma_of(int level, double typical_range) const
{
  // The sensor's, plus half a cell and the drift of a point half an angle step off.
  const double half_cell = 0.5 * levels_[level]->getResolution();
  const double half_turn = 0.5 * typical_range * angle_step(level);
  return std::sqrt(sigma_ * sigma_ + half_cell * half_cell + half_turn * half_turn);
}

std::shared_ptr<easynav::Costmap2D>
MapMatcher::half_scale(const easynav::Costmap2D & in)
{
  auto out = std::make_shared<easynav::Costmap2D>(
    in.getSizeInCellsX() / 2, in.getSizeInCellsY() / 2, in.getResolution() * 2.0,
    in.getOriginX(), in.getOriginY(), in.getCost(0, 0));

  for (unsigned int i = 0; i < out->getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < out->getSizeInCellsY(); ++j) {
      const unsigned char costs[4] = {
        in.getCost(2 * i, 2 * j), in.getCost(2 * i + 1, 2 * j),
        in.getCost(2 * i, 2 * j + 1), in.getCost(2 * i + 1, 2 * j + 1)};

      auto has = [&costs](unsigned char v) {return std::find(costs, costs + 4, v) != costs + 4;};

      const bool unknown = std::all_of(
        costs, costs + 4, [](unsigned char c) {return c == easynav::NO_INFORMATION;});
      if (has(easynav::LETHAL_OBSTACLE)) {
        out->setCost(i, j, easynav::LETHAL_OBSTACLE);
      } else if (has(easynav::FREE_SPACE)) {
        out->setCost(i, j, easynav::FREE_SPACE);
      } else if (unknown) {
        out->setCost(i, j, easynav::NO_INFORMATION);
      } else {
        out->setCost(i, j, costs[0]);
      }
    }
  }

  return out;
}

double
MapMatcher::match(
  int level, const std::vector<tf2::Vector3> & points, const tf2::Transform & pose,
  double typical_range) const
{
  // Coarser levels are evaluated with fewer points
  const std::size_t stride = static_cast<std::size_t>(1) << level;
  const double sigma = sigma_of(level, typical_range);
  const double k = -0.5 / (sigma * sigma);

  double sum = 0.0;
  std::size_t total = 0;
  for (std::size_t i = 0; i < points.size(); i += stride) {
    const tf2::Vector3 p = pose * points[i];
    const double d = distance_to_obstacle(p.x(), p.y());
    if (std::isfinite(d)) {sum += std::exp(k * d * d);}
    ++total;
  }

  return total > 0 ? sum / static_cast<double>(total) : 0.0;
}

std::vector<TransformWeighted>
MapMatcher::get_matches(const std::vector<tf2::Vector3> & points, double min_weight) const
{
  std::vector<TransformWeighted> candidates;
  if (points.empty() || levels_.empty()) {return candidates;}

  // Typical range of the points: how far an angle error moves them
  std::vector<double> ranges;
  ranges.reserve(points.size());
  for (const auto & p : points) {
    ranges.push_back(std::hypot(p.x(), p.y()));
  }
  std::nth_element(ranges.begin(), ranges.begin() + ranges.size() / 2, ranges.end());
  const double typical_range = ranges[ranges.size() / 2];

  auto by_weight = [](const TransformWeighted & a, const TransformWeighted & b) {
      return a.weight > b.weight;
    };
  auto keep_best = [&](std::vector<TransformWeighted> & cands) {
      if (cands.size() > beam_width_) {
        std::partial_sort(
          cands.begin(), cands.begin() + static_cast<std::ptrdiff_t>(beam_width_), cands.end(),
          by_weight);
        cands.resize(beam_width_);
      }
    };
  auto make = [](double x, double y, double theta) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, normalize_angle(theta));
      return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
    };

  // Coarsest level: every free cell of the whole map, at every angle step
  const int top = num_levels() - 1;
  const auto & coarse = *levels_[top];
  const double top_step = angle_step(top);
  const int top_angles = std::max(1, static_cast<int>(std::round(2.0 * M_PI / top_step)));
  for (unsigned int i = 0; i < coarse.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < coarse.getSizeInCellsY(); ++j) {
      if (coarse.getCost(i, j) != easynav::FREE_SPACE) {continue;}
      double x, y;
      coarse.mapToWorld(i, j, x, y);
      for (int a = 0; a < top_angles; ++a) {
        TransformWeighted tw;
        tw.transform = make(x, y, a * top_step);
        tw.weight = match(top, points, tw.transform, typical_range);
        candidates.push_back(tw);
      }
    }
  }
  keep_best(candidates);

  // Finer levels: the sub-cells of the best candidates, around their heading
  for (int level = top - 1; level >= 0; --level) {
    const auto & finer = *levels_[level];
    const auto & parent = *levels_[level + 1];
    const double step = angle_step(level);

    std::set<std::tuple<unsigned int, unsigned int, int64_t>> seen;
    std::vector<TransformWeighted> refined;
    for (const auto & c : candidates) {
      unsigned int pi, pj;
      if (!parent.worldToMap(
          c.transform.getOrigin().x(), c.transform.getOrigin().y(), pi, pj))
      {
        continue;
      }
      const double theta = yaw_of(c.transform);
      for (unsigned int i = 2 * pi; i <= 2 * pi + 1 && i < finer.getSizeInCellsX(); ++i) {
        for (unsigned int j = 2 * pj; j <= 2 * pj + 1 && j < finer.getSizeInCellsY(); ++j) {
          if (finer.getCost(i, j) != easynav::FREE_SPACE) {continue;}
          double x, y;
          finer.mapToWorld(i, j, x, y);
          for (int da = -1; da <= 1; ++da) {
            const double a = theta + da * step;
            const int64_t key = std::llround(normalize_angle(a) / step);
            if (!seen.emplace(i, j, key).second) {continue;}
            TransformWeighted tw;
            tw.transform = make(x, y, a);
            tw.weight = match(level, points, tw.transform, typical_range);
            refined.push_back(tw);
          }
        }
      }
    }
    candidates = std::move(refined);
    keep_best(candidates);
  }

  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(),
      [min_weight](const TransformWeighted & c) {return c.weight <= min_weight;}),
    candidates.end());
  std::sort(candidates.begin(), candidates.end(), by_weight);
  return candidates;
}

}  // namespace mhamcl
}  // namespace easynav
