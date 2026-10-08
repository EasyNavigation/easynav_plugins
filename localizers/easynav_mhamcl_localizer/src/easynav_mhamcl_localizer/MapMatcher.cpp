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
#include <set>
#include <utility>

#include "easynav_costmap_common/cost_values.hpp"

#include "easynav_mhamcl_localizer/MapMatcher.hpp"
#include "easynav_mhamcl_localizer/ParticlesDistribution.hpp"

namespace easynav
{
namespace mhamcl
{

MapMatcher::MapMatcher(const easynav::Costmap2D & map, int num_levels, double angle_step)
: angle_step_(angle_step)
{
  levels_.push_back(std::make_shared<easynav::Costmap2D>(map));
  for (int i = 1; i < num_levels; ++i) {
    // Stop before the map degenerates
    if (levels_.back()->getSizeInCellsX() < 4 || levels_.back()->getSizeInCellsY() < 4) {break;}
    levels_.push_back(half_scale(*levels_.back()));
  }
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

      if (has(easynav::LETHAL_OBSTACLE)) {
        out->setCost(i, j, easynav::LETHAL_OBSTACLE);
      } else if (has(easynav::FREE_SPACE)) {
        out->setCost(i, j, easynav::FREE_SPACE);
      } else if (std::all_of(
          costs, costs + 4, [](unsigned char c) {return c == easynav::NO_INFORMATION;}))
      {
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
  int level, const std::vector<tf2::Vector3> & points, const tf2::Transform & pose) const
{
  const auto & costmap = *levels_[level];

  // Coarser levels are evaluated with fewer points
  const std::size_t stride = static_cast<std::size_t>(1) << level;

  std::size_t hits = 0;
  std::size_t total = 0;
  for (std::size_t i = 0; i < points.size(); i += stride) {
    const tf2::Vector3 p = pose * points[i];

    unsigned int mx, my;
    if (costmap.worldToMap(p.x(), p.y(), mx, my) &&
      costmap.getCost(mx, my) == easynav::LETHAL_OBSTACLE)
    {
      ++hits;
    }
    ++total;
  }

  return total > 0 ? static_cast<double>(hits) / static_cast<double>(total) : 0.0;
}

std::vector<TransformWeighted>
MapMatcher::get_matches(const std::vector<tf2::Vector3> & points, double min_weight) const
{
  std::vector<TransformWeighted> candidates;
  if (points.empty() || levels_.empty()) {return candidates;}

  auto by_weight = [](const TransformWeighted & a, const TransformWeighted & b) {
      return a.weight > b.weight;
    };

  // Evaluate a free cell of a level with all the orientations
  auto eval_cell = [&](int level, unsigned int i, unsigned int j,
    std::vector<TransformWeighted> & out) {
      const auto & costmap = *levels_[level];
      if (costmap.getCost(i, j) != easynav::FREE_SPACE) {return;}

      double x, y;
      costmap.mapToWorld(i, j, x, y);

      for (double theta = 0.0; theta < 2.0 * M_PI - 0.1 * angle_step_; theta += angle_step_) {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, normalize_angle(theta));

        TransformWeighted tw;
        tw.transform = tf2::Transform(q, tf2::Vector3(x, y, 0.0));
        tw.weight = match(level, points, tw.transform);
        if (tw.weight > min_weight) {out.push_back(tw);}
      }
    };

  // Coarsest level: the whole map
  const int top = num_levels() - 1;
  for (unsigned int i = 0; i < levels_[top]->getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < levels_[top]->getSizeInCellsY(); ++j) {
      eval_cell(top, i, j, candidates);
    }
  }

  // Finer levels: only the cells that contain the promising candidates of the previous one
  for (int level = top; level > 0; --level) {
    std::set<std::pair<unsigned int, unsigned int>> cells;
    for (const auto & c : candidates) {
      unsigned int mx, my;
      if (levels_[level]->worldToMap(c.transform.getOrigin().x(), c.transform.getOrigin().y(),
        mx, my))
      {
        cells.emplace(mx, my);
      }
    }

    const auto & finer = *levels_[level - 1];
    candidates.clear();
    for (const auto & [cx, cy] : cells) {
      for (unsigned int i = 2 * cx; i <= 2 * cx + 1 && i < finer.getSizeInCellsX(); ++i) {
        for (unsigned int j = 2 * cy; j <= 2 * cy + 1 && j < finer.getSizeInCellsY(); ++j) {
          eval_cell(level - 1, i, j, candidates);
        }
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), by_weight);
  return candidates;
}

}  // namespace mhamcl
}  // namespace easynav
