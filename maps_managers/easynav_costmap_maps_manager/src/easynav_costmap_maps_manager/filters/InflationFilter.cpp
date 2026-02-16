/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright 2025 Intelligent Robotics Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Francisco Martín Rico
 *********************************************************************/


#include <string>

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"

#include "easynav_costmap_maps_manager/filters/InflationFilter.hpp"


namespace easynav
{

InflationFilter::InflationFilter()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
}

void
InflationFilter::on_initialize()
{
  auto node = get_node();

  inflation_radius_ = 0.3;
  inscribed_radius_ = 0.25;
  cost_scaling_factor_ = 3.0;

  node->declare_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->declare_parameter(plugin_name_ + ".inscribed_radius", inscribed_radius_);
  node->declare_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(plugin_name_ + ".inscribed_radius", inscribed_radius_);
  node->get_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);

  RCLCPP_INFO(node->get_logger(),
    "InflationFilter with inflation_radius = %lf  inscribed_radius = %lf  cost_scaling_factor = %lf",
    inflation_radius_, inscribed_radius_, cost_scaling_factor_);

  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;
}

void
InflationFilter::update(NavState & nav_state)
{
  auto dynamic_map_ptr = nav_state.get_ptr<Costmap2D>("map.dynamic.filtered");
  Costmap2D & dynamic_map = *dynamic_map_ptr;

  const auto & static_map = nav_state.get<Costmap2D>("map.static");

  if (needs_recompute_static_(static_map)) {
    recompute_static_inflation_(static_map);
  }

  if (!matchedSize_) {
    cell_inflation_radius_ = cellDistance(dynamic_map, inflation_radius_);
    matchSize(dynamic_map);
    matchedSize_ = true;
  }

  int min_i = 0;
  int min_j = 0;
  int size_x = dynamic_map.getSizeInCellsX();
  int size_y = dynamic_map.getSizeInCellsY();
  int max_i = size_x;
  int max_j = size_y;

  if (nav_state.has("map.dynamic.obstacle_bounds")) {
    const auto & bb = nav_state.get<ObstacleBounds>("map.dynamic.obstacle_bounds");

    unsigned int cmin_i, cmin_j, cmax_i, cmax_j;
    if (dynamic_map.worldToMap(bb.min_x, bb.min_y, cmin_i, cmin_j) &&
      dynamic_map.worldToMap(bb.max_x, bb.max_y, cmax_i, cmax_j))
    {
      min_i = static_cast<int>(cmin_i);
      min_j = static_cast<int>(cmin_j);
      max_i = static_cast<int>(cmax_i) + 1;
      max_j = static_cast<int>(cmax_j) + 1;

      // Expand by the inflation radius in cells, so that all cells whose
      // cost may be affected by the new obstacles are included.
      const int r = static_cast<int>(cell_inflation_radius_);
      min_i = std::max(0, min_i - r);
      min_j = std::max(0, min_j - r);
      max_i = std::min(max_i + r, size_x);
      max_j = std::min(max_j + r, size_y);
    }
  }

  updateCosts(dynamic_map, min_i, min_j, max_i, max_j);

  for (int i = 0; i < dynamic_map.getSizeInCellsX(); i++) {
    for (int j = 0; j < dynamic_map.getSizeInCellsY(); j++) {
      int index = static_cast<int>(dynamic_map.getIndex(i, j));
      unsigned char cost = std::max(
        dynamic_map.getCost(i, j), static_inflated_.getCost(i, j));
      dynamic_map.setCost(i, j, cost);
    }
  }

  nav_state.set("map.dynamic.filtered", dynamic_map_ptr);
}

void
InflationFilter::matchSize(easynav::Costmap2D & costmap)
{
  resolution_ = costmap.getResolution();
  cell_inflation_radius_ = cellDistance(costmap, inflation_radius_);
  computeCaches();
  seen_ = std::vector<bool>(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);
}

void
InflationFilter::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_reinflation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void
InflationFilter::updateCosts(
  easynav::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (cell_inflation_radius_ == 0) {
    return;
  }

  // Early exit: no valid region to inflate
  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();
  const std::size_t cell_count = static_cast<std::size_t>(size_x) * size_y;
  // Reuse boolean seen_ vector for this call to avoid full O(N) clearing
  if (seen_.size() != cell_count) {
    seen_.assign(cell_count, false);
  } else {
    // Clear only inside the current bounded region to reduce cost
    for (int j = min_j; j < max_j; ++j) {
      unsigned int row = static_cast<unsigned int>(j) * size_x;
      for (int i = min_i; i < max_i; ++i) {
        seen_[row + static_cast<unsigned int>(i)] = false;
      }
    }
  }

  // Reuse all inflation distance bins instead of reallocating them
  for (auto & dist_bin : inflation_cells_) {
    dist_bin.clear();
  }

  // Store the original bounding box (before expansion by the inflation radius)
  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;

  // Expand the region by the inflation radius (in cells). Cells up to that
  // distance outside the original box can still influence costs inside it.
  min_i -= static_cast<int>(cell_inflation_radius_);
  min_j -= static_cast<int>(cell_inflation_radius_);
  max_i += static_cast<int>(cell_inflation_radius_);
  max_j += static_cast<int>(cell_inflation_radius_);

  // Clamp expanded region to map bounds
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  // --------------------------------------------------------------------------
  // 1) Collect seeds (lethal obstacles and optionally unknown cells) inside
  //    the expanded window. These go into distance bin 0.
  // --------------------------------------------------------------------------
  auto & obs_bin = inflation_cells_[0];
  obs_bin.reserve(200);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const unsigned int index = master_grid.getIndex(i, j);
      const unsigned char cost = master_array[index];

      if (cost == LETHAL_OBSTACLE ||
        (inflate_around_unknown_ && cost == NO_INFORMATION))
      {
        obs_bin.emplace_back(i, j, i, j);
      }
    }
  }

  // No seeds in this region → nothing to inflate
  if (obs_bin.empty()) {
    return;
  }

  // --------------------------------------------------------------------------
  // 2) BFS over the distance bins. New cells are appended to the bin
  //    corresponding to their distance from the nearest obstacle.
  // --------------------------------------------------------------------------
  for (auto & dist_bin : inflation_cells_) {
    dist_bin.reserve(200);

    for (std::size_t k = 0; k < dist_bin.size(); ++k) {
      const CellData & cell = dist_bin[k];
      const unsigned int mx = cell.x_;
      const unsigned int my = cell.y_;
      const unsigned int sx = cell.src_x_;
      const unsigned int sy = cell.src_y_;
      const unsigned int index = master_grid.getIndex(mx, my);
      // Process each cell only once in this call
      if (seen_[index]) {
        continue;
      }
      seen_[index] = true;

      // Compute inflation cost based on distance to the nearest obstacle
      const unsigned char cost = costLookup(mx, my, sx, sy);
      const unsigned char old_cost = master_array[index];

      // Apply inflation only inside the original (non-expanded) bounding box
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        if (old_cost == NO_INFORMATION &&
          (inflate_unknown_ ?
          (cost > FREE_SPACE) :
          (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        {
          master_array[index] = cost;
        } else {
          master_array[index] = std::max(old_cost, cost);
        }
      }

      // Enqueue 4-connected neighbors (clipped to full map bounds)
      if (mx > 0u) {
        enqueue(index - 1u, mx - 1u, my, sx, sy);
      }
      if (my > 0u) {
        enqueue(index - size_x, mx, my - 1u, sx, sy);
      }
      if (mx + 1u < size_x) {
        enqueue(index + 1u, mx + 1u, my, sx, sy);
      }
      if (my + 1u < size_y) {
        enqueue(index + size_x, mx, my + 1u, sx, sy);
      }
    }

    // No need to shrink dist_bin here; capacity is reused on the next call.
  }
}

void
InflationFilter::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // we compute our distance table one cell further than the
    // inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the inflation list and mark
    const auto dist = distance_matrix_[mx - src_x + r][my - src_y + r];
    inflation_cells_[dist].emplace_back(mx, my, src_x, src_y);
    // Do not mark seen_ here; we mark upon processing to allow enqueuing neighbors
  }
}

void
InflationFilter::computeCaches()
{
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  int max_dist = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(max_dist + 1);
}

int
InflationFilter::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;
  const int size = r * 2 + 1;

  std::vector<std::pair<int, int>> points;

  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }

  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  distance_matrix_ = distance_matrix;
  return level;
}

bool
InflationFilter::needs_recompute_static_(const Costmap2D & static_map) const
{
  if (!has_static_inflated_) {
    return true;
  }

  if (static_map.getSizeInCellsX() != static_sig_.size_x ||
    static_map.getSizeInCellsY() != static_sig_.size_y ||
    static_map.getResolution() != static_sig_.resolution ||
    static_map.getOriginX() != static_sig_.origin_x ||
    static_map.getOriginY() != static_sig_.origin_y)
  {
    return true;
  }

  return false;
}

void
InflationFilter::recompute_static_inflation_(const Costmap2D & static_map)
{
  static_inflated_ = static_map;

  matchSize(static_inflated_);
  const int w = static_inflated_.getSizeInCellsX();
  const int h = static_inflated_.getSizeInCellsY();
  updateCosts(static_inflated_, 0, 0, w, h);

  static_sig_.size_x = static_map.getSizeInCellsX();
  static_sig_.size_y = static_map.getSizeInCellsY();
  static_sig_.resolution = static_map.getResolution();
  static_sig_.origin_x = static_map.getOriginX();
  static_sig_.origin_y = static_map.getOriginY();

  has_static_inflated_ = true;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::InflationFilter, easynav::CostmapFilter)
