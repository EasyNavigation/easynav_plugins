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
#include <queue>

#include "easynav_common/types/NavState.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"


namespace easynav
{
namespace navmap
{

InflationFilter::InflationFilter()
: inflation_radius_(0.0f),
  cost_scaling_factor_(0.0f),
  inscribed_radius_(0.0f)
{}

bool InflationFilter::inflate_layer_u8(
  ::navmap::NavMap & nm,
  const std::string & src_layer,
  const std::string & dst_layer,
  float inflation_radius,
  float cost_scaling_factor,
  float inscribed_radius)
{
  using ::navmap::NavCelId;
  using namespace navmap_ros;  // bring FREE_SPACE, LETHAL_OBSTACLE, etc.

  if (nm.navcels.empty() || inflation_radius <= 0.0f || cost_scaling_factor <= 0.0f) {
    return false;
  }

  // Source layer
  auto src_view =
    std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(nm.layers.get(src_layer));
  if (!src_view || src_view->size() != nm.navcels.size()) {
    return false;
  }

  // Destination layer (create if missing)
  auto dst_view = nm.layers.add_or_get<std::uint8_t>(dst_layer, nm.navcels.size(),
                                                     ::navmap::layer_type_tag<std::uint8_t>());
  if (!dst_view) {return false;}
  if (dst_view->data().size() != nm.navcels.size()) {
    const_cast<std::vector<std::uint8_t> &>(dst_view->data()).assign(nm.navcels.size(), FREE_SPACE);
  }

  const size_t N = nm.navcels.size();
  const float  R = inflation_radius;
  const float  r_ins = std::clamp(inscribed_radius, 0.0f, R);

  // Precompute XY centroids for each NavCel
  std::vector<Eigen::Vector2f> C(N);
  for (NavCelId cid = 0; cid < N; ++cid) {
    const Eigen::Vector3f cc = nm.navcel_centroid(cid);
    C[cid] = {cc.x(), cc.y()};
  }

  // Distance map (multi-source Dijkstra)
  const float INF = std::numeric_limits<float>::infinity();
  std::vector<float> dist(N, INF);
  struct Node { float d; NavCelId cid; bool operator<(const Node & o) const {return d > o.d;} };
  std::priority_queue<Node> pq;

  auto & dst = dst_view->mutable_data();        // mark dirty
  const auto & src = src_view->data();

  // Initialize destinations and seeds
  bool any_seed = false;
  for (NavCelId cid = 0; cid < N; ++cid) {
    const std::uint8_t s = src[cid];

    // Seed 1: lethal obstacles
    if (s == LETHAL_OBSTACLE) {
      dist[cid] = 0.0f;
      pq.push({0.0f, cid});
      dst[cid] = LETHAL_OBSTACLE;
      any_seed = true;
      continue;
    }

    // Seed 2: boundary triangles (missing neighbors)
    if (nm.navcel_neighbors(cid).size() < 3) {
      dist[cid] = 0.0f;
      pq.push({0.0f, cid});
      dst[cid] = LETHAL_OBSTACLE;
      any_seed = true;
      continue;
    }

    // Unknown handling
    if (s == NO_INFORMATION) {
      dst[cid] = NO_INFORMATION;
    } else {
      if (dst_layer != src_layer) {
        dst[cid] = FREE_SPACE;
      }
    }
  }

  if (!any_seed) {
    return true;  // nothing to inflate
  }

  // Distance-to-cost conversion
  auto cost_from_dist = [&](float d) -> std::uint8_t {
      if (d <= 0.0f) {return LETHAL_OBSTACLE;}
      if (d <= r_ins) {return INSCRIBED_INFLATED_OBSTACLE;}
      if (d > R) {return FREE_SPACE;}

      const float x = d - r_ins;
      const double base = static_cast<int>(INSCRIBED_INFLATED_OBSTACLE) - 1;
      double c = std::exp(-cost_scaling_factor * static_cast<double>(x)) * base;
      if (c < 0.0) {c = 0.0;}
      if (c > base) {c = base;}
      return static_cast<std::uint8_t>(std::lround(c));
    };

  // Bounded Dijkstra expansion
  while (!pq.empty()) {
    const auto [du, u] = pq.top(); pq.pop();
    if (du != dist[u]) {continue;}
    if (du > R) {continue;}

    if (dst[u] != NO_INFORMATION && dst[u] != LETHAL_OBSTACLE) {
      const std::uint8_t c = cost_from_dist(du);
      if (c > dst[u]) {dst[u] = c;} // monotonic accumulation
    }

    // Neighbor relaxation
    for (NavCelId v : nm.navcel_neighbors(u)) {
      const size_t vidx = static_cast<size_t>(v);
      if (vidx >= N) {continue;}

      if (src[v] == NO_INFORMATION) {
        continue;
      }

      const float step = (C[u] - C[v]).norm();
      if (step <= 0.0f) {continue;}
      const float alt = du + step;
      if (alt < dist[v] && alt <= R) {
        dist[v] = alt;
        pq.push({alt, v});
      }
    }
  }

  return true;
}

void
InflationFilter::on_initialize()
{
  auto node = get_node();

  // Defaults; may be overridden in parameters
  inflation_radius_ = 0.30f;
  cost_scaling_factor_ = 3.0f;
  inscribed_radius_ = 0.30f;

  node->declare_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->declare_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->declare_parameter(plugin_name_ + ".inscribed_radius", inscribed_radius_);

  node->get_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(plugin_name_ + ".inscribed_radius", inscribed_radius_);

  RCLCPP_INFO(node->get_logger(),
    "InflationFilter (NavMap): radius=%.3f cost_scaling=%.3f inscribed=%.3f",
    inflation_radius_, cost_scaling_factor_, inscribed_radius_);
}

void InflationFilter::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has("map.navmap")) {
    return;
  }

  navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");

  const bool ok = inflate_layer_u8(
    navmap_,
    "obstacles",
    "inflated_obstacles",
    inflation_radius_,
    cost_scaling_factor_,
    inscribed_radius_);

  if (!ok) {
    RCLCPP_ERROR(parent_node_->get_logger(), "InflationFilter: inflate_layer_u8() failed");
    return;
  }

  nav_state.set("map.navmap", navmap_);
}

}  // namespace navmap
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::InflationFilter, easynav::navmap::NavMapFilter)
