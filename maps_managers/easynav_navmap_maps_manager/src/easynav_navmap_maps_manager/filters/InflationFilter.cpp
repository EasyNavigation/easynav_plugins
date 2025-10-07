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


#include <expected>
#include <string>

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "navmap_core/NavMap.hpp"

#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"


namespace easynav
{
namespace navmap
{

static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;

InflationFilter::InflationFilter()
: inflation_radius_(0),
  cost_scaling_factor_(0)
{
}

bool InflationFilter::inflate_layer_u8(
  ::navmap::NavMap & nm,
  const std::string & src_layer,
  const std::string & dst_layer,
  float inflation_radius,
  float cost_scaling_factor,
  float inscribed_radius)
{
  using namespace ::navmap;
  if (nm.navcels.empty() || inflation_radius <= 0.0f || cost_scaling_factor <= 0.0f) {
    return false;
  }

  // Capas
  auto src_view = std::dynamic_pointer_cast<LayerView<uint8_t>>(nm.layers.get(src_layer));
  if (!src_view || src_view->size() != nm.navcels.size()) {
    return false;
  }
  auto dst_view = nm.layers.add_or_get<uint8_t>(dst_layer, nm.navcels.size(),
        layer_type_tag<uint8_t>());
  if (!dst_view) {return false;}
  if (dst_view->data().size() != nm.navcels.size()) {
    const_cast<std::vector<uint8_t> &>(dst_view->data()).assign(nm.navcels.size(), FREE_SPACE);
  }

  const size_t N = nm.navcels.size();
  const float R = inflation_radius;
  const float k = cost_scaling_factor;
  const float r_ins = std::clamp(inscribed_radius, 0.0f, R);

  // Precomputar centroides XY
  std::vector<Eigen::Vector2f> C(N);
  for (NavCelId cid = 0; cid < N; ++cid) {
    const Eigen::Vector3f cc = nm.navcel_centroid(cid);
    C[cid] = {cc.x(), cc.y()};
  }

  // Distancias y cola (Dijkstra multi-fuente)
  const float INF = std::numeric_limits<float>::infinity();
  std::vector<float> dist(N, INF);
  struct Node { float d; NavCelId cid; bool operator<(const Node & o) const {return d > o.d;} };
  std::priority_queue<Node> pq;

  // Inicializar dst y semillas
  auto & dst = dst_view->mutable_data(); // marca dirty
  const auto & src = src_view->data();

  bool any_seed = false;
  for (NavCelId cid = 0; cid < N; ++cid) {
    const uint8_t s = src[cid];
    if (s == LETHAL_OBSTACLE) {
      dist[cid] = 0.0f;
      pq.push({0.0f, cid});
      dst[cid] = LETHAL_OBSTACLE;
      any_seed = true;
    } else if (s == NO_INFORMATION) {
      dst[cid] = NO_INFORMATION; // mantener desconocido
    } else {
      // si dst==src (in-place), preserva el valor actual; si no, escribe 0
      if (dst_layer != src_layer) {dst[cid] = FREE_SPACE;}
    }
  }
  if (!any_seed) {return true;} // nada que inflar

  auto cost_from_dist = [&](float d) -> uint8_t {
      if (d <= 0.0f) {return LETHAL_OBSTACLE;}         // 254
      if (d <= r_ins) {return INSCRIBED_INFLATED_OBSTACLE;} // 253
      if (d > R) {return FREE_SPACE;}                  // 0

      const float x = d - r_ins; // >= 0
      const double factor = -1.0 * cost_scaling_factor * x;
      double c = std::exp(factor) * (static_cast<int>(INSCRIBED_INFLATED_OBSTACLE) - 1);
      if (c < 0.0) {c = 0.0;}
      if (c > INSCRIBED_INFLATED_OBSTACLE - 1) {c = INSCRIBED_INFLATED_OBSTACLE - 1;}
      return static_cast<uint8_t>(std::lround(c));
    };

  // Dijkstra acotado por R
  while (!pq.empty()) {
    const auto [du, u] = pq.top(); pq.pop();
    if (du != dist[u]) {continue;}
    if (du > R) {continue;}

    // Escribir coste si no es desconocido ni letal
    if (dst[u] != NO_INFORMATION && dst[u] != LETHAL_OBSTACLE) {
      const uint8_t c = cost_from_dist(du);
      if (c > dst[u]) {dst[u] = c;} // tomamos el máximo (monótono)
    }

    // Relajar vecinos
    for (NavCelId v : nm.navcel_neighbors(u)) {
      const size_t vidx = static_cast<size_t>(v);
      if (vidx >= N) {continue;}

      // Si quieres que el desconocido BLOQUEE la propagación, descomenta:
      // if (src[v] == NO_INFORMATION) continue;

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

std::expected<void, std::string>
InflationFilter::on_initialize()
{
  auto node = get_node();

  inflation_radius_ = 0.3;
  cost_scaling_factor_ = 3.0;
  inscribed_radius_ = 0.3;

  node->declare_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->declare_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->declare_parameter(plugin_name_ + ".inscribed_radius", inscribed_radius_);
  node->get_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(plugin_name_ + ".inscribed_radius", inscribed_radius_);

  RCLCPP_INFO(node->get_logger(),
    "InflationFilter with inflation_radius = %lf  cost_scaling_factor = %lf",
    inflation_radius_, cost_scaling_factor_);

  return {};
}

void
InflationFilter::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has("map")) {
    return;
  }

  navmap_ = nav_state.get<::navmap::NavMap>("map");

  if (!inflate_layer_u8(navmap_, "obstacles", "inflated_obstacles",
    inflation_radius_, cost_scaling_factor_, 0.3))
  {
    RCLCPP_ERROR(parent_node_->get_logger(), "Error inflating at ObstacleFilter");
  }

  nav_state.set("map", navmap_);
}


}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::InflationFilter, easynav::navmap::NavMapFilter)
