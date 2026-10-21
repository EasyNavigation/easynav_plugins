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


#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/types/NavState.hpp"

#include "navmap_core/DistanceField.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"


namespace easynav
{
namespace navmap
{

namespace
{

// Cost of a free cell at distance d (m) from the nearest obstacle (or border)
std::uint8_t cost_at(float d, float R, float r_ins, float k)
{
  using navmap_ros::FREE_SPACE;
  using navmap_ros::INSCRIBED_INFLATED_OBSTACLE;
  if (d <= r_ins) {return INSCRIBED_INFLATED_OBSTACLE;}
  if (d > R) {return FREE_SPACE;}
  const double base = static_cast<int>(INSCRIBED_INFLATED_OBSTACLE) - 1;
  const double c = std::exp(-k * static_cast<double>(d - r_ins)) * base;
  return static_cast<std::uint8_t>(std::lround(std::clamp(c, 0.0, base)));
}

// Obstacles and unknown keep their value (unknown still carries distance); the rest, by distance
std::uint8_t inflated(std::uint8_t src, float d, float R, float r_ins, float k)
{
  if (src == navmap_ros::LETHAL_OBSTACLE || src == navmap_ros::NO_INFORMATION) {return src;}
  return cost_at(d, R, r_ins, k);
}

}  // namespace

InflationFilter::InflationFilter()
: inflation_radius_(0.0f),
  cost_scaling_factor_(0.0f),
  inscribed_radius_(0.0f)
{}

InflationFilter::~InflationFilter() = default;

bool InflationFilter::inflate_layer_u8(
  ::navmap::NavMap & nm,
  const std::string & src_layer,
  const std::string & dst_layer,
  float inflation_radius,
  float cost_scaling_factor,
  float inscribed_radius,
  bool inflate_borders)
{
  using ::navmap::NavCelId;

  if (nm.navcels.empty() || inflation_radius <= 0.0f || cost_scaling_factor <= 0.0f) {
    return false;
  }

  auto src_view =
    std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(nm.layers.get(src_layer));
  if (!src_view || src_view->size() != nm.navcels.size()) {
    return false;
  }
  const std::vector<std::uint8_t> src = src_view->data();  // dst may alias src

  auto dst_view = nm.layers.add_or_get<std::uint8_t>(
    dst_layer, nm.navcels.size(), ::navmap::layer_type_tag<std::uint8_t>());
  if (!dst_view) {return false;}
  auto & dst = dst_view->mutable_data();
  dst.resize(nm.navcels.size(), navmap_ros::FREE_SPACE);

  const std::size_t N = nm.navcels.size();
  const float R = inflation_radius;
  const float r_ins = std::clamp(inscribed_radius, 0.0f, R);

  // Distance (3D, along the surface) to the nearest obstacle cell or, if enabled, to the
  // map border: an open edge is a wall, a drop or unmapped space.
  std::vector<NavCelId> lethal;
  for (NavCelId cid = 0; cid < N; ++cid) {
    if (src[cid] == navmap_ros::LETHAL_OBSTACLE) {lethal.push_back(cid);}
  }
  ::navmap::DistanceField df(nm, inflate_borders, R);
  df.compute(lethal);

  for (NavCelId cid = 0; cid < N; ++cid) {
    const std::uint8_t c = inflated(src[cid], df[cid], R, r_ins, cost_scaling_factor);
    dst[cid] = (dst_layer == src_layer) ? std::max(src[cid], c) : c;
  }
  dst_view->mark_all_changed();
  return true;
}

void
InflationFilter::on_initialize()
{
  auto node = get_node();

  // Defaults; may be overridden in parameters
  inflation_radius_ = 0.30f;
  cost_scaling_factor_ = 3.0f;

  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".inflation_radius",
    inflation_radius_);
  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".cost_scaling_factor",
    cost_scaling_factor_);

  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".inflate_borders", inflate_borders_);

  node->get_parameter(plugin_name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(plugin_name_ + ".inflate_borders", inflate_borders_);
  // The robot's: "system_node.robot_geometry" (the filter's own is deprecated).
  inscribed_radius_ = easynav::get_robot_geometry(
    *node, {"", plugin_name_ + ".inscribed_radius", ""}).inscribed_radius;

  RCLCPP_INFO(
    node->get_logger(),
    "InflationFilter (NavMap): radius=%.3f cost_scaling=%.3f inscribed=%.3f borders=%s",
    inflation_radius_, cost_scaling_factor_, inscribed_radius_, inflate_borders_ ? "yes" : "no");
}

void InflationFilter::update(::easynav::NavState & nav_state)
{
  using ::navmap::NavCelId;
  using LayerU8 = ::navmap::LayerView<std::uint8_t>;

  if (!nav_state.has("map.navmap")) {return;}
  // In place: the map in NavState is updated, not copied
  const auto map = nav_state.get_ptr<::navmap::NavMap>("map.navmap");
  auto & nm = *map;
  const std::size_t N = nm.navcels.size();
  if (N == 0) {return;}  // no map yet (e.g. a local map before its first build)
  if (inflation_radius_ <= 0.0 || cost_scaling_factor_ <= 0.0) {return;}

  const auto src = std::dynamic_pointer_cast<LayerU8>(nm.layers.get("obstacles"));
  if (!src || src->size() != N) {return;}  // the obstacle filter has not run yet

  const float R = static_cast<float>(inflation_radius_);
  const float r_ins = std::clamp(static_cast<float>(inscribed_radius_), 0.0f, R);
  const float k = static_cast<float>(cost_scaling_factor_);
  auto dst = std::dynamic_pointer_cast<LayerU8>(nm.layers.get(get_layer_name()));

  // A new map, a new obstacle layer, or our layer gone or replaced: start over
  const bool rebuild = !df_ || df_map_ != &nm || df_geometry_ != nm.geometry_id() ||
    src != last_src_.lock() || !dst || dst != last_dst_.lock() || dst->size() != N;
  if (rebuild) {
    if (!dst || dst->size() != N) {
      nm.layers.remove(get_layer_name());
      dst = nm.add_layer<std::uint8_t>(
        get_layer_name(), "cost by distance to obstacles and borders", "", 0);
    }
    lethal_.assign(N, 0);
    std::vector<NavCelId> seeds;
    for (NavCelId c = 0; c < N; ++c) {
      if ((*src)[c] == navmap_ros::LETHAL_OBSTACLE) {lethal_[c] = 1; seeds.push_back(c);}
    }
    df_ = std::make_unique<::navmap::DistanceField>(nm, inflate_borders_, R);
    df_->compute(seeds);
    auto & d = dst->data();
    for (NavCelId c = 0; c < N; ++c) {
      d[c] = inflated((*src)[c], (*df_)[c], R, r_ins, k);
    }
    dst->mark_all_changed();
    df_map_ = &nm;
    df_geometry_ = nm.geometry_id();
    last_src_ = src;
    last_dst_ = dst;
    return;
  }

  // Only what changed: the obstacle cells that came or went move the distance field, and the
  // cells whose distance or source value changed get a new cost
  std::vector<NavCelId> touched;
  if (src->all_changed()) {
    touched.resize(N);
    for (NavCelId c = 0; c < N; ++c) {
      touched[c] = c;
    }
  } else {
    touched = src->changed();
  }
  std::vector<NavCelId> added, removed;
  for (const NavCelId c : touched) {
    const char now = (*src)[c] == navmap_ros::LETHAL_OBSTACLE;
    if (now == lethal_[c]) {continue;}
    lethal_[c] = now;
    (now ? added : removed).push_back(c);
  }
  if (!added.empty() || !removed.empty()) {df_->update(added, removed);}

  auto refresh = [&](NavCelId c) {
      const std::uint8_t v = inflated((*src)[c], (*df_)[c], R, r_ins, k);
      if ((*dst)[c] != v) {
        (*dst)[c] = v;
        dst->mark_changed(c);
      }
    };
  if (!added.empty() || !removed.empty()) {
    for (const NavCelId c : df_->changed()) {
      refresh(c);
    }
  }
  for (const NavCelId c : touched) {
    refresh(c);
  }
}

}  // namespace navmap
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::InflationFilter, easynav::navmap::NavMapFilter)
