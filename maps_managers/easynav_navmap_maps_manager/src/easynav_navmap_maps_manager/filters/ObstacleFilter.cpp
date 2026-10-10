// Copyright 2025 Intelligent Robotics Lab
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


#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "easynav_navmap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
{
namespace navmap
{

ObstacleFilter::ObstacleFilter()
{
}

void
ObstacleFilter::on_initialize()
{
  auto node = get_node();
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".max_range", max_range_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".min_height", min_height_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".max_height", max_height_);
  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".downsample_resolution", downsample_resolution_);
  node->get_parameter(plugin_name_ + ".max_range", max_range_);
  node->get_parameter(plugin_name_ + ".min_height", min_height_);
  node->get_parameter(plugin_name_ + ".max_height", max_height_);
  node->get_parameter(plugin_name_ + ".downsample_resolution", downsample_resolution_);

  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".min_height_per_meter", min_height_per_meter_);
  node->get_parameter(plugin_name_ + ".min_height_per_meter", min_height_per_meter_);
  if (!std::isfinite(min_height_per_meter_) || min_height_per_meter_ < 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] min_height_per_meter = %f must be >= 0: using 0",
      plugin_name_.c_str(), min_height_per_meter_);
    min_height_per_meter_ = 0.0;
  }
}

void ObstacleFilter::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has("map.navmap")) {return;}

  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  if (perceptions.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "There are no points perceptions");
    return;
  }

  navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  // Start from the static map, if the NavMap has one (built from an occupancy grid): what the
  // sensors do not see right now (behind something, far, too low) is still an obstacle.
  navmap_.layer_clear<uint8_t>(get_layer_name(), navmap_ros::FREE_SPACE);
  if (navmap_.has_layer("occupancy")) {
    for (std::size_t c = 0; c < navmap_.navcels.size(); ++c) {
      const auto cid = static_cast<::navmap::NavCelId>(c);
      const auto v = navmap_.layer_get<uint8_t>("occupancy", cid, navmap_ros::FREE_SPACE);
      if (v != navmap_ros::FREE_SPACE) {
        navmap_.layer_set<uint8_t>(get_layer_name(), cid, v);
      }
    }
  }

  // Only the points around the robot and below its height (robot frame), downsampled first
  // (indices only) so that fewer points are transformed.
  const auto & points = PointPerceptionsOpsView(perceptions)
    .downsample(downsample_resolution_)
    .fuse(tf_info.robot_frame)
    .filter({-max_range_, -max_range_, NAN}, {max_range_, max_range_, max_height_}, false)
    .fuse(tf_info.map_frame)
    .as_points();

  const float voxel_xy = 0.30f;

  struct Key
  {
    int ix, iy;
    bool operator==(const Key & o) const noexcept {return ix == o.ix && iy == o.iy;}
  };
  struct KeyHash
  {
    std::size_t operator()(const Key & k) const noexcept
    {
      std::size_t h1 = std::hash<int64_t>{}(static_cast<int64_t>(k.ix));
      std::size_t h2 = std::hash<int64_t>{}(static_cast<int64_t>(k.iy));
      return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
  };

  // Highest point of each column
  std::unordered_map<Key, float, KeyHash> max_z;
  max_z.reserve(points.size() / 4 + 1);
  for (const auto & p : points.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const Key key{static_cast<int>(std::floor(p.x / voxel_xy)),
      static_cast<int>(std::floor(p.y / voxel_xy))};
    auto [it, inserted] = max_z.try_emplace(key, p.z);
    if (!inserted && p.z > it->second) {it->second = p.z;}
  }

  // The robot's position, for min_height_per_meter
  Eigen::Vector2f robot_xy(0.0f, 0.0f);
  bool robot_known = false;
  if (min_height_per_meter_ > 0.0) {
    try {
      const auto tf = RTTFBuffer::getInstance()->lookupTransform(
        tf_info.map_frame, tf_info.robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
      robot_xy = {static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y)};
      robot_known = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "[%s] No robot pose (%s): min_height_per_meter not applied", plugin_name_.c_str(),
        ex.what());
    }
  }

  std::optional<size_t> last_surface;
  std::optional<::navmap::NavCelId> last_cid;

  for (const auto & [key, top] : max_z) {
    const float cx = (static_cast<float>(key.ix) + 0.5f) * voxel_xy;
    const float cy = (static_cast<float>(key.iy) + 0.5f) * voxel_xy;
    const Eigen::Vector3f query(cx, cy, top);

    size_t surface_idx = 0;
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;

    ::navmap::NavMap::LocateOpts opts;
    opts.use_downward_ray = true;
    opts.height_eps = 0.50f;
    if (last_surface) {opts.hint_surface = *last_surface;}
    if (last_cid) {opts.hint_cid = *last_cid;}

    bool ok = navmap_.locate_navcel(query, surface_idx, cid, bary, &hit, opts);

    if (!ok) {
      ::navmap::NavMap::LocateOpts nohint;
      nohint.use_downward_ray = true;
      nohint.height_eps = 0.50f;
      ok = navmap_.locate_navcel(query, surface_idx, cid, bary, &hit, nohint);
      if (!ok) {
        ok = navmap_.locate_navcel(query, surface_idx, cid, bary, &hit);
      }
    }
    if (!ok) {continue;}
    last_surface = surface_idx;
    last_cid = cid;

    // Height above the NavCel's plane (along its upward normal): on a ramp, the ramp is not one
    const auto & cel = navmap_.navcels[cid];
    Eigen::Vector3f normal = cel.normal;
    if (normal.z() < 0.0f) {normal = -normal;}
    const float height = normal.dot(query - navmap_.positions.at(cel.v[0]));
    // Farther, a small attitude error lifts the ground more: a higher threshold
    const double distance = robot_known ? (Eigen::Vector2f(cx, cy) - robot_xy).norm() : 0.0;
    if (!(height > min_height_ + min_height_per_meter_ * distance)) {continue;}

    navmap_.layer_set<uint8_t>(
      get_layer_name(), cid, static_cast<uint8_t>(navmap_ros::LETHAL_OBSTACLE));
  }

  nav_state.set("map.navmap", navmap_);
}


}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::ObstacleFilter, easynav::navmap::NavMapFilter)
