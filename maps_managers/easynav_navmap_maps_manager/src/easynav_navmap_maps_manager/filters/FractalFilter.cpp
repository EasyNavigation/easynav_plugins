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

#include "easynav_navmap_maps_manager/filters/FractalFilter.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

namespace easynav
{
namespace navmap
{

void FractalFilter::on_initialize()
{
  auto node = get_node();
  const std::string p = get_plugin_name() + ".";
  double height_tolerance = params_.height_tolerance;
  double max_angle_deg = params_.max_angle_deg;
  double tile_size = params_.tile_size;
  easynav::declare_parameter_if_absent(*node, p + "output_key", output_key_);
  easynav::declare_parameter_if_absent(*node, p + "layer", layer_);
  easynav::declare_parameter_if_absent(*node, p + "rebuild_period", rebuild_period_);
  easynav::declare_parameter_if_absent(*node, p + "height_tolerance", height_tolerance);
  easynav::declare_parameter_if_absent(*node, p + "max_angle_deg", max_angle_deg);
  easynav::declare_parameter_if_absent(*node, p + "tile_size", tile_size);
  easynav::declare_parameter_if_absent(*node, p + "window_radius", window_radius_);
  easynav::declare_parameter_if_absent(*node, p + "sweep", sweep_);
  easynav::declare_parameter_if_absent(*node, p + "publish_period", publish_period_);
  easynav::declare_parameter_if_absent(*node, p + "full_period", full_period_);
  easynav::declare_parameter_if_absent(*node, p + "log_period", log_period_);
  node->get_parameter(p + "output_key", output_key_);
  node->get_parameter(p + "layer", layer_);
  node->get_parameter(p + "rebuild_period", rebuild_period_);
  node->get_parameter(p + "height_tolerance", height_tolerance);
  node->get_parameter(p + "max_angle_deg", max_angle_deg);
  node->get_parameter(p + "tile_size", tile_size);
  node->get_parameter(p + "window_radius", window_radius_);
  node->get_parameter(p + "sweep", sweep_);
  node->get_parameter(p + "publish_period", publish_period_);
  node->get_parameter(p + "full_period", full_period_);
  node->get_parameter(p + "log_period", log_period_);
  params_.height_tolerance = static_cast<float>(height_tolerance);
  params_.max_angle_deg = static_cast<float>(max_angle_deg);
  params_.tile_size = static_cast<float>(tile_size);
  if (output_key_ == input_key_) {
    // the other filters would then run on the merged map
    throw std::invalid_argument(
            "[" + get_plugin_name() + "] output_key must not be " + input_key_);
  }

  // "navmap.fractal" -> ".../navmap/fractal/map"
  std::string topic_name = get_plugin_name();
  std::replace(topic_name.begin(), topic_name.end(), '.', '/');
  map_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") +
    topic_name + "/map", rclcpp::QoS(1).transient_local().reliable());
  // Only what changed since the last map or patch: what RViz's NavMap display applies
  patch_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMapPatch>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") +
    topic_name + "/map_patch", rclcpp::QoS(10).reliable());
  RCLCPP_INFO(
    node->get_logger(),
    "[%s] %s (layer %s) -> %s; merges within %.1f deg and %.2f m of the plane; costs read "
    "within %.1f m of the robot plus %d NavCels per cycle", get_plugin_name().c_str(),
    input_key_.c_str(), layer_.c_str(), output_key_.c_str(), max_angle_deg, height_tolerance,
    window_radius_, sweep_);
}

void FractalFilter::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has(input_key_)) {return;}
  auto node = get_node();
  const auto & src = nav_state.get<::navmap::NavMap>(input_key_);
  using clock = std::chrono::steady_clock;

  // A new source map (or the first one with the layer): the tree is built again
  if (!lod_.ready() || src.geometry_id() != source_geometry_ ||
    src.navcels.size() != source_navcels_)
  {
    if (!src.layers.get(layer_)) {
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000, "[%s] waiting for layer %s in %s",
        get_plugin_name().c_str(), layer_.c_str(), input_key_.c_str());
      return;
    }
    const auto t0 = clock::now();
    std::string error;
    if (!lod_.build(src, layer_, params_, &error)) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000, "[%s] cannot simplify %s: %s",
        get_plugin_name().c_str(), input_key_.c_str(), error.c_str());
      return;
    }
    source_geometry_ = src.geometry_id();
    source_navcels_ = src.navcels.size();
    dirty_ = true;
    RCLCPP_INFO(
      node->get_logger(), "[%s] tree over %zu NavCels of %.2f m built in %.0f ms",
      get_plugin_name().c_str(), source_navcels_, lod_.resolution(),
      std::chrono::duration<double, std::milli>(clock::now() - t0).count());
  } else {
    // only around the robot (where the sensors change the costs) and a slice of the map
    double x = 0.0, y = 0.0, radius = -1.0;
    if (nav_state.has("robot_pose")) {
      const auto & pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose.position;
      x = pose.x;
      y = pose.y;
      radius = window_radius_;
    }
    if (lod_.update_window(src, x, y, radius, static_cast<std::size_t>(std::max(0, sweep_))) > 0) {
      dirty_ = true;
    }
  }

  const auto now = node->now();
  if (dirty_ && (!extracted_once_ || (now - last_extract_).seconds() >= rebuild_period_)) {
    const auto t0 = clock::now();
    RtinLod::Stats st;
    out_ = lod_.extract(&st);
    const double ms_extract =
      std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    const std::string frame = RTTFBuffer::getInstance()->get_tf_info().map_frame;
    for (auto & s : out_->surfaces) {
      s.frame_id = frame;
    }
    // The same object every time, edited in place by extract(): no copy. Safe because its only
    // reader (A*) runs in the same non real-time executor as the maps managers and their filters.
    nav_state.set(output_key_, out_);
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(),
      extracted_once_ ? static_cast<int64_t>(log_period_ * 1000.0) : 0,
      "[%s] %zu NavCels (%.1f %% of %zu; %zu at the source resolution, largest %d merges) in "
      "%.0f ms (%zu of %zu tiles walked %.0f, written %.0f, indices %.0f; %zu dead)",
      get_plugin_name().c_str(), st.triangles,
      100.0 * st.triangles / std::max<std::size_t>(1, source_navcels_), source_navcels_,
      st.finest, st.max_level, ms_extract, st.tiles_rebuilt, st.tiles, st.ms_mesh,
      st.ms_assemble, st.ms_finish, st.dead);
    dirty_ = false;
    extracted_once_ = true;
    last_extract_ = now;
  }
  publish(now);
}

void FractalFilter::publish(const rclcpp::Time & now)
{
  if (!out_) {return;}
  auto node = get_node();
  const std::size_t subscribers =
    map_pub_->get_subscription_count() + patch_pub_->get_subscription_count();
  const bool new_subscriber = subscribers > last_subscribers_;
  last_subscribers_ = subscribers;
  if (subscribers == 0) {
    // Nobody keeps a copy: the next one gets a full map
    lod_.take_changes();
    need_full_ = true;
    return;
  }
  const bool due = !sent_once_ || (now - last_publish_).seconds() >= publish_period_;
  if (!due && !new_subscriber) {return;}

  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  const auto changes = lod_.take_changes();
  // (times compared only once set: an unset rclcpp::Time is of another clock)
  const bool full_due =
    sent_once_ && full_period_ > 0.0 && (now - last_full_).seconds() >= full_period_;
  std_msgs::msg::Header header;
  header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  header.stamp = now;
  std::size_t sent = 0;
  const char * kind = "";
  if (changes.full || need_full_ || new_subscriber || full_due || !sent_once_) {
    // A new output, a receiver without a copy, or the periodic resync: the whole map
    auto msg = navmap_ros::to_msg(*out_, header);
    sent = msg.navcels_v0.size();
    map_pub_->publish(msg);
    last_full_ = now;
    kind = "map";
  } else if (!changes.navcels.empty() || !changes.surfaces.empty()) {
    const auto patch = navmap_ros::to_patch_msg(
      *out_, last_sent_id_, changes.first_vertex, changes.navcels, changes.surfaces, header);
    sent = patch.navcel_indices.size();
    patch_pub_->publish(patch);
    kind = "patch";
  } else {
    last_publish_ = now;
    return;   // nothing changed
  }
  last_sent_id_ = out_->geometry_id();
  need_full_ = false;
  sent_once_ = true;
  last_publish_ = now;
  RCLCPP_DEBUG(
    node->get_logger(), "[%s] %s with %zu NavCels published in %.1f ms", get_plugin_name().c_str(),
    kind, sent, std::chrono::duration<double, std::milli>(clock::now() - t0).count());
}

}  // namespace navmap
}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::navmap::FractalFilter, easynav::navmap::NavMapFilter)
