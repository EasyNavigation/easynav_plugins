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

/// \file
/// \brief Implementation of the SimpleMapsManager class.

#include "easynav_simple_maps_manager/SimpleMapsManager.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "easynav_common/YTSession.hpp"

namespace easynav
{

using std::placeholders::_1;

SimpleMapsManager::SimpleMapsManager()
{
  NavState::register_printer<SimpleMap>(
    [](const SimpleMap & map) {
      std::string ret = "SimpleMap of (" +
      std::to_string(map.width()) + " x " +
      std::to_string(map.height()) + ") with resolution " + std::to_string(map.resolution());
      return ret;
    });
}

SimpleMapsManager::~SimpleMapsManager()
{
}


void
SimpleMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".map_path_file", map_path_file);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);

  map_path_ = "/tmp/default.map";
  if (package_name != "" && map_path_file != "") {
    std::string pkgpath;
    try {
      pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + "/" + map_path_file;
    } catch(ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (!static_map_.load_from_file(map_path_)) {
      throw std::runtime_error("File [" + map_path_ + "] not found");
    }
  }

  static_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  dynamic_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/dynamic_map", 100);

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  incoming_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [&](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
      static_map_.from_occupancy_grid(*msg);
      dynamic_map_.from_occupancy_grid(*msg);

      static_map_.to_occupancy_grid(static_grid_msg_);
      static_grid_msg_.header.frame_id = tf_info.map_frame;
      static_grid_msg_.header.stamp = this->get_node()->now();

      static_occ_pub_->publish(static_grid_msg_);
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
    [&](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      if (!static_map_.save_to_file(map_path_)) {
        response->success = false;
        response->message = "Failed to save map to: " + map_path_;
      } else {
        response->success = true;
        response->message = "Map successfully saved to: " + map_path_;
      }
    });

  static_map_.to_occupancy_grid(static_grid_msg_);
  static_grid_msg_.header.frame_id = tf_info.map_frame;
  static_grid_msg_.header.stamp = node->now();

  static_occ_pub_->publish(static_grid_msg_);
}

void
SimpleMapsManager::set_static_map(const SimpleMap & new_map)
{
  static_map_ = new_map;
}

void
SimpleMapsManager::set_dynamic_map(const SimpleMap & new_map)
{
  dynamic_map_ = new_map;
}

void
SimpleMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  dynamic_map_.deep_copy(static_map_);

  if (!nav_state.has("points")) {
    nav_state.set("map.static", static_map_);
    nav_state.set("map.dynamic", dynamic_map_);
    return;
  }

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(dynamic_map_.resolution())
    .fuse(tf_info.map_frame)
    .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .as_points();

  for (const auto & p : fused) {
    if (dynamic_map_.check_bounds_metric(p.x, p.y)) {
      auto [cx, cy] = dynamic_map_.metric_to_cell(p.x, p.y);
      dynamic_map_.at(cx, cy) = 1;
    }
  }

  nav_state.set("map.static", static_map_);
  nav_state.set("map.dynamic", dynamic_map_);

  dynamic_map_.to_occupancy_grid(dynamic_grid_msg_);
  dynamic_grid_msg_.header.frame_id = tf_info.map_frame;
  dynamic_grid_msg_.header.stamp = get_node()->now();
  dynamic_occ_pub_->publish(dynamic_grid_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SimpleMapsManager, easynav::MapsManagerBase)
