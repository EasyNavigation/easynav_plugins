// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the SimpleMapsManager class.

#include <expected>

#include "easynav_simple_maps_manager/SimpleMapsManager.hpp"
#include "easynav_common/types/Perceptions.hpp"
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


std::expected<void, std::string>
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
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (!static_map_.load_from_file(map_path_)) {
      return std::unexpected("File [" + map_path_ + "] not found");
    }
  }

  static_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  dynamic_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/dynamic_map", 100);

  incoming_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
      static_map_.from_occupancy_grid(*msg);
      dynamic_map_.from_occupancy_grid(*msg);

      static_map_.to_occupancy_grid(static_grid_msg_);
      static_grid_msg_.header.frame_id = get_tf_prefix() + "map";
      static_grid_msg_.header.stamp = this->get_node()->now();

      static_occ_pub_->publish(static_grid_msg_);
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
    [this](
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
  static_grid_msg_.header.frame_id = get_tf_prefix() + "map";
  static_grid_msg_.header.stamp = node->now();

  static_occ_pub_->publish(static_grid_msg_);

  return {};
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

  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(dynamic_map_.resolution())
    .fuse(get_tf_prefix() + "map")
    ->filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
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
  dynamic_grid_msg_.header.frame_id = get_tf_prefix() + "map";
  dynamic_grid_msg_.header.stamp = get_node()->now();
  dynamic_occ_pub_->publish(dynamic_grid_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SimpleMapsManager, easynav::MapsManagerBase)
