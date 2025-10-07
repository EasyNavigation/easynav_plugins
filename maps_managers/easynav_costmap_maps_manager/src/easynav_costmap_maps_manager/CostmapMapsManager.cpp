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

#include <expected>
#include <string>

#include "easynav_costmap_maps_manager/CostmapMapsManager.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_maps_manager/map_io.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{

using std::placeholders::_1;

CostmapMapsManager::CostmapMapsManager()
{
  NavState::register_printer<Costmap2D>(
    [](const Costmap2D & map) {
      std::ostringstream oss;
      oss << "Costmap2D of (" << map.getSizeInCellsX() << " x " << map.getSizeInCellsY()
          << ") with resolution " << map.getResolution();
      return oss.str();
    });

  costmap_filters_loader_ = std::make_unique<pluginlib::ClassLoader<CostmapFilter>>(
    "easynav_costmap_maps_manager", "easynav::CostmapFilter");
}

CostmapMapsManager::~CostmapMapsManager() {}

std::expected<void, std::string>
CostmapMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".map_path_file", map_path_file);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);

  std::vector<std::string> costmap_filters;
  node->declare_parameter(plugin_name + ".filters", costmap_filters);
  node->get_parameter(plugin_name + ".filters", costmap_filters);

  for (const auto & costmap_filter : costmap_filters) {
    std::string plugin;
    node->declare_parameter(plugin_name + "." + costmap_filter + ".plugin", plugin);
    node->get_parameter(plugin_name + "." + costmap_filter + ".plugin", plugin);

    try {
      RCLCPP_INFO(node->get_logger(),
        "Loading CostmapFilter %s [%s]", costmap_filter.c_str(), plugin.c_str());

      std::shared_ptr<CostmapFilter> instance;
      instance = costmap_filters_loader_->createSharedInstance(plugin);

      auto result = instance->initialize(node, plugin_name + "." + costmap_filter,
        get_tf_prefix());

      if (!result) {
        RCLCPP_ERROR(node->get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), result.error().c_str());
        return std::unexpected("Unable to initialize " +
          plugin + " . Error: " + result.error());
      }

      costmap_filters_.push_back(instance);

      RCLCPP_INFO(node->get_logger(),
        "Loaded CostmapFilter %s [%s]", costmap_filter.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(node->get_logger(),
        "Unable to load plugin easynav::CostmapFilter. Error: %s", ex.what());
      return std::unexpected("Unable to load plugin easynav::CostmapFilter " +
        costmap_filter + " . Error: " + ex.what());
    }
  }

  static_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  dynamic_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/dynamic_map", 100);

  map_path_ = "/tmp/default.map.yaml";
  if (!package_name.empty() && !map_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + map_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (auto ret = loadMapFromYaml(map_path_, static_grid_msg_) != LOAD_MAP_SUCCESS) {
      std::cerr << "loadMapFromYaml returned" << ret << std::endl;
      return std::unexpected("YAML file [" + map_path_ + "] not found or invalid: ");
    }

    static_map_ = Costmap2D(static_grid_msg_);

    static_grid_msg_.header.frame_id = get_tf_prefix() + "map";
    static_grid_msg_.header.stamp = node->now();
    static_occ_pub_->publish(static_grid_msg_);
  }

  incoming_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
      static_grid_msg_ = *msg;

      static_map_ = Costmap2D(*msg);

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

      SaveParameters params;
      params.map_file_name = map_path_;
      params.image_format = "pgm";
      params.mode = MapMode::Trinary;
      params.free_thresh = 0.25;
      params.occupied_thresh = 0.65;

      if (!saveMapToFile(static_grid_msg_, params)) {
        response->success = false;
        response->message = "Failed to save map to: " + map_path_;
      } else {
        response->success = true;
        response->message = "Map successfully saved to: " + map_path_;
      }
    });

  return {};
}

void
CostmapMapsManager::set_static_map(const Costmap2D & new_map)
{
  static_map_ = new_map;
}


void
CostmapMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  if (!nav_state.has("map.static")) {
    nav_state.set("map.static", static_map_);
  }

  Costmap2D dynamic_map = static_map_;
  nav_state.set("map.dynamic.filtered", dynamic_map);

  for (const auto & filter : costmap_filters_) {
    filter->update(nav_state);
  }

  const auto & final_dynamic_map = nav_state.get<Costmap2D>("map.dynamic.filtered");
  nav_state.set("map.dynamic", final_dynamic_map);

  final_dynamic_map.toOccupancyGridMsg(dynamic_grid_msg_);
  dynamic_grid_msg_.header.frame_id = get_tf_prefix() + "map";
  dynamic_grid_msg_.header.stamp = get_node()->now();
  dynamic_occ_pub_->publish(dynamic_grid_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapMapsManager, easynav::MapsManagerBase)
