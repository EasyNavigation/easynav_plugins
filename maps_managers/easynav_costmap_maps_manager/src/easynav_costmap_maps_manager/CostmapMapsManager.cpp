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

#include <stdexcept>
#include <string>

#include "easynav_costmap_maps_manager/CostmapMapsManager.hpp"

#include "easynav_common/YTSession.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_maps_manager/map_io.hpp"
#include "easynav_common/RTTFBuffer.hpp"

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

void
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

      try {
        instance->initialize(node, plugin_name + "." + costmap_filter);
      } catch (std::runtime_error & ex) {
        RCLCPP_ERROR(node->get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), ex.what());
        throw std::runtime_error("Unable to initialize " +
          plugin + " . Error: " + ex.what());
      }

      costmap_filters_.push_back(instance);

      RCLCPP_INFO(node->get_logger(),
        "Loaded CostmapFilter %s [%s]", costmap_filter.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(node->get_logger(),
        "Unable to load plugin easynav::CostmapFilter. Error: %s", ex.what());
      throw std::runtime_error("Unable to load plugin easynav::CostmapFilter " +
        costmap_filter + " . Error: " + ex.what());
    }
  }

  static_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  dynamic_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/dynamic_map", 100);

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  map_path_ = "/tmp/default.map.yaml";
  if (!package_name.empty() && !map_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + map_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (auto ret = loadMapFromYaml(map_path_, static_grid_msg_) != LOAD_MAP_SUCCESS) {
      std::cerr << "loadMapFromYaml returned" << ret << std::endl;
      throw std::runtime_error("YAML file [" + map_path_ + "] not found or invalid: ");
    }

    static_map_ = Costmap2D(static_grid_msg_);

    static_grid_msg_.header.frame_id = tf_info.map_frame;
    static_grid_msg_.header.stamp = node->now();
    static_occ_pub_->publish(static_grid_msg_);
  }

  incoming_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [&](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
      static_grid_msg_ = *msg;

      static_map_ = Costmap2D(*msg);

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

  if (!dynamic_map_) {
    dynamic_map_ = std::make_shared<Costmap2D>(static_map_);
  } else {
    *dynamic_map_ = static_map_;
  }

  nav_state.set("map.dynamic.filtered", dynamic_map_);

  if (!nav_state.has("map_time")) {
    nav_state.set("map_time", get_node()->now());
  }

  for (const auto & filter : costmap_filters_) {
    filter->update(nav_state);
  }

  nav_state.set("map.dynamic", dynamic_map_);

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  rclcpp::Time map_stamp = nav_state.get<rclcpp::Time>("map_time");

  dynamic_map_->toOccupancyGridMsg(dynamic_grid_msg_);
  dynamic_grid_msg_.header.frame_id = tf_info.map_frame;
  dynamic_grid_msg_.header.stamp = map_stamp;
  dynamic_occ_pub_->publish(dynamic_grid_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapMapsManager, easynav::MapsManagerBase)
