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

#include "easynav_navmap_maps_manager/NavMapMapsManager.hpp"

#include "easynav_common/YTSession.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "easynav_navmap_maps_manager/map_io.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{
namespace navmap
{

using std::placeholders::_1;

NavMapMapsManager::NavMapMapsManager()
{
  ::easynav::NavState::register_printer<::navmap::NavMap>(
    [](const ::navmap::NavMap & map) {
      std::ostringstream oss;
      oss << "NavMap: navcels=" << map.navcels.size()
          << " surfaces=" << map.surfaces.size()
          << " layers=" << map.list_layers().size() << " (";

      for (const auto & layer : map.list_layers()) {
        oss << layer << " ";
      }
      oss << ")";

      return oss.str();
    });

  navmap_filters_loader_ = std::make_unique<pluginlib::ClassLoader<NavMapFilter>>(
    "easynav_navmap_maps_manager", "easynav::navmap::NavMapFilter");
}

NavMapMapsManager::~NavMapMapsManager() {}

void
NavMapMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, occmap_path_file, navmap_path_file;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".occmap_path_file", occmap_path_file);
  node->declare_parameter(plugin_name + ".navmap_path_file", navmap_path_file);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".occmap_path_file", occmap_path_file);
  node->get_parameter(plugin_name + ".navmap_path_file", navmap_path_file);

  std::vector<std::string> navmap_filters;
  node->declare_parameter(plugin_name + ".filters", navmap_filters);
  node->get_parameter(plugin_name + ".filters", navmap_filters);

  for (const auto & navmap_filter : navmap_filters) {
    std::string plugin;
    node->declare_parameter(plugin_name + "." + navmap_filter + ".plugin", plugin);
    node->get_parameter(plugin_name + "." + navmap_filter + ".plugin", plugin);

    try {
      RCLCPP_INFO(node->get_logger(),
        "Loading NavMapFilter %s [%s]", navmap_filter.c_str(), plugin.c_str());
      std::shared_ptr<NavMapFilter> instance;
      instance = navmap_filters_loader_->createSharedInstance(plugin);

      try {
        instance->initialize(node, plugin_name + "." + navmap_filter);
      } catch (const std::runtime_error & ex) {
        RCLCPP_ERROR(node->get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), ex.what());
        throw;
      }

      navmap_filters_.push_back(instance);

      RCLCPP_INFO(node->get_logger(),
        "Loaded NavMapFilter %s [%s]", navmap_filter.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(node->get_logger(),
        "Unable to load plugin easynav::navmap::NavMapFilter. Error: %s", ex.what());
      throw std::runtime_error("Unable to load plugin easynav::navmap::NavMapFilter " +
        navmap_filter + " . Error: " + ex.what());
    }
  }

  navmap_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  layer_updates_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMapLayer>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map_updates",
    rclcpp::QoS(100));

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();


  if (!package_name.empty() && !occmap_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + occmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    nav_msgs::msg::OccupancyGrid occ_msg;
    if (auto ret = loadMapFromYaml(map_path_, occ_msg) != LOAD_MAP_SUCCESS) {
      std::cerr << "loadMapFromYaml returned" << ret << std::endl;
      throw std::runtime_error("YAML file [" + map_path_ + "] not found or invalid: ");
    }

    resolution_ = occ_msg.info.resolution;
    navmap_ = navmap_ros::from_occupancy_grid(occ_msg);

    navmap_msg_ = navmap_ros::to_msg(navmap_);
    navmap_msg_.header.frame_id = tf_info.map_frame;
    navmap_msg_.header.stamp = node->now();
    navmap_pub_->publish(navmap_msg_);
  }

  if (!package_name.empty() && !navmap_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + navmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (navmap_ros::io::load_from_file(map_path_, navmap_)) {
      navmap_msg_ = navmap_ros::to_msg(navmap_);
      navmap_msg_.header.frame_id = tf_info.map_frame;
      navmap_msg_.header.stamp = node->now();
      navmap_pub_->publish(navmap_msg_);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to load %s", map_path_.c_str());
    }
  }

  incoming_occ_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_occ_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [&](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {

      resolution_ = msg->info.resolution;
      navmap_ = navmap_ros::from_occupancy_grid(*msg);

      navmap_msg_ = navmap_ros::to_msg(navmap_);
      navmap_msg_.header.frame_id = tf_info.map_frame;
      navmap_msg_.header.stamp = this->get_node()->now();
      navmap_pub_->publish(navmap_msg_);
    });

  incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_pc2_map",
    rclcpp::QoS(100),
    [&](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

      navmap_ros::BuildParams params;
      navmap_ = navmap_ros::from_pointcloud2(*msg, navmap_msg_, params);


      navmap_msg_.header.frame_id = tf_info.map_frame;
      navmap_msg_.header.stamp = this->get_node()->now();
      navmap_pub_->publish(navmap_msg_);
    });


  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      (void)response;
      navmap_ros::io::save_to_file(navmap_, "/tmp/map.navmap");
      // ToDo
    });
}

void
NavMapMapsManager::set_static_map([[maybe_unused]] const ::navmap::NavMap & new_map)
{
  // navmap_ = new_map;
}

void
NavMapMapsManager::update(::easynav::NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  if (!nav_state.has("map.navmap")) {
    nav_state.set("map.navmap", navmap_);
  }

  for (const auto & filter : navmap_filters_) {
    filter->set_map_resolution(resolution_);
    filter->update(nav_state);

    const auto & navmap = nav_state.get<::navmap::NavMap>("map.navmap");

    if (filter->is_adding_layer() && navmap.has_layer(filter->get_layer_name())) {
      auto update_msg = navmap_ros::to_msg(navmap, filter->get_layer_name());
      layer_updates_pub_->publish(update_msg);
    }
  }
}


}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::NavMapMapsManager, easynav::MapsManagerBase)
