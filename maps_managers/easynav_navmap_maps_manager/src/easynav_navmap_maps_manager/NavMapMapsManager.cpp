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

#include "easynav_navmap_maps_manager/NavMapMapsManager.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/YTSession.hpp"

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
      (void)map;
      std::ostringstream oss;
      oss << "NavMap Navcells = " << map.navcels.size() << "\tlayers = " <<
        map.list_layers().size();
      return oss.str();
    });

  // navmap_filters_loader_ = std::make_unique<pluginlib::ClassLoader<NavMapFilter>>(
  //   "easynav_navmap_filter", "easynav::navmap::NavMapFilter");
}

NavMapMapsManager::~NavMapMapsManager() {}

std::expected<void, std::string>
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

      auto result = instance->initialize(node, plugin_name + "." + navmap_filter,
        get_tf_prefix());

      if (!result) {
        RCLCPP_ERROR(node->get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), result.error().c_str());
        return std::unexpected("Unable to initialize " +
          plugin + " . Error: " + result.error());
      }

      navmap_filters_.push_back(instance);

      RCLCPP_INFO(node->get_logger(),
        "Loaded NavMapFilter %s [%s]", navmap_filter.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(node->get_logger(),
        "Unable to load plugin easynav::navmap::NavMapFilter. Error: %s", ex.what());
      return std::unexpected("Unable to load plugin easynav::navmap::NavMapFilter " +
        navmap_filter + " . Error: " + ex.what());
    }
  }

  navmap_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  layer_updates_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMapLayer>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map_updates",
    rclcpp::QoS(100));

  if (!package_name.empty() && !occmap_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + occmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    nav_msgs::msg::OccupancyGrid occ_msg;
    if (auto ret = loadMapFromYaml(map_path_, occ_msg) != LOAD_MAP_SUCCESS) {
      std::cerr << "loadMapFromYaml returned" << ret << std::endl;
      return std::unexpected("YAML file [" + map_path_ + "] not found or invalid: ");
    }

    resolution_ = occ_msg.info.resolution;
    navmap_ = navmap_ros::from_occupancy_grid(occ_msg);

    navmap_msg_ = navmap_ros::to_msg(navmap_);
    navmap_msg_.header.frame_id = "map";
    navmap_msg_.header.stamp = node->now();
    navmap_pub_->publish(navmap_msg_);
  }

  if (!package_name.empty() && !navmap_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + navmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (navmap_ros::io::load_from_file(map_path_, navmap_)) {
      navmap_msg_ = navmap_ros::to_msg(navmap_);
      navmap_msg_.header.frame_id = "map";
      navmap_msg_.header.stamp = node->now();
      navmap_pub_->publish(navmap_msg_);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to load %s", map_path_.c_str());
    }
  }

  incoming_occ_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_occ_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {

      resolution_ = msg->info.resolution;
      navmap_ = navmap_ros::from_occupancy_grid(*msg);

      navmap_msg_ = navmap_ros::to_msg(navmap_);
      navmap_msg_.header.frame_id = "map";
      navmap_msg_.header.stamp = this->get_node()->now();
      navmap_pub_->publish(navmap_msg_);
    });

  incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_pc2_map",
    rclcpp::QoS(100),
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

      navmap_ros::BuildParams params;
      navmap_ = navmap_ros::from_pointcloud2(*msg, navmap_msg_, params);


      navmap_msg_.header.frame_id = "map";
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

  return {};
}

void
NavMapMapsManager::set_static_map(const ::navmap::NavMap & new_map)
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

  nav_state.set("map.navmap", navmap_);

//   if (!navmap_.has_layer("occupancy")) {
//     navmap_.add_layer<uint8_t>("occupancy", "Per-NavCel occupancy (0=free, 255=unknown)", "",
//           static_cast<uint8_t>(0));
//   }
//
//   for (const auto & filter : navmap_filters_) {
//     filter->set_map_resolution(resolution_);
//     filter->update(nav_state);
//   }
//
//   navmap_ = nav_state.get<::navmap::NavMap>("map");
//
//   for (const auto & filter : navmap_filters_) {
//
//     if (filter->is_adding_layer() && navmap_.has_layer(filter->get_layer_name())) {
//       auto update_msg = navmap_ros::to_msg(navmap_, filter->get_layer_name());
//       layer_updates_pub_->publish(update_msg);
//     }
//   }
}


}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::NavMapMapsManager, easynav::MapsManagerBase)
