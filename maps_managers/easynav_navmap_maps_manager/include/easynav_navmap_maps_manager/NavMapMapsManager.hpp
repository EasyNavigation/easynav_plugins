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
/// \brief Declaration of the NavMapMapsManager method.

#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__NAVMAP_MAPS_MANAGER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__NAVMAP_MAPS_MANAGER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "easynav_core/MapsManagerBase.hpp"
#include "navmap_core/NavMap.hpp"

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"
#include "pluginlib/class_loader.hpp"

#include "yaets/tracing.hpp"

namespace easynav
{
namespace navmap
{

/**
 * @class NavMapMapsManager
 * @brief A plugin-based map manager using the SimpleMap data structure.
 *
 * This manager implements a minimal mapping approach using boolean grid maps
 * (SimpleMap) for both static and dynamic maps. It supports publishing and
 * receiving ROS occupancy grids.
 */
class NavMapMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  NavMapMapsManager();

  /**
   * @brief Destructor.
   */
  ~NavMapMapsManager();

  /**
   * @brief Initializes the maps manager.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the internal maps using the current navigation state.
   *
   * Intended to be called periodically. May perform dynamic map updates
   * based on new sensor data or internal state.
   *
   * @param nav_state Current state of the navigation system.
   */
  virtual void update(::easynav::NavState & nav_state) override;

  /**
   * @brief Replaces the current static map.
   *
   * @param new_map Shared pointer to a new map object. Must be of type SimpleMap.
   */
  void set_static_map(const ::navmap::NavMap & new_map);

protected:
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;

private:
  /**
   * @brief Internal static map.
   */
  ::navmap::NavMap navmap_;

  /**
   * @brief Publisher for the navmap.
   */
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr navmap_pub_;

  /**
   * @brief Publisher for ::navmap::NavMap updates.
   */
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr layer_updates_pub_;

  /**
   * @brief Subscriber for external incoming static map updates.
   */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr incoming_occ_map_sub_;

  /**
   * @brief Subscriber for external incoming static map updates.
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;

  /**
   * @brief Service for saving current map to disk.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

  /**
   * @brief Cached occupancy grid message for the static map.
   */
  navmap_ros_interfaces::msg::NavMap navmap_msg_;

  std::unique_ptr<pluginlib::ClassLoader<NavMapFilter>> navmap_filters_loader_;

  std::vector<std::shared_ptr<NavMapFilter>> navmap_filters_;

  double resolution_ {1.0};
};

}  // namespace navmap
}  // namespace easynav
#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__NAVMAP_MAPS_MANAGER_HPP_
