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
/// \brief Declaration of the OctomapMapsManager method.

#ifndef EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_MAPS_MANAGER_HPP_
#define EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_MAPS_MANAGER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "easynav_core/MapsManagerBase.hpp"
#include "octomap/octomap.h"

#include "easynav_octomap_maps_manager/filters/OctomapFilter.hpp"
#include "pluginlib/class_loader.hpp"

#include "yaets/tracing.hpp"

namespace easynav
{
namespace octomap
{

/**
 * @class OctomapMapsManager
 * @brief A plugin-based map manager using the SimpleMap data structure.
 *
 * This manager implements a minimal mapping approach using boolean grid maps
 * (SimpleMap) for both static and dynamic maps. It supports publishing and
 * receiving ROS occupancy grids.
 */
class OctomapMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  OctomapMapsManager();

  /**
   * @brief Destructor.
   */
  ~OctomapMapsManager();

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

protected:
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;

private:
  /**
   * @brief Internal static map.
   */
  std::shared_ptr<::octomap::OcTree> octomap_;

  /**
   * @brief Publisher for the octomap.
   */
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

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
  octomap_msgs::msg::Octomap octomap_msg_;

  std::unique_ptr<pluginlib::ClassLoader<OctomapFilter>> octomap_filters_loader_;

  std::vector<std::shared_ptr<OctomapFilter>> octomap_filters_;

  double resolution_ {1.0};
};

}  // namespace octomap
}  // namespace easynav
#endif  // EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_MAPS_MANAGER_HPP_
