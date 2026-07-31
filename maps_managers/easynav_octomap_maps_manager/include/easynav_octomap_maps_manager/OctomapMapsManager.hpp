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
/// \brief Declaration of the OctomapMapsManager method.

#ifndef EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_MAPS_MANAGER_HPP_
#define EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_MAPS_MANAGER_HPP_

#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "octomap/octomap.h"

#include "easynav_octomap_maps_manager/filters/OctomapFilter.hpp"
#include "pluginlib/class_loader.hpp"

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
   * @throws std::runtime_error if initialization fails.
   */
  virtual void on_initialize() override;

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
