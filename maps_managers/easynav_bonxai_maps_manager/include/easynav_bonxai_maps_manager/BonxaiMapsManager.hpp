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
/// \brief Declaration of the BonxaiMapsManager method.

#ifndef EASYNAV_BONXAI_MAPS_MANAGER__BONXAI_MAPS_MANAGER_HPP_
#define EASYNAV_BONXAI_MAPS_MANAGER__BONXAI_MAPS_MANAGER_HPP_

#include "bonxai/probabilistic_map.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_core/MapsManagerBase.hpp"

namespace easynav_bonxai
{


/**
 * @class BonxaiMapsManager
 * @brief A plugin-based map manager using the SimpleMap data structure.
 *
 * This manager implements a minimal mapping approach using boolean grid maps
 * (SimpleMap) for both static and dynamic maps. It supports publishing and
 * receiving ROS occupancy grids.
 */
class BonxaiMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  BonxaiMapsManager();

  /**
   * @brief Destructor.
   */
  ~BonxaiMapsManager();

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
  std::shared_ptr<Bonxai::ProbabilisticMap> bonxai_map_;

  /**
   * @brief Publisher for the bonxai.
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bonxai_pub_;

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
  sensor_msgs::msg::PointCloud2 bonxai_msg_;

  void update_from_pc2(const sensor_msgs::msg::PointCloud2 & pc2);
  void update_from_occ(const nav_msgs::msg::OccupancyGrid & occ);
  void publish_map();

  double resolution_ {0.3};
  double height_with_occ_ {1.0};
};

}  // namespace easynav_bonxai
#endif  // EASYNAV_BONXAI_MAPS_MANAGER__BONXAI_MAPS_MANAGER_HPP_
