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
/// \brief Declaration of the SimpleMapsManager method.

#ifndef EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_
#define EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"

namespace easynav
{

/**
 * @class SimpleMapsManager
 * @brief A plugin-based map manager using the SimpleMap data structure.
 *
 * This manager implements a minimal mapping approach using boolean grid maps
 * (SimpleMap) for both static and dynamic maps. It supports publishing and
 * receiving ROS occupancy grids.
 */
class SimpleMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  SimpleMapsManager();

  /**
   * @brief Destructor.
   */
  ~SimpleMapsManager();

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
  virtual void update(NavState & nav_state) override;

  /**
   * @brief Replaces the current static map.
   *
   * @param new_map Shared pointer to a new map object. Must be of type SimpleMap.
   */
  void set_static_map(const SimpleMap & new_map);

  /**
   * @brief Replaces the current dynamic map.
   *
   * @param new_map Shared pointer to a new map object. Must be of type SimpleMap.
   */
  void set_dynamic_map(const SimpleMap & new_map);

protected:
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;

private:
  /**
   * @brief Internal static map.
   */
  SimpleMap static_map_;

  /**
   * @brief Internal dynamic map.
   */
  SimpleMap dynamic_map_;

  /**
   * @brief Publisher for the static occupancy grid.
   */
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_occ_pub_;

  /**
   * @brief Publisher for the dynamic occupancy grid.
   */
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_occ_pub_;

  /**
   * @brief Subscriber for external incoming static map updates.
   */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr incoming_map_sub_;

  /**
   * @brief Service for saving current map to disk.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

  /**
   * @brief Cached occupancy grid message for the static map.
   */
  nav_msgs::msg::OccupancyGrid static_grid_msg_;

  /**
   * @brief Cached occupancy grid message for the dynamic map.
   */
  nav_msgs::msg::OccupancyGrid dynamic_grid_msg_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_
