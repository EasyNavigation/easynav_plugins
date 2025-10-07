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
/// \brief Declaration of the SimpleMapsManager method.

#ifndef EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_
#define EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"

#include "yaets/tracing.hpp"

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
