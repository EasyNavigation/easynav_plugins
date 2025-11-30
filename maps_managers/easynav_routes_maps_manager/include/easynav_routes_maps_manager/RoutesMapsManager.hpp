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
/// \brief Declaration of the RoutesMapsManager method.

#ifndef EASYNAV_PLANNER__ROUTESMAPMANAGER_HPP_
#define EASYNAV_PLANNER__ROUTESMAPMANAGER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "easynav_core/MapsManagerBase.hpp"

namespace easynav
{

struct RouteSegment
{
  geometry_msgs::msg::Pose start;
  geometry_msgs::msg::Pose end;
};

using RoutesMap = std::vector<RouteSegment>;

/**
 * @class RoutesMapsManager
 * @brief A plugin-based map manager using the RoutesMap data structure.
 *
 * This manager implements a minimal mapping approach using a set of predefined
 * navigation routes (RoutesMap). Routes are defined as straight-line segments
 * between two poses and are loaded from a YAML file. The segments are
 * visualized using MarkerArray messages.
 */
class RoutesMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  RoutesMapsManager();

  /**
   * @brief Destructor.
   */
  ~RoutesMapsManager();

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

  /// @brief Access to the loaded routes (for testing and tools).
  const RoutesMap & get_routes() const {return routes_;}

private:
  /// @brief Load routes from the YAML file into routes_.
  void load_routes_from_yaml();

  /// @brief Publish the current routes as visualization markers.
  void publish_routes_markers();
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;
  /// @brief In-memory collection of route segments.
  RoutesMap routes_;

  /// @brief Publisher for visualizing routes as markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr routes_pub_;

  /// @brief Service for reloading the routes YAML file.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_routes_srv_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__ROUTESMAPMANAGER_HPP_
