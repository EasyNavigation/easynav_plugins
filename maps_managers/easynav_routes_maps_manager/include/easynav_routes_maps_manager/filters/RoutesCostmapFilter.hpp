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

#ifndef EASYNAV_ROUTES_MAPS_MANAGER_FILTERS_ROUTES_COSTMAP_FILTER_HPP_
#define EASYNAV_ROUTES_MAPS_MANAGER_FILTERS_ROUTES_COSTMAP_FILTER_HPP_

#include <string>

#include "easynav_routes_maps_manager/RoutesFilter.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace easynav
{

/// @brief Costmap filter that raises costs outside navigation routes.
///
/// This filter reads the current RoutesMap and a dynamic costmap
/// ("map.dynamic.filtered") from the NavState. All cells that do not
/// lie within a configurable corridor around any route segment have
/// their cost raised to at least @c min_cost_. A debug occupancy grid
/// representing the filtered costmap is also published.
class RoutesCostmapFilter : public RoutesFilter
{
public:
  /// @brief Default constructor.
  RoutesCostmapFilter();

  /// @brief Configure the costmap filter.
  ///
  /// Declares and reads plugin-specific parameters (such as
  /// @c min_cost and @c route_width), and creates the debug
  /// OccupancyGrid publisher.
  ///
  /// @param node Owning lifecycle node provided by the
  ///   RoutesMapsManager.
  /// @param plugin_ns Namespace used to resolve this filter's
  ///   parameters and topics.
  /// @param tf_info TF frame information used by the navigation stack.
  /// @throws std::runtime_error if initialization fails.
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_ns) override;

  /// @brief Apply the routes-based filtering to the costmap.
  ///
  /// If both "routes" and "map.dynamic.filtered" entries are present
  /// in the NavState, the costmap is modified in place and the debug
  /// occupancy grid is published. If either entry is missing, the
  /// function returns without making changes.
  ///
  /// @param nav_state Current navigation state containing the routes
  ///   and costmap to be filtered.
  void update(NavState & nav_state) override;

private:
  /// @brief Weak reference to the owning lifecycle node.
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  /// @brief Plugin namespace used for parameters and topics.
  std::string plugin_ns_;

  /// @brief Minimum cost enforced outside the route corridor.
  int min_cost_{50};

  /// @brief Corridor half-width around route segments in meters.
  ///
  /// When set to @c 0.0, a default corresponding to half of the
  /// costmap cell diagonal is used.
  double route_width_{0.0};

  /// @brief Publisher for the debug occupancy grid representing the
  /// filtered costmap.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr routes_occ_pub_;

  /// @brief Reusable message used to publish the debug grid.
  nav_msgs::msg::OccupancyGrid routes_grid_msg_;
};

}  // namespace easynav

#endif  // EASYNAV_ROUTES_MAPS_MANAGER_FILTERS_ROUTES_COSTMAP_FILTER_HPP_
