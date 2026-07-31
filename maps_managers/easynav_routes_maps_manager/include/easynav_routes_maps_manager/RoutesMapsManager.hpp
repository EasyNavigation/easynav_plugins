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
/// \brief Declaration of the RoutesMapsManager class and related types.

#ifndef EASYNAV_PLANNER__ROUTESMAPMANAGER_HPP_
#define EASYNAV_PLANNER__ROUTESMAPMANAGER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

#include "std_srvs/srv/trigger.hpp"

#include <memory>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "easynav_core/MapsManagerBase.hpp"

#include "easynav_routes_maps_manager/RoutesFilter.hpp"

namespace easynav
{

/// @brief Simple directed segment between two poses.
///
/// Each RouteSegment represents a straight-line connection between two
/// poses in the navigation frame. The segment can be individually
/// edited and identified via its @ref id field.
struct RouteSegment
{
  /// @brief Unique identifier for this segment.
  std::string id;

  /// @brief Start pose of the segment.
  geometry_msgs::msg::Pose start;

  /// @brief End pose of the segment.
  geometry_msgs::msg::Pose end;

  /// @brief Whether this segment is currently in edit mode.
  bool edit_mode{false};
};

/// @brief Container type representing a full set of navigation routes.
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

  /// @brief Access to the loaded routes (for testing and tools).
  const RoutesMap & get_routes() const {return routes_;}

private:
  /// @brief Load routes from the configured YAML file into @ref routes_.
  ///
  /// When the configured YAML file is missing, invalid, or the
  /// "routes" key is not present, a default map with a single segment
  /// from (0, 0, 0) to (1, 0, 0) is created instead.
  void load_routes_from_yaml();

  /// @brief Publish the current routes as visualization markers.
  void publish_routes_markers();

  /// @brief Publish or update interactive markers for editing endpoints.
  void publish_interactive_markers();

  /// @brief Handle feedback from interactive markers and update @ref routes_.
  void handle_interactive_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  /// @brief Full path to the YAML file used to load/save routes.
  std::string map_path_;

  /// @brief In-memory collection of route segments.
  RoutesMap routes_;

  /// @brief Monotonic counter for generating unique route IDs.
  int next_route_id_{0};

  /// @brief Plugin loader for route filters.
  std::unique_ptr<pluginlib::ClassLoader<RoutesFilter>> routes_filters_loader_;

  /// @brief Loaded route filters.
  std::vector<std::shared_ptr<RoutesFilter>> routes_filters_;

  /// @brief Publisher for visualizing routes as markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr routes_pub_;

  /// @brief Interactive marker server for editing endpoints.
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;

  /// @brief Service for saving current routes back to disk.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_routes_srv_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__ROUTESMAPMANAGER_HPP_
