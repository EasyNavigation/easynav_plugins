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
/// \brief Declaration of RouteSegment/RoutesMap, split out of
/// RoutesMapsManager.hpp so that code needing only the plain data types
/// (e.g. route_io.hpp, or an external publisher) doesn't have to pull in
/// RoutesMapsManager's own heavier dependencies (pluginlib,
/// interactive_markers, ...).

#ifndef EASYNAV_ROUTES_MAPS_MANAGER__ROUTES_MAP_HPP_
#define EASYNAV_ROUTES_MAPS_MANAGER__ROUTES_MAP_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

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

}  // namespace easynav

#endif  // EASYNAV_ROUTES_MAPS_MANAGER__ROUTES_MAP_HPP_
