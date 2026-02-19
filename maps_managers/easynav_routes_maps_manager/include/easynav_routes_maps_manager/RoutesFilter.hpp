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

#ifndef EASYNAV_ROUTES_MAPS_MANAGER_ROUTES_FILTER_HPP_
#define EASYNAV_ROUTES_MAPS_MANAGER_ROUTES_FILTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

class NavState;

/// @brief Base interface for all routes filters.
///
/// A RoutesFilter consumes the current navigation state (including the
/// active RoutesMap written by RoutesMapsManager) and is allowed to
/// modify other entries in the NavState (for example, a costmap) in
/// order to enforce corridor-following behaviour, masking, etc.
class RoutesFilter
{
public:
  /// @brief Shared pointer alias for convenience.
  using Ptr = std::shared_ptr<RoutesFilter>;

  /// @brief Virtual destructor.
  virtual ~RoutesFilter() = default;

  /// @brief Configure the filter instance.
  ///
  /// This method is called once after construction by the
  /// RoutesMapsManager. Implementations should declare/read any
  /// required parameters, create publishers or other resources, and
  /// store references to the lifecycle node.
  ///
  /// @param node Shared pointer to the owning lifecycle node.
  /// @param plugin_ns Namespace under which this filter is configured
  ///   (used as prefix for ROS parameters and topics).
  /// @param tf_info TF frame information used by the navigation stack.
  /// @throws std::runtime_error if initialization fails.
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_ns) = 0;

  /// @brief Update hook called every navigation cycle.
  ///
  /// Implementations may read the current "routes" entry from the
  /// NavState as well as other map representations and modify them in
  /// place. The filter must not assume ownership of data stored in the
  /// NavState.
  ///
  /// @param nav_state Blackboard-like navigation state container.
  virtual void update(NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_ROUTES_MAPS_MANAGER_ROUTES_FILTER_HPP_
