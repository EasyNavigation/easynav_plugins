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

#include "easynav_routes_maps_manager/filters/RoutesCostmapFilter.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "easynav_common/types/NavState.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"

#include "easynav_routes_maps_manager/RoutesMapsManager.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

RoutesCostmapFilter::RoutesCostmapFilter() = default;

void
RoutesCostmapFilter::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_ns)
{
  node_ = node;
  plugin_ns_ = plugin_ns;
  // Parameter for minimum cost to apply outside routes
  if (!node->has_parameter(plugin_ns_ + ".min_cost")) {
    node->declare_parameter(plugin_ns_ + ".min_cost", min_cost_);
  }
  node->get_parameter(plugin_ns_ + ".min_cost", min_cost_);

  // Width of the route corridor around the segment (meters).
  // If set to 0.0 (default), a width of one cell is used.
  if (!node->has_parameter(plugin_ns_ + ".route_width")) {
    node->declare_parameter(plugin_ns_ + ".route_width", route_width_);
  }
  node->get_parameter(plugin_ns_ + ".route_width", route_width_);

  // Publisher for the routes-influenced occupancy grid (for debugging/visualization)
  // Topic: /<maps_manager_node>/<plugin_name>/routes_map
  // plugin_ns_ is typically "<plugin_name>.<filter_name>", so we extract the
  // base plugin_name (e.g. "routes" from "routes.routes_costmap").
  const auto dot_pos = plugin_ns_.find('.');
  const std::string plugin_name =
    (dot_pos == std::string::npos) ? plugin_ns_ : plugin_ns_.substr(0, dot_pos);

  routes_occ_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/routes_map",
    rclcpp::QoS(1).reliable());
}

void
RoutesCostmapFilter::update(NavState & nav_state)
{
  if (!nav_state.has("routes")) {
    return;
  }

  if (!nav_state.has("map.dynamic.filtered")) {
    return;
  }

  const auto & routes = nav_state.get<RoutesMap>("routes");

  auto costmap_ptr = nav_state.get_ptr<Costmap2D>("map.dynamic.filtered");
  if (!costmap_ptr) {
    return;
  }

  Costmap2D & costmap = *costmap_ptr;

  const double resolution = costmap.getResolution();
  const double origin_x = costmap.getOriginX();
  const double origin_y = costmap.getOriginY();

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  // Precompute a list of segments in world coordinates
  struct Segment2D
  {
    double x0, y0, x1, y1;
  };

  std::vector<Segment2D> segments;
  segments.reserve(routes.size());
  for (const auto & seg : routes) {
    segments.push_back({
        seg.start.position.x,
        seg.start.position.y,
        seg.end.position.x,
        seg.end.position.y});
  }

  auto cellHasRoute = [&segments, resolution, origin_x, origin_y, this](
    unsigned int mx, unsigned int my) -> bool
    {
      // Cell center in world coordinates
      const double cx = origin_x + (mx + 0.5) * resolution;
      const double cy = origin_y + (my + 0.5) * resolution;

      for (const auto & s : segments) {
        // Distance from (cx,cy) to segment (s.x0,s.y0)-(s.x1,s.y1)
        const double vx = s.x1 - s.x0;
        const double vy = s.y1 - s.y0;
        const double wx = cx - s.x0;
        const double wy = cy - s.y0;

        const double c1 = vx * wx + vy * wy;
        const double c2 = vx * vx + vy * vy;

        // Allow projection also before the start point (t >= 0)
        if (c2 <= 0.0) {
          continue;
        }
        const double t = c1 / c2;
        if (t < 0.0 || t > 1.0) {
          continue;
        }

        const double proj_x = s.x0 + t * vx;
        const double proj_y = s.y0 + t * vy;

        const double dx = cx - proj_x;
        const double dy = cy - proj_y;
        const double dist2 = dx * dx + dy * dy;

        // Consider the cell as on the route if the distance is less than
        // the configured route width (or, if zero, half the cell diagonal).
        double max_dist = route_width_;
        if (max_dist <= 0.0) {
          max_dist = std::sqrt(2.0) * resolution * 0.5;
        }

        // Make the boundary slightly inclusive with a small epsilon to
        // avoid dropping edge cells due to numerical issues.
        constexpr double epsilon = 1e-6;
        if (dist2 <= (max_dist * max_dist) + epsilon) {
          return true;
        }
      }

      return false;
    };

  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      if (cellHasRoute(x, y)) {
        // Do not modify cost on route
        continue;
      }

      unsigned char old_cost = costmap.getCost(x, y);
      if (old_cost < static_cast<unsigned char>(min_cost_)) {
        costmap.setCost(x, y, static_cast<unsigned char>(min_cost_));
      }
    }
  }

  costmap.toOccupancyGridMsg(routes_grid_msg_);
  routes_grid_msg_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  if (auto node_locked = node_.lock()) {
    routes_grid_msg_.header.stamp = node_locked->now();
  }
  routes_occ_pub_->publish(routes_grid_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::RoutesCostmapFilter, easynav::RoutesFilter)
