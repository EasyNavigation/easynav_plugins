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
/// \brief Implementation of the CostmapPlanner class using A* on Costmap2D.

#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <tuple>

#include "easynav_costmap_planner/CostmapPlanner.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"

namespace easynav
{

struct GridNode
{
  int x, y;
  double cost;
  double priority;
  bool operator>(const GridNode & other) const
  {
    return priority > other.priority;
  }
};

static double heuristic(int x1, int y1, int x2, int y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

static std::vector<std::pair<int, int>> neighbors8 = {
  {-1, -1}, {-1, 0}, {-1, 1},
  {0, -1}, {0, 1},
  {1, -1}, {1, 0}, {1, 1}
};

static double compute_path_length(const nav_msgs::msg::Path & path)
{
  double total_length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;
    total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  return total_length;
}

CostmapPlanner::CostmapPlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;
      ret << "Path with " << path.poses.size() << " poses and length "
          << compute_path_length(path) << " m.";
      return ret.str();
    });
}

std::expected<void, std::string> CostmapPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  node->declare_parameter<double>(plugin_name + ".cost_factor", 2.0);
  node->declare_parameter<double>(plugin_name + ".inflation_penalty", 5.0);
  node->declare_parameter<double>(plugin_name + ".cost_axial", 1.0);
  node->declare_parameter<double>(plugin_name + ".cost_diagonal", 1.41);
  node->declare_parameter<bool>(plugin_name + ".continuous_replan", true);

  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  node->get_parameter(plugin_name + ".inflation_penalty", inflation_penalty_);
  node->get_parameter(plugin_name + ".cost_axial", cost_axial_);
  node->get_parameter(plugin_name + ".cost_diagonal", cost_diagonal_);
  node->get_parameter(plugin_name + ".cost_diagonal", cost_diagonal_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
  return {};
}

void CostmapPlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map.dynamic")) {
    return;
  }

  const auto goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }

  const auto & map = nav_state.get<Costmap2D>("map.dynamic");
  const auto robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;

  if (goals.header.frame_id != get_tf_prefix() + "map") {
    RCLCPP_WARN(get_node()->get_logger(), "Goals frame is not 'map': %s",
        goals.header.frame_id.c_str());
    return;
  }

  unsigned int gx, gy;
  if (!map.worldToMap(goal.position.x, goal.position.y, gx, gy)) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal (%.2f, %.2f) is outside the map", goal.position.x,
        goal.position.y);
    return;
  }

  auto goals_ts = rclcpp::Time(goals.header.stamp);
  if (!continuous_replan_ &&
    goals_ts < rclcpp::Time(current_path_.header.stamp) &&
    goals.goals.front().pose == current_goal_)
  {
    return;
  }

  current_goal_ = goal;

  auto poses = a_star_path(map, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;
    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = goals.header.frame_id;
      pose_stamped.header.stamp = current_path_.header.stamp;
      pose_stamped.pose = pose;
      current_path_.poses.push_back(pose_stamped);
    }
    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  }
  nav_state.set("path", current_path_);
}

std::vector<geometry_msgs::msg::Pose> CostmapPlanner::a_star_path(
  const Costmap2D & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  unsigned int sx, sy, gx, gy;
  if (!map.worldToMap(start.position.x, start.position.y, sx, sy)) {return {};}
  if (!map.worldToMap(goal.position.x, goal.position.y, gx, gy)) {return {};}

  int width = map.getSizeInCellsX();
  auto idx = [&](int x, int y) {return y * width + x;};

  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<>> open;
  std::unordered_map<int, std::pair<int, int>> came_from;
  std::unordered_map<int, double> cost_so_far;

  open.push(GridNode{static_cast<int>(sx), static_cast<int>(sy), 0.0, heuristic(sx, sy, gx, gy)});
  cost_so_far[idx(sx, sy)] = 0.0;

  while (!open.empty()) {
    auto current = open.top();
    open.pop();
    if (current.x == static_cast<int>(gx) && current.y == static_cast<int>(gy)) {break;}

    for (auto [dx, dy] : neighbors8) {
      int nx = current.x + dx;
      int ny = current.y + dy;
      if (!map.inBounds(nx, ny)) {continue;}

      uint8_t cell_cost = map.getCost(nx, ny);
      if (cell_cost >= LETHAL_OBSTACLE) {continue;}

      double traversal_cost = 1.0 + cost_factor_ * static_cast<double>(cell_cost) / LETHAL_OBSTACLE;
      if (cell_cost >= INSCRIBED_INFLATED_OBSTACLE) {
        double inflation_ratio = static_cast<double>(cell_cost - INSCRIBED_INFLATED_OBSTACLE) /
          (LETHAL_OBSTACLE - INSCRIBED_INFLATED_OBSTACLE);
        traversal_cost += inflation_penalty_ * inflation_ratio;
      }

      double step_cost = (dx == 0 || dy == 0) ? cost_axial_ : cost_diagonal_;
      double new_cost = cost_so_far[idx(current.x, current.y)] + traversal_cost * step_cost;
      int nid = idx(nx, ny);

      if (!cost_so_far.contains(nid) || new_cost < cost_so_far[nid]) {
        cost_so_far[nid] = new_cost;
        open.push(GridNode{nx, ny, new_cost, new_cost + heuristic(nx, ny, gx, gy)});
        came_from[nid] = {current.x, current.y};
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose> path;
  int cx = static_cast<int>(gx), cy = static_cast<int>(gy);
  while (came_from.contains(idx(cx, cy))) {
    double wx, wy;
    map.mapToWorld(cx, cy, wx, wy);
    geometry_msgs::msg::Pose pose;
    pose.position.x = wx;
    pose.position.y = wy;
    pose.orientation = goal.orientation;
    path.push_back(pose);
    std::tie(cx, cy) = came_from[idx(cx, cy)];
  }
  std::reverse(path.begin(), path.end());

  if (path.empty()) {path.push_back(goal);}
  return path;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapPlanner, easynav::PlannerMethodBase)
