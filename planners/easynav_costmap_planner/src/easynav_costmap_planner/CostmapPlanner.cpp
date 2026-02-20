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
/// \brief Implementation of the CostmapPlanner class using A* on Costmap2D.

#include <queue>
#include <unordered_map>
#include <cmath>
#include <tuple>

#include "easynav_costmap_planner/CostmapPlanner.hpp"
#include "easynav_common/RTTFBuffer.hpp"

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

// Simple path smoother: moving average over a sliding window in XY.
// Keeps endpoints unchanged to preserve exact start and goal.
static void smooth_path(std::vector<geometry_msgs::msg::Pose> & poses, int window_size = 5)
{
  if (poses.size() < 3 || window_size <= 1) {
    return;
  }

  // Ensure window_size is odd and at least 3
  if (window_size < 3) {
    window_size = 3;
  }
  if (window_size % 2 == 0) {
    window_size += 1;
  }

  const int half = window_size / 2;
  const size_t n = poses.size();
  std::vector<geometry_msgs::msg::Pose> original = poses;

  // Leave first and last pose untouched
  for (size_t i = 1; i + 1 < n; ++i) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;

    const int begin = static_cast<int>(std::max<size_t>(0,
        i > static_cast<size_t>(half) ? i - half : 0));
    const int end = static_cast<int>(std::min<size_t>(n - 1, i + half));

    for (int j = begin; j <= end; ++j) {
      sum_x += original[j].position.x;
      sum_y += original[j].position.y;
      ++count;
    }

    if (count > 0) {
      poses[i].position.x = sum_x / static_cast<double>(count);
      poses[i].position.y = sum_y / static_cast<double>(count);
    }
  }
}

CostmapPlanner::CostmapPlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;
      ret << "{ " << rclcpp::Time(path.header.stamp).seconds() << " } Path with " <<
        path.poses.size() << " poses and length "
          << compute_path_length(path) << " m.";
      return ret.str();
    });
}

void CostmapPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  node->declare_parameter<double>(plugin_name + ".cost_factor", 2.0);
  node->declare_parameter<double>(plugin_name + ".inflation_penalty", 5.0);
  node->declare_parameter<double>(plugin_name + ".heuristic_scale", 1.0);
  node->declare_parameter<bool>(plugin_name + ".continuous_replan", true);

  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  node->get_parameter(plugin_name + ".inflation_penalty", inflation_penalty_);
  node->get_parameter(plugin_name + ".heuristic_scale", heuristic_scale_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
}

void CostmapPlanner::update(NavState & nav_state)
{
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map.dynamic")) {
    return;
  }

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }

  const auto & map = nav_state.get<Costmap2D>("map.dynamic");
  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  rclcpp::Time latest_stamp = nav_state.get<rclcpp::Time>("map_time");
  if (rclcpp::Time(robot_pose.header.stamp, latest_stamp.get_clock_type()) > latest_stamp) {
    latest_stamp = robot_pose.header.stamp;
  }
  if (rclcpp::Time(goals.goals.front().header.stamp,
      latest_stamp.get_clock_type()) > latest_stamp)
  {
    latest_stamp = goals.goals.front().header.stamp;
  }
  current_path_.header.stamp = latest_stamp;

  if (goals.header.frame_id != tf_info.map_frame) {
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

  // Lightweight skip: if inputs unchanged, avoid recomputation for a short window
  static int last_sx = -1;
  static int last_sy = -1;
  static geometry_msgs::msg::Pose last_goal_pose;
  static rclcpp::Time last_plan_time;

  unsigned int sx_chk, sy_chk;
  if (map.worldToMap(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, sx_chk,
      sy_chk))
  {
    const bool same_start_cell = (static_cast<int>(sx_chk) == last_sx) &&
      (static_cast<int>(sy_chk) == last_sy);
    const bool same_goal_pose = (
      std::fabs(goal.position.x - last_goal_pose.position.x) < 1e-6 &&
      std::fabs(goal.position.y - last_goal_pose.position.y) < 1e-6 &&
      goal.orientation.x == last_goal_pose.orientation.x &&
      goal.orientation.y == last_goal_pose.orientation.y &&
      goal.orientation.z == last_goal_pose.orientation.z &&
      goal.orientation.w == last_goal_pose.orientation.w);

    // Initialize last_plan_time on first use to current node time
    if (last_plan_time.nanoseconds() == 0) {
      last_plan_time = get_node()->now();
    }
    const double since_last = (get_node()->now() - last_plan_time).seconds();
    // Only allow skipping when continuous_replan_ is disabled (event-based planning)
    if (!continuous_replan_ && same_start_cell && same_goal_pose && since_last < 0.05) {
      // Skip re-planning when nothing relevant changed recently
      nav_state.set("path", current_path_);
      return;
    }
  }

  auto poses = a_star_path(map, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    // Apply a light smoothing to the raw grid path
    smooth_path(poses);

    current_path_.poses.clear();
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;
    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = goals.header.frame_id;
      pose_stamped.header.stamp = current_path_.header.stamp;
      pose_stamped.pose = pose;
      current_path_.poses.push_back(pose_stamped);
    }
    // Always publish a newly computed path
    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
    // Update last inputs snapshot
    unsigned int sx, sy;
    if (map.worldToMap(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, sx, sy)) {
      last_sx = static_cast<int>(sx);
      last_sy = static_cast<int>(sy);
    }
    last_goal_pose = goal;
    last_plan_time = get_node()->now();
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
  // Precompute constants used inside the neighbor loop
  const double lethal_norm = 1.0 / static_cast<double>(LETHAL_OBSTACLE);
  const double axial_cost = 1.0;
  const double diagonal_cost = std::sqrt(2.0);
  auto idx = [&](int x, int y) {return y * width + x;};

  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<>> open;

  const int height = map.getSizeInCellsY();
  const int total_cells = width * height;
  std::vector<int> parent_x(total_cells, -1);
  std::vector<int> parent_y(total_cells, -1);
  std::vector<double> cost_so_far(total_cells, std::numeric_limits<double>::infinity());

  const double initial_h = heuristic(static_cast<int>(sx), static_cast<int>(sy),
      static_cast<int>(gx), static_cast<int>(gy)) * heuristic_scale_;
  open.push(GridNode{static_cast<int>(sx), static_cast<int>(sy), 0.0, initial_h});
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
      // Reject cells that would cause collision (>= INSCRIBED_INFLATED_OBSTACLE = 253)
      if (cell_cost >= INSCRIBED_INFLATED_OBSTACLE) {continue;}

      // Calculate traversal cost for free and lightly inflated cells
      double traversal_cost = 1.0 + cost_factor_ * static_cast<double>(cell_cost) * lethal_norm;

      double step_cost = (dx == 0 || dy == 0) ? axial_cost : diagonal_cost;
      double new_cost = cost_so_far[idx(current.x, current.y)] + traversal_cost * step_cost;
      int nid = idx(nx, ny);

      if (new_cost < cost_so_far[nid]) {
        cost_so_far[nid] = new_cost;
        const double h = heuristic(nx, ny, static_cast<int>(gx), static_cast<int>(gy)) *
          heuristic_scale_;
        open.push(GridNode{nx, ny, new_cost, new_cost + h});
        parent_x[nid] = current.x;
        parent_y[nid] = current.y;
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose> path;
  int cx = static_cast<int>(gx), cy = static_cast<int>(gy);
  while (parent_x[idx(cx, cy)] != -1) {
    double wx, wy;
    map.mapToWorld(cx, cy, wx, wy);
    geometry_msgs::msg::Pose pose;
    pose.position.x = wx;
    pose.position.y = wy;
    pose.orientation = goal.orientation;
    path.push_back(pose);
    int px = parent_x[idx(cx, cy)];
    int py = parent_y[idx(cx, cy)];
    cx = px;
    cy = py;
  }
  std::reverse(path.begin(), path.end());

  if (path.empty()) {path.push_back(goal);}
  return path;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapPlanner, easynav::PlannerMethodBase)
