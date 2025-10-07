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
/// \brief Implementation of the AStarPlanner class using A* on ::navmap::NavMap (triangle graph).

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <tuple>
#include <optional>
#include <algorithm>

#include "easynav_navmap_planner/AStarPlanner.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navmap_core/NavMap.hpp"

namespace easynav
{
namespace navmap
{

static constexpr uint8_t NO_INFORMATION = 255;
static constexpr uint8_t LETHAL_OBSTACLE = 254;
static constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr uint8_t MAX_NON_OBSTACLE = 252;
static constexpr uint8_t FREE_SPACE = 0;

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

AStarPlanner::AStarPlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;
      ret << "Path with " << path.poses.size() << " poses and length "
          << compute_path_length(path) << " m.";
      return ret.str();
    });
}

std::expected<void, std::string> AStarPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<std::string>(plugin_name + ".layer", "inflated_obstacles");
  node->declare_parameter<double>(plugin_name + ".cost_factor", 2.0);
  node->declare_parameter<double>(plugin_name + ".inflation_penalty", 5.0);
  node->declare_parameter<bool>(plugin_name + ".continuous_replan", true);

  node->get_parameter(plugin_name + ".layer", layer_name_);
  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  node->get_parameter(plugin_name + ".inflation_penalty", inflation_penalty_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
  return {};
}

void AStarPlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map")) {
    return;
  }

  const auto goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }

  navmap_ = nav_state.get<::navmap::NavMap>("map");

  const auto robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;

  if (goals.header.frame_id != get_tf_prefix() + "map") {
    RCLCPP_WARN(get_node()->get_logger(), "Goals frame is not 'map': %s",
                goals.header.frame_id.c_str());
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

  auto poses = a_star_path(navmap_, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;
    current_path_.poses.reserve(poses.size());
    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = goals.header.frame_id;
      ps.header.stamp = current_path_.header.stamp;
      ps.pose = pose;
      current_path_.poses.push_back(std::move(ps));
    }
    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  }
  nav_state.set("path", current_path_);
}

std::vector<geometry_msgs::msg::Pose> AStarPlanner::a_star_path(
  const ::navmap::NavMap & nm,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  using ::navmap::NavCelId;

  if (nm.navcels.empty()) {return {};}

  // 1) Locate start & goal navcels (fallback to closest triangle)
  size_t sidx_s = 0, sidx_g = 0;
  NavCelId cid_start = 0, cid_goal = 0;
  Eigen::Vector3f bary; Eigen::Vector3f hit;

  Eigen::Vector3f pS(start.position.x, start.position.y, start.position.z);
  Eigen::Vector3f pG(goal.position.x, goal.position.y, goal.position.z);

  bool okS = nm.locate_navcel(pS, sidx_s, cid_start, bary, &hit);
  if (!okS) {
    Eigen::Vector3f q; float d2;
    if (!nm.closest_navcel(pS, sidx_s, cid_start, q, d2)) {return {};}
  }
  bool okG = nm.locate_navcel(pG, sidx_g, cid_goal, bary, &hit);
  if (!okG) {
    Eigen::Vector3f q; float d2;
    if (!nm.closest_navcel(pG, sidx_g, cid_goal, q, d2)) {return {};}
  }

  // 2) Choose layer to query
  std::string layer = layer_name_;
  if (!nm.has_layer(layer)) {
    if (!nm.has_layer(layer)) {
      RCLCPP_WARN(get_node()->get_logger(), "No layer '%s' nor 'obstacles' found",
            layer_name_.c_str());
      return {};
    }
  }

  // Impassable predicate
  auto blocked = [&](NavCelId c) -> bool {
      uint8_t v = nm.layer_get<uint8_t>(layer, c, FREE_SPACE);
      return (v == NO_INFORMATION) || (v >= LETHAL_OBSTACLE);
    };
  if (blocked(cid_start) || blocked(cid_goal)) {
    return {};
  }

  const size_t N = nm.navcels.size();

  // 3) Precompute centroids (2D) for consistent metric and heuristic
  std::vector<Eigen::Vector2f> C(N);
  for (NavCelId c = 0; c < N; ++c) {
    const auto cc = nm.navcel_centroid(c);
    C[c] = {cc.x(), cc.y()};
  }

  auto h = [&](NavCelId a, NavCelId b) -> double {
      const auto d = C[a] - C[b];
      return static_cast<double>(d.norm());
    };

  auto step_cost = [&](NavCelId from, NavCelId to) -> double {
      const double dist = static_cast<double>((C[from] - C[to]).norm());
      const uint8_t cost_to = nm.layer_get<uint8_t>(layer, to, FREE_SPACE);
      if (cost_to >= LETHAL_OBSTACLE || cost_to == NO_INFORMATION) {
        return std::numeric_limits<double>::infinity();
      }
      const double norm = static_cast<double>(cost_to) / static_cast<double>(LETHAL_OBSTACLE);
      double c = dist * (1.0 + cost_factor_ * norm);
      if (cost_to >= INSCRIBED_INFLATED_OBSTACLE) {
      // extra penalty near obstacles
        const double ratio =
          static_cast<double>(cost_to - INSCRIBED_INFLATED_OBSTACLE) /
          static_cast<double>(LETHAL_OBSTACLE - INSCRIBED_INFLATED_OBSTACLE);
        c += inflation_penalty_ * std::max(0.0, ratio);
      }
      return c;
    };

  // 4) A* on triangle graph
  struct Node { NavCelId cid; double f; };
  struct Cmp { bool operator()(const Node & a, const Node & b) const {return a.f > b.f;} };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::vector<double> g(N, std::numeric_limits<double>::infinity());
  std::vector<uint8_t> in_open(N, 0);
  std::vector<NavCelId> parent(N, std::numeric_limits<NavCelId>::max());

  g[cid_start] = 0.0;
  open.push(Node{cid_start, h(cid_start, cid_goal)});
  in_open[cid_start] = 1;

  const size_t surface_goal = sidx_g;

  while (!open.empty()) {
    const auto cur = open.top(); open.pop();
    const NavCelId u = cur.cid;

    if (u == cid_goal) {break;}

    // Optional: restrict to the goal surface (comment if you want cross-surface paths via explicit neighbors)
    // if (surface_goal != sidx_s) {
    //   // do nothing special; graph neighbors already encode connectivity
    // }

    for (NavCelId v : nm.navcel_neighbors(u)) {
      const size_t vidx = static_cast<size_t>(v);
      if (vidx >= N) {continue;}
      if (blocked(v)) {continue;}

      const double sc = step_cost(u, v);
      if (!std::isfinite(sc)) {continue;}

      const double tentative = g[u] + sc;
      if (tentative < g[v]) {
        g[v] = tentative;
        parent[v] = u;
        const double f = tentative + h(v, cid_goal);
        open.push(Node{v, f});
        in_open[v] = 1;
      }
    }
  }

  if (!std::isfinite(g[cid_goal])) {
    return {};
  }

  // 5) Reconstruct path (centroidal polyline)
  std::vector<geometry_msgs::msg::Pose> path;
  for (NavCelId c = cid_goal; c != std::numeric_limits<NavCelId>::max(); c = parent[c]) {
    geometry_msgs::msg::Pose p;
    p.position.x = C[c].x();
    p.position.y = C[c].y();
    p.position.z = 0.0;
    p.orientation = goal.orientation;
    path.push_back(std::move(p));
    if (c == cid_start) {break;}
  }
  std::reverse(path.begin(), path.end());

  // Ensure at least goal pose
  if (path.empty()) {path.push_back(goal);}
  return path;
}

}  // namespace navmap
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::AStarPlanner, easynav::PlannerMethodBase)
