// Copyright 2026 Intelligent Robotics Lab
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

#include "easynav_bonxai_planner/BonxaiPlanner.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "bonxai/probabilistic_map.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void BonxaiPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  auto param = [&](const std::string & name, auto & value) {
      using T = std::decay_t<decltype(value)>;
      easynav::declare_parameter_if_absent<T>(*node, plugin_name + "." + name, value);
      node->get_parameter<T>(plugin_name + "." + name, value);
    };
  int max_expansions = static_cast<int>(params_.max_expansions);
  param("resolution", resolution_);
  param("margin", margin_);
  param("min_clearance", params_.min_clearance);
  param("preferred_clearance", params_.preferred_clearance);
  param("clearance_weight", params_.clearance_weight);
  param("snap_radius", params_.snap_radius);
  param("max_expansions", max_expansions);
  param("min_altitude", min_altitude_);
  param("max_altitude", max_altitude_);
  param("path_step", path_step_);
  param("smooth_iterations", smoother_.iterations);
  param("smooth_weight", smoother_.smooth_weight);
  param("smooth_data_weight", smoother_.data_weight);
  param("continuous_replan", continuous_replan_);
  std::string fill_below = "lowest";
  param("fill_below", fill_below);
  if (!bonxai_planner::parse_fill_below(fill_below, fill_below_)) {
    throw std::runtime_error(
            "[" + plugin_name + "] fill_below '" + fill_below + "' is not none, lowest or highest");
  }
  params_.max_expansions = static_cast<std::size_t>(std::max(1, max_expansions));
  if (!(resolution_ > 0.0) || margin_ < 0.0 || params_.min_clearance < 0.0 ||
    params_.preferred_clearance < 0.0 || params_.clearance_weight < 0.0 ||
    !(path_step_ > 0.0) || smoother_.iterations < 0 || smoother_.smooth_weight < 0.0 ||
    smoother_.smooth_weight > 1.0 || smoother_.data_weight < 0.0 || smoother_.data_weight > 1.0)
  {
    throw std::runtime_error(
            "[" + plugin_name + "] invalid parameters: resolution > 0, path_step > 0, "
            "margin, clearances and clearance_weight >= 0, smooth_iterations >= 0, "
            "smooth_weight and smooth_data_weight in [0, 1]");
  }
  map_ptr_ = nullptr;
  map_voxels_ = 0;
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") +
    plugin_name + "/path", 10);
}

bool BonxaiPlanner::refresh_grid(const NavState & nav_state)
{
  const auto & map = nav_state.get_ptr<Bonxai::ProbabilisticMap>("map.bonxai");
  std::vector<Bonxai::CoordT> coords;
  map->getOccupiedVoxels(coords);
  if (map.get() == map_ptr_ && coords.size() == map_voxels_ && !grid_.empty()) {
    return true;
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(coords.size());
  for (const auto & c : coords) {
    const auto p = map->grid().coordToPos(c);
    points.emplace_back(p.x, p.y, p.z);
  }
  const auto start = std::chrono::steady_clock::now();
  if (!grid_.build(points, resolution_, margin_, min_altitude_, max_altitude_, fill_below_)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "[%s] cannot build the clearance grid (%zu occupied voxels): empty map, or too large for "
      "resolution %.2f", get_plugin_name().c_str(), points.size(), resolution_);
    return false;
  }
  map_ptr_ = map.get();
  map_voxels_ = coords.size();
  RCLCPP_INFO(
    get_node()->get_logger(), "[%s] clearance grid %dx%dx%d at %.2f m from %zu voxels in %.0f ms",
    get_plugin_name().c_str(), grid_.nx(), grid_.ny(), grid_.nz(), resolution_, points.size(),
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  return true;
}

void BonxaiPlanner::update(NavState & nav_state)
{
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map.bonxai")) {
    return;
  }
  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    clear_path(nav_state);
    return;
  }
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  if (goals.header.frame_id != tf_info.map_frame) {
    RCLCPP_WARN(
      get_node()->get_logger(), "[%s] goals frame is not '%s': %s", get_plugin_name().c_str(),
      tf_info.map_frame.c_str(), goals.header.frame_id.c_str());
    clear_path(nav_state);
    return;
  }
  const auto & goal = goals.goals.front().pose;
  if (!continuous_replan_ && !current_path_.poses.empty() && goal == current_goal_) {
    nav_state.set("path", current_path_);
    return;
  }
  if (!refresh_grid(nav_state)) {
    clear_path(nav_state);
    return;
  }

  const auto & robot = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  const Eigen::Vector3d start(robot.position.x, robot.position.y, robot.position.z);
  const Eigen::Vector3d target(goal.position.x, goal.position.y, goal.position.z);
  bonxai_planner::PlanStatus status;
  const auto points = bonxai_planner::plan(grid_, start, target, params_, &status);
  current_goal_ = goal;
  if (points.size() < 2) {
    const char * why = status == bonxai_planner::PlanStatus::StartBlocked ? "the start is" :
      status == bonxai_planner::PlanStatus::GoalBlocked ? "the goal is" : "nothing is";
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "[%s] no path from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f): %s at %.1f m from the "
      "obstacles within %.1f m", get_plugin_name().c_str(), start.x(), start.y(), start.z(),
      target.x(), target.y(), target.z(), why, params_.min_clearance, params_.snap_radius);
    clear_path(nav_state);
    return;
  }

  // Smoothed and evenly spaced poses, facing along the path; the last one, the goal's heading
  const auto dense = bonxai_planner::smooth(
    grid_, points, path_step_, params_.min_clearance, smoother_);
  current_path_.header.frame_id = goals.header.frame_id;
  current_path_.header.stamp = get_node()->now();
  current_path_.poses.clear();
  double yaw = 0.0;
  for (std::size_t i = 0; i + 1 < dense.size(); ++i) {
    // The tangent: from the previous point to the next one
    const Eigen::Vector3d d = dense[i + 1] - dense[i > 0 ? i - 1 : 0];
    if (d.head<2>().norm() > 1e-6) {
      yaw = std::atan2(d.y(), d.x());
    } else if (i == 0) {
      yaw = 2.0 * std::atan2(robot.orientation.z, robot.orientation.w);  // straight up: as now
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header = current_path_.header;
    ps.pose.position.x = dense[i].x();
    ps.pose.position.y = dense[i].y();
    ps.pose.position.z = dense[i].z();
    ps.pose.orientation.z = std::sin(0.5 * yaw);
    ps.pose.orientation.w = std::cos(0.5 * yaw);
    current_path_.poses.push_back(ps);
  }
  geometry_msgs::msg::PoseStamped last;
  last.header = current_path_.header;
  last.pose.position.x = points.back().x();
  last.pose.position.y = points.back().y();
  last.pose.position.z = points.back().z();
  last.pose.orientation = goal.orientation;
  current_path_.poses.push_back(last);

  nav_state.set("path", current_path_);
  if (path_pub_->get_subscription_count() > 0) {
    path_pub_->publish(current_path_);
    path_published_ = true;
  }
}

void BonxaiPlanner::clear_path(NavState & nav_state)
{
  current_path_.poses.clear();
  if (path_published_ && path_pub_->get_subscription_count() > 0) {
    current_path_.header.stamp = get_node()->now();
    path_pub_->publish(current_path_);
  }
  path_published_ = false;
  nav_state.set("path", current_path_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::BonxaiPlanner, easynav::PlannerMethodBase)
