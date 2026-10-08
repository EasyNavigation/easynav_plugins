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
/// \brief Implementation of the MPPIController class.

#include "easynav_common/Parameters.hpp"
#include "easynav_mppi_controller/MPPIController.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_system/GoalManager.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

MPPIController::MPPIController() {}

MPPIController::~MPPIController() = default;

void
MPPIController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<int>(*node, plugin_name + ".num_samples", num_samples_);
  easynav::declare_parameter_if_absent<int>(*node, plugin_name + ".horizon_steps", horizon_steps_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".dt", dt_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".lambda", lambda_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".fov", fov_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".safety_radius",
    safety_radius_);

  node->get_parameter<int>(plugin_name + ".num_samples", num_samples_);
  node->get_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->get_parameter<double>(plugin_name + ".dt", dt_);
  node->get_parameter<double>(plugin_name + ".lambda", lambda_);
  // Velocity and acceleration limits: the robot's (controller_node "robot_limits.*").
  const auto limits = get_robot_limits(
    {"max_linear_velocity", "", "max_angular_velocity", "max_linear_acceleration", "",
      "max_angular_acceleration", ""});
  max_lin_vel_ = limits.max_linear_vel;
  max_ang_vel_ = limits.max_angular_vel;
  max_lin_acc_ = limits.max_linear_acc;
  max_ang_acc_ = limits.max_angular_acc;
  node->get_parameter<double>(plugin_name + ".fov", fov_);
  node->get_parameter<double>(plugin_name + ".safety_radius", safety_radius_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".obstacle_range", obstacle_range_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".z_min_filter", z_min_filter_);
  node->get_parameter<double>(plugin_name + ".obstacle_range", obstacle_range_);
  node->get_parameter<double>(plugin_name + ".z_min_filter", z_min_filter_);
  const auto geometry = get_robot_geometry();
  robot_radius_ = geometry.radius;
  robot_height_ = geometry.height;

  optimizer_ = std::make_unique<MPPIOptimizer>(
    num_samples_, horizon_steps_, dt_, lambda_,
    max_lin_vel_, max_ang_vel_, fov_, safety_radius_);

  mppi_candidates_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/candidates", 10);
  mppi_optimal_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/optimal_path", 10);
}

void MPPIController::publish_mppi_markers(
  const std::vector<std::vector<std::pair<double, double>>> & all_trajs,
  const std::vector<std::pair<double, double>> & best_traj)
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  visualization_msgs::msg::MarkerArray candidates;
  visualization_msgs::msg::MarkerArray optimal;
  int id = 0;

  // Candidates in blue
  for (const auto & traj : all_trajs) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = tf_info.map_frame;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "mppi_candidates";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;

    for (const auto & [x, y] : traj) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = 0.05;
      marker.points.push_back(p);
    }

    candidates.markers.push_back(marker);
  }

  // Best trajectory in red
  visualization_msgs::msg::Marker best_marker;
  best_marker.header.frame_id = tf_info.map_frame;
  best_marker.header.stamp = rclcpp::Clock().now();
  best_marker.ns = "mppi_optimal_path";
  best_marker.id = id++;
  best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  best_marker.action = visualization_msgs::msg::Marker::ADD;
  best_marker.scale.x = 0.05;
  best_marker.color.r = 1.0;
  best_marker.color.g = 0.0;
  best_marker.color.b = 0.0;
  best_marker.color.a = 0.8;

  for (const auto & [x, y] : best_traj) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.1;
    best_marker.points.push_back(p);
  }

  optimal.markers.push_back(best_marker);

  // Publish the markers
  mppi_candidates_pub_->publish(candidates);
  mppi_optimal_pub_->publish(optimal);
}


pcl::PointCloud<pcl::PointXYZ>
MPPIController::obstacle_points(const NavState & nav_state, bool backward) const
{
  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  // Robot frame: from just behind the robot to obstacle_range ahead (mirrored when backward).
  const double x_min = backward ? -obstacle_range_ : -robot_radius_;
  const double x_max = backward ? robot_radius_ : obstacle_range_;
  // Downsampled first (indices only): fewer points to transform in the filter.
  return PointPerceptionsOpsView(perceptions)
         .downsample(0.1)
         .fuse(tf_info.robot_frame)
         .filter(
    {x_min, -obstacle_range_, z_min_filter_},
    {x_max, obstacle_range_, robot_height_}, false)
         .fuse(tf_info.map_frame)
         .collapse({NAN, NAN, 0.1})
         .downsample(0.1)
         .as_points();
}

void
MPPIController::update_rt(NavState & nav_state)
{
  // If navigation is IDLE, force zero velocity
  if (nav_state.has("navigation_state")) {
    const auto nav_state_val = nav_state.get_safe<easynav::GoalManager::State>("navigation_state");
    if (nav_state_val == easynav::GoalManager::State::IDLE) {
      twist_stamped_.header.stamp = get_node()->now();
      twist_stamped_.twist.linear.x = 0.0;
      twist_stamped_.twist.angular.z = 0.0;
      nav_state.set("cmd_vel", twist_stamped_);

      // Also clear visualization markers when idle
      visualization_msgs::msg::MarkerArray clear_markers;
      visualization_msgs::msg::Marker delete_all;
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      clear_markers.markers.push_back(delete_all);

      mppi_candidates_pub_->publish(clear_markers);
      mppi_optimal_pub_->publish(clear_markers);
      return;
    }
  }

  if (!nav_state.has("path") || !nav_state.has("robot_pose")) {
    return;
  }

  const auto & path = nav_state.get_safe<nav_msgs::msg::Path>("path");

  if (path.poses.empty()) {
    // If the path is empty, stop the robot and clear markers
    twist_stamped_.header.frame_id = path.header.frame_id;
    twist_stamped_.header.stamp = get_node()->now();
    twist_stamped_.twist.linear.x = 0.0;
    twist_stamped_.twist.angular.z = 0.0;
    nav_state.set("cmd_vel", twist_stamped_);

    visualization_msgs::msg::MarkerArray clear_markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(delete_all);

    mppi_candidates_pub_->publish(clear_markers);
    mppi_optimal_pub_->publish(clear_markers);
    return;
  }

  const auto pose = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  const bool backward = nav_state.has("cmd_vel") &&
    nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x < 0.0;
  const auto filtered = obstacle_points(nav_state, backward);

  // Compute the control using MPPI with points
  auto result = optimizer_->compute_control(pose, path, filtered);

  // Prevent abrupt changes in velocity
  const auto & current_twist = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  double dv = result.v - current_twist.twist.linear.x;
  double dw = result.w - current_twist.twist.angular.z;
  double max_dv = max_lin_acc_ * dt_;
  double max_dw = max_ang_acc_ * dt_;
  if (std::abs(dv) > max_dv) {
    result.v = current_twist.twist.linear.x + (dv > 0 ? max_dv : -max_dv);
  }
  if (std::abs(dw) > max_dw) {
    result.w = current_twist.twist.angular.z + (dw > 0 ? max_dw : -max_dw);
  }
  result.v = std::clamp(result.v, -max_lin_vel_, max_lin_vel_);
  result.w = std::clamp(result.w, -max_ang_vel_, max_ang_vel_);

  // Publish the computed velocity command
  twist_stamped_.header.frame_id = path.header.frame_id;
  twist_stamped_.header.stamp = get_node()->now();
  twist_stamped_.twist.linear.x = result.v;
  twist_stamped_.twist.angular.z = result.w;

  nav_state.set("cmd_vel", twist_stamped_);

  // Publish the MPPI markers
  publish_mppi_markers(result.all_trajectories, result.best_trajectory);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MPPIController, easynav::ControllerMethodBase)
