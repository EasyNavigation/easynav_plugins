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

#include "easynav_mppi_controller/MPPIController.hpp"
#include "easynav_common/types/PointPerception.hpp"
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

  node->declare_parameter<int>(plugin_name + ".num_samples", num_samples_);
  node->declare_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->declare_parameter<double>(plugin_name + ".dt", dt_);
  node->declare_parameter<double>(plugin_name + ".lambda", lambda_);
  node->declare_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->declare_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);
  node->declare_parameter<double>(plugin_name + ".max_linear_acceleration", max_lin_acc_);
  node->declare_parameter<double>(plugin_name + ".max_angular_acceleration", max_ang_acc_);
  node->declare_parameter<double>(plugin_name + ".fov", fov_);
  node->declare_parameter<double>(plugin_name + ".safety_radius", safety_radius_);

  node->get_parameter<int>(plugin_name + ".num_samples", num_samples_);
  node->get_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->get_parameter<double>(plugin_name + ".dt", dt_);
  node->get_parameter<double>(plugin_name + ".lambda", lambda_);
  node->get_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->get_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);
  node->get_parameter<double>(plugin_name + ".max_linear_acceleration", max_lin_acc_);
  node->get_parameter<double>(plugin_name + ".max_angular_acceleration", max_ang_acc_);
  node->get_parameter<double>(plugin_name + ".fov", fov_);
  node->get_parameter<double>(plugin_name + ".safety_radius", safety_radius_);

  optimizer_ = std::make_unique<MPPIOptimizer>(num_samples_, horizon_steps_, dt_, lambda_,
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


void
MPPIController::update_rt(NavState & nav_state)
{
  // If navigation is IDLE, force zero velocity
  if (nav_state.has("navigation_state")) {
    const auto nav_state_val = nav_state.get<easynav::GoalManager::State>("navigation_state");
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

  const auto & path = nav_state.get<nav_msgs::msg::Path>("path");

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

  const auto & pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const auto & filtered = PointPerceptionsOpsView(perceptions)
    .filter({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0})
    .fuse(tf_info.map_frame)
    .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .collapse({NAN, NAN, 0.1})
    .downsample(0.1)
    .as_points();

  if (filtered.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
        "No valid points available for MPPI optimization, using the path only.");
  }

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
