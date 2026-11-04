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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "easynav_common/Parameters.hpp"
#include "easynav_mppi_controller/MPPIController.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_system/GoalManager.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace easynav
{

MPPIController::MPPIController() {}

MPPIController::~MPPIController() = default;

void
MPPIController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  auto param = [&](const std::string & name, auto & value) {
      using T = std::decay_t<decltype(value)>;
      easynav::declare_parameter_if_absent<T>(*node, plugin_name + "." + name, value);
      node->get_parameter<T>(plugin_name + "." + name, value);
    };

  std::string motion_model = "diff";
  std::string heading_mode = "path";
  param("motion_model", motion_model);
  param("heading_mode", heading_mode);
  MotionModel model;
  if (!parse_motion_model(motion_model, model)) {
    throw std::runtime_error(
            "[" + plugin_name + "] motion_model '" + motion_model + "' is not diff or omni");
  }
  MPPIParams p;
  p.motion_model = model;
  int num_samples = p.num_samples;
  int horizon_steps = p.horizon_steps;
  param("heading_yaw", heading_yaw_);
  param("num_samples", num_samples);
  param("horizon_steps", horizon_steps);
  param("dt", p.dt);
  param("lambda", p.lambda);
  param("vx_std", p.vx_std);
  param("vy_std", p.vy_std);
  param("wz_std", p.wz_std);
  param("safety_radius", p.safety_radius);
  param("path_weight", p.path_weight);
  param("progress_weight", p.progress_weight);
  param("heading_weight", p.heading_weight);
  param("heading_lookahead", p.heading_lookahead);
  param("noise_correlation", p.noise_correlation);
  param("goal_weight", p.goal_weight);
  param("goal_heading_weight", p.goal_heading_weight);
  param("goal_heading_distance", p.goal_heading_distance);
  param("obstacle_weight", p.obstacle_weight);
  param("smoothness_weight", p.smoothness_weight);
  param("forward_weight", p.forward_weight);
  param("grid_resolution", p.grid_resolution);
  param("xy_goal_tolerance", xy_goal_tolerance_);
  param("yaw_goal_tolerance", yaw_goal_tolerance_);
  param("obstacle_range", obstacle_range_);
  param("z_min_filter", z_min_filter_);
  param("max_visualized_rollouts", max_visualized_rollouts_);
  p.num_samples = num_samples;
  p.horizon_steps = horizon_steps;

  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count(plugin_name + ".fov") > 0) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] '%s.fov' has no effect any more: remove it",
      plugin_name.c_str(), plugin_name.c_str());
  }

  if (!parse_heading_mode(heading_mode, heading_mode_)) {
    throw std::runtime_error(
            "[" + plugin_name + "] heading_mode '" + heading_mode +
            "' is not path, free, goal, hold or fixed");
  }
  if (p.motion_model == MotionModel::Diff && heading_mode_ != HeadingMode::Path) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] heading_mode '%s' needs motion_model 'omni': a differential "
      "robot faces along the path", plugin_name.c_str(), heading_mode.c_str());
    heading_mode_ = HeadingMode::Path;
  }

  const auto geometry = get_robot_geometry();
  p.robot_radius = geometry.radius;
  p.inscribed_radius = geometry.inscribed_radius;
  p.footprint = geometry.footprint;
  if (!p.footprint.empty()) {
    // The robot's exact shape: the configured radii may be planning margins.
    p.robot_radius = footprint_radius(p.footprint);
    p.inscribed_radius = footprint_inscribed_radius(p.footprint);
  }
  robot_height_ = geometry.height;
  if (p.safety_radius < p.robot_radius) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] safety_radius %.2f is below robot_geometry.radius %.2f: using "
      "the radius", plugin_name.c_str(), p.safety_radius, p.robot_radius);
  }

  // Velocity and acceleration limits: the robot's (controller_node "robot_limits.*").
  limits_ = get_robot_limits(
    {"max_linear_velocity", "", "max_angular_velocity", "max_linear_acceleration", "",
      "max_angular_acceleration", ""});
  params_ = p;
  optimizer_.configure(params_);
  apply_limits(limits_);

  last_cmd_ = Velocity{};
  last_update_.reset();
  have_goal_ = false;
  last_status_ = MPPIStatus::NoPath;

  mppi_candidates_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/candidates", 10);
  mppi_optimal_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/optimal_path", 10);
}

void
MPPIController::apply_limits(const RobotLimits & limits)
{
  limits_ = limits;
  MPPIParams p = optimizer_.params();
  p.max_vx = limits.max_linear_vel;
  p.min_vx = limits.min_linear_vel;
  p.max_vy = limits.max_lateral_vel;
  p.max_wz = limits.max_angular_vel;
  p.acc_vx = limits.max_linear_acc;
  p.decel_vx = limits.max_linear_decel;
  p.acc_vy = limits.max_lateral_acc;
  p.decel_vy = limits.max_lateral_decel;
  p.acc_wz = limits.max_angular_acc;
  p.decel_wz = limits.max_angular_decel;
  optimizer_.set_limits(p);
}

void
MPPIController::publish_markers()
{
  const bool candidates = max_visualized_rollouts_ > 0 &&
    mppi_candidates_pub_->get_subscription_count() > 0 && optimizer_.rollouts() > 0;
  const bool optimal = mppi_optimal_pub_->get_subscription_count() > 0;
  if (!candidates && !optimal) {
    return;
  }
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const auto stamp = get_node()->now();
  const int steps = optimizer_.params().horizon_steps;

  auto line = [&](const std::string & ns, int id, double width, double r, double b, double a) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = tf_info.map_frame;
      marker.header.stamp = stamp;
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = width;
      marker.color.r = r;
      marker.color.b = b;
      marker.color.a = a;
      marker.pose.orientation.w = 1.0;
      return marker;
    };

  if (candidates) {
    visualization_msgs::msg::MarkerArray array;
    const int shown = std::min(max_visualized_rollouts_, optimizer_.rollouts());
    const int stride = std::max(1, optimizer_.rollouts() / shown);
    for (int i = 0; i < shown; ++i) {
      auto marker = line("mppi_candidates", i, 0.02, 0.0, 1.0, 0.5);
      const auto * poses = optimizer_.rollout(i * stride);
      for (int t = 0; t <= steps; ++t) {
        geometry_msgs::msg::Point p;
        p.x = poses[t].x;
        p.y = poses[t].y;
        p.z = 0.05;
        marker.points.push_back(p);
      }
      array.markers.push_back(marker);
    }
    mppi_candidates_pub_->publish(array);
  }

  if (optimal) {
    visualization_msgs::msg::MarkerArray array;
    auto marker = line("mppi_optimal_path", 0, 0.05, 1.0, 0.0, 0.8);
    for (const auto & pose : optimizer_.optimal_trajectory()) {
      geometry_msgs::msg::Point p;
      p.x = pose.x;
      p.y = pose.y;
      p.z = 0.1;
      marker.points.push_back(p);
    }
    array.markers.push_back(marker);
    mppi_optimal_pub_->publish(array);
  }
}

void
MPPIController::clear_markers()
{
  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(delete_all);
  mppi_candidates_pub_->publish(clear);
  mppi_optimal_pub_->publish(clear);
}

void
MPPIController::stop(NavState & nav_state, bool forget_goal)
{
  optimizer_.reset();
  last_cmd_ = Velocity{};
  if (forget_goal) {
    have_goal_ = false;
  }
  twist_stamped_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  twist_stamped_.header.stamp = get_node()->now();
  twist_stamped_.twist = geometry_msgs::msg::Twist();
  nav_state.set("cmd_vel", twist_stamped_);
  if (last_status_ == MPPIStatus::Following) {
    clear_markers();
  }
  last_status_ = MPPIStatus::NoPath;
}

pcl::PointCloud<pcl::PointXYZ>
MPPIController::obstacle_points(const NavState & nav_state) const
{
  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  // Downsampled first (indices only): fewer points to transform in the filter.
  return PointPerceptionsOpsView(perceptions)
         .downsample(0.1)
         .fuse(tf_info.robot_frame)
         .filter(
    {-obstacle_range_, -obstacle_range_, z_min_filter_},
    {obstacle_range_, obstacle_range_, robot_height_}, false)
         .fuse(tf_info.map_frame)
         .collapse({NAN, NAN, 0.1})
         .downsample(0.1)
         .as_points();
}

void
MPPIController::update_rt(NavState & nav_state)
{
  if (nav_state.has("navigation_state") &&
    nav_state.get_safe<GoalManager::State>("navigation_state") == GoalManager::State::IDLE)
  {
    stop(nav_state, true);
    return;
  }

  if (!nav_state.has("path") || !nav_state.has("robot_pose")) {
    return;
  }

  const auto & path = nav_state.get_safe<nav_msgs::msg::Path>("path");
  if (path.poses.empty()) {
    stop(nav_state, true);
    return;
  }

  // Limits may change at runtime (e.g. a safely limited speed).
  if (auto * provider = dynamic_cast<RobotLimitsProvider *>(get_node().get())) {
    const auto limits = provider->get_robot_limits();
    if (limits.max_linear_vel != limits_.max_linear_vel ||
      limits.min_linear_vel != limits_.min_linear_vel ||
      limits.max_lateral_vel != limits_.max_lateral_vel ||
      limits.max_angular_vel != limits_.max_angular_vel)
    {
      apply_limits(limits);
    }
  }

  const auto & pose = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  MPPIInput input;
  input.pose = {pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
  input.velocity = last_cmd_;
  input.path = &path;

  // A new goal: its "hold" heading is the robot's now.
  const auto & goal = path.poses.back().pose.position;
  if (!have_goal_ || goal.x != goal_x_ || goal.y != goal_y_) {
    have_goal_ = true;
    goal_x_ = goal.x;
    goal_y_ = goal.y;
    hold_yaw_ = input.pose.yaw;
  }

  input.heading_mode = heading_mode_;
  input.heading_yaw = heading_yaw_;
  if (heading_mode_ == HeadingMode::Hold) {
    input.heading_mode = HeadingMode::Fixed;
    input.heading_yaw = hold_yaw_;
  }
  if (params_.motion_model == MotionModel::Omni && nav_state.has(kHeadingReferenceKey)) {
    input.heading_mode = HeadingMode::Fixed;
    input.heading_yaw = nav_state.get_safe<double>(kHeadingReferenceKey);
  }

  // The strictest of the controller's and the system's tolerances (the system decides success).
  input.xy_goal_tolerance = xy_goal_tolerance_;
  input.yaw_goal_tolerance = yaw_goal_tolerance_;
  if (nav_state.has("goal_tolerance.position")) {
    input.xy_goal_tolerance =
      std::min(input.xy_goal_tolerance, nav_state.get_safe<double>("goal_tolerance.position"));
  }
  if (nav_state.has("goal_tolerance.yaw")) {
    input.yaw_goal_tolerance =
      std::min(input.yaw_goal_tolerance, nav_state.get_safe<double>("goal_tolerance.yaw"));
  }

  const auto now = get_node()->now();
  if (last_update_ && last_update_->get_clock_type() == now.get_clock_type()) {
    input.elapsed = std::clamp((now - *last_update_).seconds(), 0.0, 1.0);
  }
  last_update_ = now;

  const auto obstacles = obstacle_points(nav_state);
  input.obstacles = &obstacles;

  const auto result = optimizer_.compute(input);
  if (result.status == MPPIStatus::Blocked) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "[%s] every trajectory collides: stopping", get_plugin_name().c_str());
  } else if (result.status == MPPIStatus::NoPath) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "[%s] non-finite path or pose: stopping", get_plugin_name().c_str());
  }
  last_cmd_ = result.cmd;

  twist_stamped_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  twist_stamped_.header.stamp = now;
  twist_stamped_.twist = geometry_msgs::msg::Twist();
  twist_stamped_.twist.linear.x = result.cmd.vx;
  twist_stamped_.twist.linear.y = result.cmd.vy;
  twist_stamped_.twist.angular.z = result.cmd.wz;
  nav_state.set("cmd_vel", twist_stamped_);

  if (result.status == MPPIStatus::Following) {
    publish_markers();
  } else if (last_status_ == MPPIStatus::Following) {
    clear_markers();
  }
  last_status_ = result.status;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MPPIController, easynav::ControllerMethodBase)
