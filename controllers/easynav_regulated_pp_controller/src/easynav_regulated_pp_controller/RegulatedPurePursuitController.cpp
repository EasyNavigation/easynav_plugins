// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// and is a derivative work of nav2_regulated_pure_pursuit_controller,
// ported to the EasyNav plugin architecture.
//
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
/// \brief Implementation of the RegulatedPurePursuitController class.

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_regulated_pp_controller/RegulatedPurePursuitController.hpp"
#include "easynav_regulated_pp_controller/regulation_functions.hpp"
#include "easynav_regulated_pp_controller/dynamic_window_pure_pursuit_functions.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_system/GoalManager.hpp"

namespace easynav
{

namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}
}  // namespace

RegulatedPurePursuitController::RegulatedPurePursuitController() {}

RegulatedPurePursuitController::~RegulatedPurePursuitController() = default;

void
RegulatedPurePursuitController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  auto declare_and_get = [&node, &plugin_name](const std::string & name, auto & value) {
      node->declare_parameter(plugin_name + "." + name, value);
      node->get_parameter(plugin_name + "." + name, value);
    };

  declare_and_get("lookahead_dist", lookahead_dist_);
  declare_and_get("min_lookahead_dist", min_lookahead_dist_);
  declare_and_get("max_lookahead_dist", max_lookahead_dist_);
  declare_and_get("lookahead_time", lookahead_time_);
  declare_and_get("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);

  declare_and_get("max_linear_vel", max_linear_vel_);
  declare_and_get("min_linear_vel", min_linear_vel_);
  declare_and_get("max_angular_vel", max_angular_vel_);
  declare_and_get("min_angular_vel", min_angular_vel_);
  declare_and_get("max_linear_accel", max_linear_accel_);
  declare_and_get("max_linear_decel", max_linear_decel_);
  declare_and_get("max_angular_accel", max_angular_accel_);
  declare_and_get("max_angular_decel", max_angular_decel_);
  declare_and_get("use_dynamic_window", use_dynamic_window_);
  declare_and_get("allow_reversing", allow_reversing_);

  declare_and_get("use_rotate_to_heading", use_rotate_to_heading_);
  declare_and_get("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  declare_and_get("rotate_to_heading_min_angle", rotate_to_heading_min_angle_);

  declare_and_get("use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_);
  declare_and_get("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  declare_and_get("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_);
  declare_and_get("use_fixed_curvature_lookahead", use_fixed_curvature_lookahead_);
  declare_and_get("curvature_lookahead_dist", curvature_lookahead_dist_);
  declare_and_get("interpolate_curvature_after_goal", interpolate_curvature_after_goal_);

  declare_and_get(
    "use_obstacle_regulated_linear_velocity_scaling",
    use_obstacle_regulated_linear_velocity_scaling_);
  declare_and_get("obstacle_scaling_dist", obstacle_scaling_dist_);
  declare_and_get("obstacle_scaling_gain", obstacle_scaling_gain_);

  declare_and_get("min_approach_linear_velocity", min_approach_linear_velocity_);
  declare_and_get("approach_velocity_scaling_dist", approach_velocity_scaling_dist_);

  declare_and_get("xy_goal_tolerance", xy_goal_tolerance_);
  declare_and_get("yaw_goal_tolerance", yaw_goal_tolerance_);

  lookahead_point_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "lookahead_point", 10);
  curvature_lookahead_point_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "curvature_lookahead_point", 10);
  is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "is_rotating_to_heading", 10);

  last_update_ts_ = node->now();
}

void
RegulatedPurePursuitController::stop(NavState & nav_state, const std_msgs::msg::Header & header)
{
  cmd_vel_.header = header;
  cmd_vel_.twist.linear.x = 0.0;
  cmd_vel_.twist.angular.z = 0.0;
  last_linear_vel_ = 0.0;
  last_angular_vel_ = 0.0;
  is_rotating_to_heading_ = false;
  last_update_ts_ = get_node()->now();
  nav_state.set("cmd_vel", cmd_vel_);
}

double
RegulatedPurePursuitController::getLookAheadDistance(double linear_vel) const
{
  double dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    dist = std::fabs(linear_vel) * lookahead_time_;
    dist = std::clamp(dist, min_lookahead_dist_, max_lookahead_dist_);
  }
  return dist;
}

geometry_msgs::msg::Point
RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r,
  const geometry_msgs::msg::Point & center)
{
  const double x1 = p1.x - center.x;
  const double y1 = p1.y - center.y;
  const double x2 = p2.x - center.x;
  const double y2 = p2.y - center.y;

  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double dr2 = dx * dx + dy * dy;

  geometry_msgs::msg::Point result;

  if (dr2 < 1e-9) {
    result = p2;
    return result;
  }

  const double D = x1 * y2 - x2 * y1;
  const double discriminant = r * r * dr2 - D * D;

  if (discriminant < 0.0) {
    // No real intersection (can happen with degenerate/noisy paths): fall back to p2.
    result = p2;
    return result;
  }

  const double sqrt_term = std::sqrt(discriminant);
  const double sign_dy = (dy < 0.0) ? -1.0 : 1.0;

  const double sol1_x = (D * dy + sign_dy * dx * sqrt_term) / dr2;
  const double sol1_y = (-D * dx + std::fabs(dy) * sqrt_term) / dr2;
  const double sol2_x = (D * dy - sign_dy * dx * sqrt_term) / dr2;
  const double sol2_y = (-D * dx - std::fabs(dy) * sqrt_term) / dr2;

  // Choose the solution further along the segment direction (closer to p2).
  const double t1 = ((sol1_x - x1) * dx + (sol1_y - y1) * dy) / dr2;
  const double t2 = ((sol2_x - x1) * dx + (sol2_y - y1) * dy) / dr2;

  if (t1 >= t2) {
    result.x = sol1_x + center.x;
    result.y = sol1_y + center.y;
  } else {
    result.x = sol2_x + center.x;
    result.y = sol2_y + center.y;
  }
  result.z = 0.0;
  return result;
}

std::size_t
RegulatedPurePursuitController::findClosestPoseIndex(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & robot_position)
{
  std::size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double d = std::hypot(p.x - robot_position.x, p.y - robot_position.y);
    if (d < closest_dist) {
      closest_dist = d;
      closest_idx = i;
    }
  }
  return closest_idx;
}

geometry_msgs::msg::Point
RegulatedPurePursuitController::getLookAheadPoint(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & robot_position,
  double lookahead_dist,
  bool interpolate_after_end)
{
  auto dist_to_robot = [&robot_position](const geometry_msgs::msg::Point & p) {
      return std::hypot(p.x - robot_position.x, p.y - robot_position.y);
    };

  // Only look forward from the pose closest to the robot: the controller receives the full,
  // un-pruned global path, so searching from index 0 would (incorrectly) match the path's start
  // pose as soon as the robot has travelled more than a lookahead distance away from it, freezing
  // the carrot behind the robot for the rest of the path.
  const std::size_t start_idx = findClosestPoseIndex(path, robot_position);
  const auto begin_it = path.poses.begin() + static_cast<std::ptrdiff_t>(start_idx);

  auto it = std::find_if(
    begin_it, path.poses.end(),
    [&](const geometry_msgs::msg::PoseStamped & ps) {
      return dist_to_robot(ps.pose.position) >= lookahead_dist;
    });

  if (it == path.poses.end()) {
    const auto & goal = path.poses.back().pose.position;
    if (interpolate_after_end && path.poses.size() >= 2 && dist_to_robot(goal) < lookahead_dist) {
      const auto & prev = path.poses[path.poses.size() - 2].pose.position;
      const double seg_len = std::hypot(goal.x - prev.x, goal.y - prev.y);
      if (seg_len > 1e-6) {
        geometry_msgs::msg::Point extended;
        extended.x = goal.x + (goal.x - prev.x) / seg_len * lookahead_dist;
        extended.y = goal.y + (goal.y - prev.y) / seg_len * lookahead_dist;
        return circleSegmentIntersection(goal, extended, lookahead_dist, robot_position);
      }
    }
    return goal;
  }

  if (it == begin_it) {
    return it->pose.position;
  }

  const auto prev_it = std::prev(it);
  return circleSegmentIntersection(
    prev_it->pose.position, it->pose.position, lookahead_dist, robot_position);
}

geometry_msgs::msg::Point
RegulatedPurePursuitController::toRobotFrame(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Pose & robot_pose,
  double robot_yaw)
{
  const double dx = point.x - robot_pose.position.x;
  const double dy = point.y - robot_pose.position.y;
  const double c = std::cos(-robot_yaw);
  const double s = std::sin(-robot_yaw);

  geometry_msgs::msg::Point local;
  local.x = c * dx - s * dy;
  local.y = s * dx + c * dy;
  local.z = 0.0;
  return local;
}

bool
RegulatedPurePursuitController::shouldRotateToPath(double angle_to_path) const
{
  return use_rotate_to_heading_ && std::fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

void
RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel, double angle_to_target, double dt) const
{
  linear_vel = 0.0;
  const double sign = angle_to_target > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  // Braking (magnitude decreasing) is bounded by max_angular_decel_, speeding up
  // (magnitude increasing) by max_angular_accel_; which bound applies to which side
  // of the window depends on the current direction of rotation.
  const double decel_bound = last_angular_vel_ >= 0.0 ? max_angular_decel_ : max_angular_accel_;
  const double accel_bound = last_angular_vel_ >= 0.0 ? max_angular_accel_ : max_angular_decel_;
  const double min_feasible = last_angular_vel_ - decel_bound * dt;
  const double max_feasible = last_angular_vel_ + accel_bound * dt;
  angular_vel = std::clamp(angular_vel, min_feasible, max_feasible);

  // Slow down to avoid overshooting the target angle.
  const double decel = std::max(max_angular_decel_, 1e-9);
  const double max_vel_to_stop = std::sqrt(2.0 * decel * std::fabs(angle_to_target));
  if (std::fabs(angular_vel) > max_vel_to_stop) {
    angular_vel = sign * max_vel_to_stop;
  }
}

void
RegulatedPurePursuitController::applyConstraints(
  double curvature,
  double min_obstacle_distance,
  double remaining_path_distance,
  double euclidean_dist_to_goal,
  double & linear_vel) const
{
  double curvature_vel = linear_vel;
  double obstacle_vel = linear_vel;

  if (use_regulated_linear_velocity_scaling_) {
    curvature_vel = heuristics::curvatureConstraint(
      linear_vel, curvature, regulated_linear_scaling_min_radius_);
  }

  if (use_obstacle_regulated_linear_velocity_scaling_) {
    obstacle_vel = heuristics::obstacleConstraint(
      linear_vel, min_obstacle_distance, obstacle_scaling_dist_, obstacle_scaling_gain_);
  }

  linear_vel = std::min(curvature_vel, obstacle_vel);
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  linear_vel = heuristics::approachVelocityConstraint(
    linear_vel, remaining_path_distance, euclidean_dist_to_goal,
    min_approach_linear_velocity_, approach_velocity_scaling_dist_);
}

double
RegulatedPurePursuitController::computeMinObstacleDistance(
  NavState & nav_state, double max_range) const
{
  const auto & perceptions = nav_state.get_by_type<PointPerception>();
  if (perceptions.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const auto & robot_frame = tf_info.robot_frame;

  auto view = PointPerceptionsOpsView(perceptions);
  view.fuse(robot_frame)
  .filter(
    {0.0, -robot_radius_ - safety_margin_, z_min_filter_},
    {std::max(max_range, 0.0), robot_radius_ + safety_margin_, robot_height_});

  const auto & cloud = view.as_points();

  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {continue;}
    const double d = std::hypot(p.x, p.y) - robot_radius_;
    if (d < min_dist) {min_dist = d;}
  }

  return min_dist;
}

double
RegulatedPurePursuitController::remainingPathDistance(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & robot_position)
{
  if (path.poses.empty()) {return 0.0;}

  const std::size_t closest_idx = findClosestPoseIndex(path, robot_position);
  const auto & closest_p = path.poses[closest_idx].pose.position;
  double distance = std::hypot(closest_p.x - robot_position.x, closest_p.y - robot_position.y);
  for (std::size_t i = closest_idx + 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & cur = path.poses[i].pose.position;
    distance += std::hypot(cur.x - prev.x, cur.y - prev.y);
  }

  return distance;
}

void
RegulatedPurePursuitController::publishCarrot(
  const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & pub,
  const geometry_msgs::msg::Point & carrot,
  const std_msgs::msg::Header & header) const
{
  if (!pub || pub->get_subscription_count() == 0) {return;}
  geometry_msgs::msg::PointStamped msg;
  msg.header = header;
  msg.point = carrot;
  msg.point.z = 0.05;
  pub->publish(msg);
}

void
RegulatedPurePursuitController::update_rt(NavState & nav_state)
{
  if (nav_state.has("navigation_state")) {
    const auto goal_state = nav_state.get<easynav::GoalManager::State>("navigation_state");
    if (goal_state == easynav::GoalManager::State::IDLE) {
      std_msgs::msg::Header header;
      header.stamp = get_node()->now();
      stop(nav_state, header);
      return;
    }
  }

  if (!nav_state.has("path") || !nav_state.has("robot_pose")) {return;}

  const auto & path = nav_state.get<nav_msgs::msg::Path>("path");

  std_msgs::msg::Header header;
  header.frame_id = path.header.frame_id;
  header.stamp = get_node()->now();

  if (path.poses.empty()) {
    stop(nav_state, header);
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  const double robot_yaw = tf2::getYaw(robot_pose.orientation);

  const auto & goal_pose = path.poses.back().pose;

  double xy_tol = xy_goal_tolerance_;
  double yaw_tol = yaw_goal_tolerance_;
  if (nav_state.has("goal_tolerance.position")) {
    xy_tol = nav_state.get<double>("goal_tolerance.position");
  }
  if (nav_state.has("goal_tolerance.yaw")) {
    yaw_tol = nav_state.get<double>("goal_tolerance.yaw");
  }

  const double dist_to_goal = std::hypot(
    goal_pose.position.x - robot_pose.position.x,
    goal_pose.position.y - robot_pose.position.y);
  const double yaw_goal = tf2::getYaw(goal_pose.orientation);
  const double e_theta_goal = normalizeAngle(yaw_goal - robot_yaw);

  const bool at_goal_xy = dist_to_goal <= xy_tol;

  if (at_goal_xy && std::fabs(e_theta_goal) <= yaw_tol) {
    stop(nav_state, header);
    return;
  }

  double dt = (get_node()->now() - last_update_ts_).seconds();
  if (dt <= 0.0 || dt > 1.0) {dt = 0.05;}
  last_update_ts_ = get_node()->now();

  double linear_vel = max_linear_vel_;
  double angular_vel = 0.0;

  if (use_rotate_to_heading_ && at_goal_xy) {
    is_rotating_to_heading_ = true;
    rotateToHeading(linear_vel, angular_vel, e_theta_goal, dt);
  } else {
    const double lookahead_dist = getLookAheadDistance(last_linear_vel_);
    const auto carrot_global = getLookAheadPoint(path, robot_pose.position, lookahead_dist);
    const auto carrot_local = toRobotFrame(carrot_global, robot_pose, robot_yaw);
    publishCarrot(lookahead_point_pub_, carrot_global, header);

    auto curvature_local = carrot_local;
    if (use_fixed_curvature_lookahead_) {
      const auto curvature_carrot_global = getLookAheadPoint(
        path, robot_pose.position, curvature_lookahead_dist_, interpolate_curvature_after_goal_);
      curvature_local = toRobotFrame(curvature_carrot_global, robot_pose, robot_yaw);
      publishCarrot(curvature_lookahead_point_pub_, curvature_carrot_global, header);
    }

    double x_vel_sign = 1.0;
    if (allow_reversing_) {
      x_vel_sign = carrot_local.x >= 0.0 ? 1.0 : -1.0;
    }

    double angle_to_path = std::atan2(curvature_local.y, curvature_local.x);
    if (x_vel_sign < 0.0) {
      angle_to_path = normalizeAngle(angle_to_path + M_PI);
    }

    const double regulation_curvature = heuristics::calculateCurvature(
      curvature_local.x, curvature_local.y);

    if (shouldRotateToPath(angle_to_path)) {
      is_rotating_to_heading_ = true;
      rotateToHeading(linear_vel, angular_vel, angle_to_path, dt);
    } else {
      is_rotating_to_heading_ = false;

      const double min_obstacle_distance = use_obstacle_regulated_linear_velocity_scaling_ ?
        computeMinObstacleDistance(nav_state, lookahead_dist) :
        std::numeric_limits<double>::infinity();
      const double remaining_path_distance = remainingPathDistance(path, robot_pose.position);

      applyConstraints(
        regulation_curvature, min_obstacle_distance, remaining_path_distance, dist_to_goal,
        linear_vel);
      linear_vel = x_vel_sign * std::clamp(std::fabs(linear_vel), 0.0, max_linear_vel_);

      if (!use_dynamic_window_) {
        angular_vel = linear_vel * regulation_curvature;
      } else {
        geometry_msgs::msg::Twist current_speed;
        current_speed.linear.x = last_linear_vel_;
        current_speed.angular.z = last_angular_vel_;
        std::tie(linear_vel, angular_vel) =
          dynamic_window_pure_pursuit::computeDynamicWindowVelocities(
          current_speed, max_linear_vel_, min_linear_vel_, max_angular_vel_, min_angular_vel_,
          max_linear_accel_, max_linear_decel_, max_angular_accel_, max_angular_decel_,
          linear_vel, regulation_curvature, x_vel_sign, dt);
      }
    }
  }

  last_linear_vel_ = linear_vel;
  last_angular_vel_ = angular_vel;

  if (is_rotating_to_heading_pub_ && is_rotating_to_heading_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Bool msg;
    msg.data = is_rotating_to_heading_;
    is_rotating_to_heading_pub_->publish(msg);
  }

  cmd_vel_.header = header;
  cmd_vel_.twist.linear.x = linear_vel;
  cmd_vel_.twist.angular.z = angular_vel;
  nav_state.set("cmd_vel", cmd_vel_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  easynav::RegulatedPurePursuitController, easynav::ControllerMethodBase)
