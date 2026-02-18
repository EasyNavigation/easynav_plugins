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
/// \brief Implementation of the MPCController class.

#include "easynav_mpc_controller/MPCController.hpp"
#include "easynav_system/GoalManager.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

MPCController::MPCController() {}

MPCController::~MPCController() = default;

void
MPCController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->declare_parameter<double>(plugin_name + ".dt", dt_);
  node->declare_parameter<double>(plugin_name + ".safety_radius", safety_radius_);
  node->declare_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->declare_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);
  node->declare_parameter<bool>(plugin_name + ".verbose", verbose_);

  node->declare_parameter<double>(plugin_name + ".fallback_goal_pos_tol", fallback_goal_pos_tol_);
  node->declare_parameter<double>(plugin_name + ".fallback_goal_yaw_tol", fallback_goal_yaw_tol_);

  node->get_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->get_parameter<double>(plugin_name + ".dt", dt_);
  node->get_parameter<double>(plugin_name + ".safety_radius", safety_radius_);
  node->get_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->get_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);
  node->get_parameter<bool>(plugin_name + ".verbose", verbose_);

  node->get_parameter<double>(plugin_name + ".fallback_goal_pos_tol", fallback_goal_pos_tol_);
  node->get_parameter<double>(plugin_name + ".fallback_goal_yaw_tol", fallback_goal_yaw_tol_);

  optimizer_ = std::make_unique<MPCOptimizer>();

  mpc_path_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("/mpc/path", 10);

  detection_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("/mpc/detection", 10);
}

void
MPCController::publish_mpc_path(
  void *data, const std::vector<double> & best_vel,
  nav_msgs::msg::Path path)
{
  MPCParameters *params = reinterpret_cast<MPCParameters *>(data);
  nav_msgs::msg::Path mpc_path_;

  if (best_vel.size() > 0) {
    mpc_path_.header.stamp = get_node()->now();
    mpc_path_.header.frame_id = path.header.frame_id;
    for (size_t i = 0; i + 1 < best_vel.size(); i += 2) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      double v = best_vel[i];
      double w = best_vel[i + 1];
      pose_stamped.header.frame_id = path.header.frame_id;
      pose_stamped.header.stamp = path.header.stamp;
      auto state = optimizer_->kinematic_model(params->x0, params->theta0, v, w, dt_);
      pose_stamped.pose.position.x = state[0];
      pose_stamped.pose.position.y = state[1];
      mpc_path_.poses.push_back(pose_stamped);
    }
    mpc_path_pub_->publish(mpc_path_);

  }
}

void
MPCController::collision_checker(void *data, std::vector<double> & u)
{
  MPCParameters *params = reinterpret_cast<MPCParameters *>(data);
  double x_m = 0.0, y_m = 0.0, dist = 0.0, angle = 0.0;
  size_t real_points = 0;
  for (const auto & point : params->points) {
    if(!std::isnan(point.x) || !std::isnan(point.y)) {
      x_m += (point.x - params->x0[0]);
      y_m += (point.y - params->x0[1]);
      real_points++;
    }
  }
  x_m /= real_points;
  y_m /= real_points;
  dist = std::hypot(x_m, y_m);
  angle = std::atan2(y_m, x_m) - params->theta0[2];
  if(real_points != 0 && dist < safety_radius_) {
    std::cerr << "Detection at: " << dist << " Theta: " << angle << std::endl;
    u[0] -= dist / real_points * dt_;
    u[1] -= angle / real_points * dt_;
  }
}

void
MPCController::update_rt(NavState & nav_state)
{
  // If navigation is IDLE, force zero velocity
  if (nav_state.has("navigation_state")) {
    const auto nav_state_val = nav_state.get<easynav::GoalManager::State>("navigation_state");
    if (nav_state_val == easynav::GoalManager::State::IDLE) {
      cmd_vel_.header.stamp = get_node()->now();
      cmd_vel_.twist.linear.x = 0.0;
      cmd_vel_.twist.angular.z = 0.0;
      nav_state.set("cmd_vel", cmd_vel_);
      return;
    }
  }

  if (!nav_state.has("path") || !nav_state.has("robot_pose") || !nav_state.has("points")) {
    if(verbose_) {
      std::cout << "No Path, No Points or No Robot Pose" << std::endl;
    }
    return;
  }

  nav_msgs::msg::Path path = nav_state.get<nav_msgs::msg::Path>("path");
  if (path.poses.empty()) {
    // If the path is empty, stop the robot
    cmd_vel_.header.frame_id = path.header.frame_id;
    cmd_vel_.header.stamp = get_node()->now();
    cmd_vel_.twist.linear.x = 0.0;
    cmd_vel_.twist.angular.z = 0.0;
    nav_state.set("cmd_vel", cmd_vel_);
    return;
  }

  // Build a local path that:
  // 1) keeps only the segment that brings the robot closer to the goal, and
  // 2) prepends a short straight segment from the robot pose to that segment.
  const auto & robot_pose_msg = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & robot_p = robot_pose_msg.pose.pose.position;

  // Goal is the last point of the planner path
  const auto & goal_p = path.poses.back().pose.position;

  nav_msgs::msg::Path local_path;
  local_path.header = path.header;
  local_path.poses.clear();

  // 1) Find the first index that actually brings us closer to the goal than our current pose
  const double d_robot_goal = std::hypot(goal_p.x - robot_p.x, goal_p.y - robot_p.y);
  std::size_t start_idx = 0;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pi = path.poses[i].pose.position;
    const double d_pi_goal = std::hypot(goal_p.x - pi.x, goal_p.y - pi.y);
    if (d_pi_goal <= d_robot_goal) {
      start_idx = i;
      break;
    }
  }

  // 2) Prepend a point at the robot position to ensure continuity from the current pose
  geometry_msgs::msg::PoseStamped robot_ps;
  robot_ps.header = path.header;
  robot_ps.pose.position = robot_p;
  robot_ps.pose.orientation = path.poses[start_idx].pose.orientation;
  local_path.poses.push_back(robot_ps);

  // 3) Copy the remaining points from start_idx to the goal
  for (std::size_t i = start_idx; i < path.poses.size(); ++i) {
    local_path.poses.push_back(path.poses[i]);
  }

  // Use the local path from now on
  path = local_path;

  int num_elements = path.poses.size();
  size_t local_horizon;
  if (num_elements > horizon_steps_) {
    local_horizon = horizon_steps_;
  } else {
    local_horizon = num_elements - 1;
  }
  const auto & last_pose = path.poses[local_horizon].pose.position;

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  const auto & filtered = PointPerceptionsOpsView(perceptions)
    .filter({-2.0, -0.35, -1.0}, {0.0, 0.35, 1.0})
    .fuse(tf_info.map_frame)
    .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .collapse({NAN, NAN, 0.1})
    .downsample(0.1)
    .as_points();

  sensor_msgs::msg::PointCloud2 cloud_out;
  pcl::toROSMsg(filtered, cloud_out);
  cloud_out.header.frame_id = path.header.frame_id;
  cloud_out.header.stamp = get_node()->now();
  detection_pub_->publish(cloud_out);

  const auto pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  double roll_, pitch_, yaw_;
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // MPC Code
  double minf;
  std::vector<double> u(2 * horizon_steps_, 0.0);

  auto params = MPCParameters(
    Eigen::Vector2d(static_cast<double>(last_pose.x), static_cast<double>(last_pose.y)),
    {pose.position.x, pose.position.y, pose.position.z},
    {roll_, pitch_, yaw_},
    filtered,
    static_cast<int>(horizon_steps_),
    dt_);

  NLoptCallbackData cbdata{optimizer_.get(), &params};

  nlopt::opt opt(nlopt::LN_COBYLA, static_cast<int>(u.size()));
  opt.set_min_objective(easynav::MPCOptimizer::nlopt_cost_callback, &cbdata);

  std::vector<double> lb(2 * horizon_steps_);
  std::vector<double> ub(2 * horizon_steps_);
  for (int k = 0; k < horizon_steps_ ; k++) {
    lb[2 * k] = -max_lin_vel_;
    lb[2 * k + 1] = -max_ang_vel_;
    ub[2 * k] = max_lin_vel_;
    ub[2 * k + 1] = max_ang_vel_;
  }
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_xtol_rel(1e-3);
  opt.set_ftol_rel(1e-3);
  // opt.set_maxeval(1000);

  try {
    nlopt::result result = opt.optimize(u, minf);
    if(verbose_) {
      if (result > 0) {
        std::cerr << "Optimization Successful " << std::endl;
        std::cout << "Result: " << result << std::endl;
      } else {
        std::cerr << "Optimization Unsuccessful " << std::endl;
      }
    }

  } catch (std::exception & e) {
    std::cerr << "Optimization Error: " << e.what() << std::endl;
  }

  collision_checker(&params, u);

  // Final alignment phase with hysteresis on distance:
  // - Enter when dist_to_goal <= 0.5 * pos_tol
  // - Stay in this phase (even if dist grows slightly) until dist_to_goal > pos_tol
  {
    const auto & goal_pose = path.poses.back().pose;

    double pos_tol = fallback_goal_pos_tol_;
    double yaw_tol = fallback_goal_yaw_tol_;

    if (nav_state.has("goal_tolerance.position")) {
      pos_tol = nav_state.get<double>("goal_tolerance.position");
    }
    if (nav_state.has("goal_tolerance.yaw")) {
      yaw_tol = nav_state.get<double>("goal_tolerance.yaw");
    }

    const double dx_g = goal_pose.position.x - pose.position.x;
    const double dy_g = goal_pose.position.y - pose.position.y;
    const double dist_to_goal = std::hypot(dx_g, dy_g);

    const double yaw_goal = std::atan2(
      2.0 * (goal_pose.orientation.w * goal_pose.orientation.z +
      goal_pose.orientation.x * goal_pose.orientation.y),
      1.0 - 2.0 * (goal_pose.orientation.y * goal_pose.orientation.y +
      goal_pose.orientation.z * goal_pose.orientation.z));
    double e_theta_goal = std::atan2(std::sin(yaw_ - yaw_goal), std::cos(yaw_ - yaw_goal));

    const double enter_dist = 0.5 * pos_tol;

    // Hysteresis based on distance to goal: start aligning when we are
    // well inside the goal radius (enter_dist) and keep aligning until
    // we move clearly outside (dist_to_goal > pos_tol).
    const bool inside_hysteresis_band = (dist_to_goal <= pos_tol);
    const bool should_enter_alignment = (dist_to_goal <= enter_dist);

    if ((inside_hysteresis_band && std::fabs(e_theta_goal) > yaw_tol) ||
      should_enter_alignment)
    {
      // Stay in place and rotate towards the goal orientation using a simple P controller
      const double k_align = 1.0;
      double vlin = 0.0;
      double vrot = -k_align * e_theta_goal;

      vrot = std::clamp(vrot, -max_ang_vel_, max_ang_vel_);

      cmd_vel_.header.frame_id = path.header.frame_id;
      cmd_vel_.header.stamp = get_node()->now();
      cmd_vel_.twist.linear.x = vlin;
      cmd_vel_.twist.angular.z = vrot;

      nav_state.set("cmd_vel", cmd_vel_);
      publish_mpc_path(&params, u, path);
      return;
    }
  }

  // Publish the computed velocity command
  cmd_vel_.header.frame_id = path.header.frame_id;
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.twist.linear.x = u[0];
  cmd_vel_.twist.angular.z = u[1];

  nav_state.set("cmd_vel", cmd_vel_);

  // Publish the path
  publish_mpc_path(&params, u, path);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MPCController, easynav::ControllerMethodBase)
