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

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_system/GoalManager.hpp"

namespace easynav
{

namespace
{

const char * status_name(mpc::QPStatus status)
{
  switch (status) {
    case mpc::QPStatus::Solved: return "solved";
    case mpc::QPStatus::Inaccurate: return "inaccurate";
    case mpc::QPStatus::Infeasible: return "infeasible";
    default: return "failed";
  }
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

void
MPCController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  auto param = [&](const std::string & name, auto & value) {
      using T = std::decay_t<decltype(value)>;
      easynav::declare_parameter_if_absent<T>(*node, plugin_name + "." + name, value);
      node->get_parameter<T>(plugin_name + "." + name, value);
    };
  auto given = [&](const std::string & name) {
      return overrides.count(plugin_name + "." + name) > 0;
    };

  std::string motion_model = "diff";
  param("motion_model", motion_model);
  if (!mpc::parse_motion_model(motion_model, model_)) {
    throw std::runtime_error(
            "[" + plugin_name + "] motion_model '" + motion_model +
            "' is not diff, omni, ackermann or aerial");
  }
  layout_ = mpc::ModelLayout(model_);

  mpc::MPCParams p;
  p.model = model_;
  param("horizon_steps", p.horizon_steps);
  param("dt", p.dt);
  param("time_constant", p.time_constant);
  param("weight_xy", p.weight_xy);
  param("weight_z", p.weight_z);
  param("weight_yaw", p.weight_yaw);
  param("terminal_factor", p.terminal_factor);
  param("near_weight", p.near_weight);
  param("weight_linear_vel", p.weight_linear_vel);
  param("weight_angular_vel", p.weight_angular_vel);
  param("weight_linear_rate", p.weight_linear_rate);
  param("weight_angular_rate", p.weight_angular_rate);
  param("obstacle_range", p.obstacle_range);
  param("obstacles_per_step", p.obstacles_per_step);
  param("safety_margin", p.safety_margin);
  param("max_iterations", p.qp.max_iterations);
  param("tolerance", p.qp.eps_abs);
  p.qp.eps_rel = p.qp.eps_abs;
  param("latency", latency_);
  param("rotate_to_heading_angle", rotate_to_heading_angle_);
  param("goal_align_gain", goal_align_gain_);
  param("min_height", min_height_);
  param("use_odometry_velocity", use_odometry_velocity_);
  param("max_lateral_acc", speed_limits_.max_lateral_acc);
  param("approach_speed", speed_limits_.approach_speed);
  double curvature_window = 1.0;
  param("curvature_window", curvature_window);
  double path_smoothing = -1.0;  // auto: from the turning radius
  param("path_smoothing", path_smoothing);

  // Deprecated goal tolerances: still read if the new ones are not given
  if (given("fallback_goal_pos_tol") && !given("xy_goal_tolerance")) {
    param("fallback_goal_pos_tol", xy_goal_tolerance_);
  }
  if (given("fallback_goal_yaw_tol") && !given("yaw_goal_tolerance")) {
    param("fallback_goal_yaw_tol", yaw_goal_tolerance_);
  }
  param("xy_goal_tolerance", xy_goal_tolerance_);
  param("yaw_goal_tolerance", yaw_goal_tolerance_);
  for (const char * old : {"fallback_goal_pos_tol", "fallback_goal_yaw_tol"}) {
    if (given(old)) {
      RCLCPP_WARN(
        node->get_logger(), "[%s] '%s' is deprecated: use %s", plugin_name.c_str(), old,
        std::string(old).find("pos") != std::string::npos ? "xy_goal_tolerance" :
        "yaw_goal_tolerance");
    }
  }
  for (const char * removed : {"safety_radius", "use_collision_checker", "verbose"}) {
    if (given(removed)) {
      RCLCPP_WARN(
        node->get_logger(), "[%s] '%s' has no effect any more: remove it (obstacles are "
        "constraints, with robot_geometry and safety_margin)", plugin_name.c_str(), removed);
    }
  }

  if (p.horizon_steps < 1 || p.horizon_steps > 500 || !(p.dt > 0.0) || p.time_constant < 0.0 ||
    p.obstacles_per_step < 0 || p.qp.max_iterations < 1 || !(p.qp.eps_abs > 0.0) ||
    !(p.near_weight > 0.0) || p.near_weight > 1.0 ||
    latency_ < 0.0 || !(curvature_window > 0.0) || !(speed_limits_.max_lateral_acc > 0.0) ||
    speed_limits_.approach_speed < 0.0)
  {
    throw std::runtime_error(
            "[" + plugin_name + "] invalid parameters: horizon_steps 1-500, dt > 0, "
            "time_constant >= 0, max_iterations >= 1, tolerance > 0, latency >= 0, "
            "near_weight in (0, 1], "
            "curvature_window > 0, max_lateral_acc > 0, approach_speed >= 0");
  }

  const auto geometry = get_robot_geometry();
  shape_.radius = geometry.radius;
  shape_.footprint.clear();
  for (const auto & v : geometry.footprint) {
    shape_.footprint.emplace_back(v.x, v.y);
  }
  robot_height_ = geometry.height;

  optimizer_.configure(p);
  reference_ = mpc::Reference(model_);
  reference_.curvature_window = curvature_window;

  // Velocity and acceleration limits: the robot's (controller_node "robot_limits.*").
  apply_limits(
    get_robot_limits({"max_linear_velocity", "", "max_angular_velocity", "", "", "", ""}));
  reference_.path_smoothing = path_smoothing >= 0.0 ? path_smoothing :
    (layout_.holonomic() ? 0.0 : 1.45 * limits_.min_turning_radius);
  if (model_ == mpc::MotionModel::Ackermann && limits_.min_turning_radius <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] motion_model 'ackermann' without "
      "robot_limits.min_turning_radius: nothing keeps the curvature within the steering",
      plugin_name.c_str());
  }
  if (model_ == mpc::MotionModel::Aerial && limits_.max_vertical_vel <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] motion_model 'aerial' with robot_limits.max_vertical_vel 0: "
      "it will not change height", plugin_name.c_str());
  }

  velocity_ = Eigen::VectorXd::Zero(layout_.nv);
  last_cmd_ = Eigen::VectorXd::Zero(layout_.nv);
  last_update_.reset();
  aligning_ = rotating_ = false;

  const std::string prefix =
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name;
  predicted_pub_ = node->create_publisher<nav_msgs::msg::Path>(prefix + "/predicted", 10);
  reference_pub_ = node->create_publisher<nav_msgs::msg::Path>(prefix + "/reference", 10);
  obstacles_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    prefix + "/obstacles", 10);
}

void
MPCController::apply_limits(const RobotLimits & limits)
{
  limits_ = limits;
  std::vector<mpc::AxisLimits> axes(layout_.nv);
  axes[0] = {limits.min_linear_vel, limits.max_linear_vel, limits.max_linear_acc,
    limits.max_linear_decel};
  if (layout_.vy >= 0) {
    axes[layout_.vy] = {-limits.max_lateral_vel, limits.max_lateral_vel, limits.max_lateral_acc,
      limits.max_lateral_decel};
  }
  if (layout_.vz >= 0) {
    axes[layout_.vz] = {-limits.max_vertical_vel, limits.max_vertical_vel,
      limits.max_vertical_acc, limits.max_vertical_decel};
  }
  axes[layout_.wz] = {-limits.max_angular_vel, limits.max_angular_vel, limits.max_angular_acc,
    limits.max_angular_decel};
  optimizer_.set_axes(axes, layout_.holonomic() ? 0.0 : limits.min_turning_radius);

  speed_limits_.max_vel = limits.max_linear_vel;
  speed_limits_.max_vertical_vel = limits.max_vertical_vel;
  speed_limits_.acc = limits.max_linear_acc;
  speed_limits_.decel = limits.max_linear_decel;
  speed_limits_.max_angular_vel = limits.max_angular_vel;
}

std::vector<Eigen::Vector3d>
MPCController::obstacle_points(const NavState & nav_state, const Eigen::VectorXd & pose)
{
  std::vector<Eigen::Vector3d> out;
  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  if (perceptions.empty()) {
    return out;
  }
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const auto & params = optimizer_.params();
  // As far as the reference looks: the horizon at full speed and the stop after it
  const double vmax = speed_limits_.max_vel;
  const double braking = vmax * vmax / (2.0 * std::max(speed_limits_.decel, 1e-3));
  const double range = std::max(
    params.obstacle_range, vmax * params.horizon_steps * params.dt + braking + shape_.radius);
  // Ground robots: what is between the floor and their top; aerial ones: everything around
  const double z_lo = layout_.aerial() ? -range : min_height_;
  const double z_hi = layout_.aerial() ? range : robot_height_;
  const auto & points = PointPerceptionsOpsView(perceptions)
    .downsample(0.1)
    .fuse(tf_info.robot_frame)
    .filter({-range, -range, z_lo}, {range, range, z_hi}, false)
    .fuse(tf_info.map_frame)
    .downsample(0.2)
    .as_points();
  constexpr double kSelfMargin = 0.1;  // sensor noise around the robot's own body (m)
  const Eigen::Vector3d position(pose(0), pose(1), layout_.aerial() ? pose(2) : 0.0);
  const double c = std::cos(pose(layout_.yaw)), s = std::sin(pose(layout_.yaw));
  pcl::PointCloud<pcl::PointXYZ> kept;
  out.reserve(points.size());
  for (const auto & pt : points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    const Eigen::Vector3d d = Eigen::Vector3d(pt.x, pt.y, pt.z) - position;
    const bool own_body = layout_.aerial() ?
      d.norm() <= shape_.radius + kSelfMargin :
      shape_.contains(Eigen::Vector2d(c * d.x() + s * d.y(), -s * d.x() + c * d.y()), kSelfMargin);
    if (own_body) {
      continue;
    }
    out.emplace_back(pt.x, pt.y, pt.z);
    kept.push_back(pt);
  }
  if (obstacles_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 cloud;
    if (!kept.empty()) {
      pcl::toROSMsg(kept, cloud);
    } else {
      cloud.height = 1;  // toROSMsg() indexes the data of an empty cloud
    }
    cloud.header.frame_id = tf_info.map_frame;
    cloud.header.stamp = get_node()->now();
    obstacles_pub_->publish(cloud);
  }
  return out;
}

void
MPCController::publish_command(NavState & nav_state, const Eigen::VectorXd & v)
{
  cmd_vel_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.twist = geometry_msgs::msg::Twist();
  cmd_vel_.twist.linear.x = v(0);
  if (layout_.vy >= 0) {cmd_vel_.twist.linear.y = v(layout_.vy);}
  if (layout_.vz >= 0) {cmd_vel_.twist.linear.z = v(layout_.vz);}
  cmd_vel_.twist.angular.z = v(layout_.wz);
  nav_state.set("cmd_vel", cmd_vel_);
  last_cmd_ = v;
}

void
MPCController::stop(NavState & nav_state)
{
  optimizer_.reset();
  aligning_ = rotating_ = false;
  publish_command(nav_state, Eigen::VectorXd::Zero(layout_.nv));
}

Eigen::VectorXd
MPCController::braking(double dt) const
{
  const auto & axes = optimizer_.params().axes;
  Eigen::VectorXd v = velocity_;
  for (int j = 0; j < layout_.nv; ++j) {
    const double step = axes[j].decel * dt;
    v(j) = v(j) > 0.0 ? std::max(0.0, v(j) - step) : std::min(0.0, v(j) + step);
  }
  return v;
}

void
MPCController::publish_paths(
  const std::vector<Eigen::VectorXd> & predicted, const std::vector<Eigen::VectorXd> & reference,
  const std::string & frame)
{
  auto to_path = [&](const std::vector<Eigen::VectorXd> & poses) {
      nav_msgs::msg::Path path;
      path.header.frame_id = frame;
      path.header.stamp = get_node()->now();
      for (const auto & p : poses) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p(0);
        ps.pose.position.y = p(1);
        if (layout_.aerial()) {ps.pose.position.z = p(2);}
        ps.pose.orientation.z = std::sin(0.5 * p(layout_.yaw));
        ps.pose.orientation.w = std::cos(0.5 * p(layout_.yaw));
        path.poses.push_back(ps);
      }
      return path;
    };
  if (predicted_pub_->get_subscription_count() > 0) {
    predicted_pub_->publish(to_path(predicted));
  }
  if (reference_pub_->get_subscription_count() > 0) {
    reference_pub_->publish(to_path(reference));
  }
}

void
MPCController::update_rt(NavState & nav_state)
{
  const auto now = get_node()->now();
  const auto & params = optimizer_.params();
  double elapsed = params.dt;
  if (last_update_ && last_update_->get_clock_type() == now.get_clock_type()) {
    elapsed = std::clamp((now - *last_update_).seconds(), 0.0, 0.5);
  }
  last_update_ = now;

  if (nav_state.has("navigation_state") &&
    nav_state.get_safe<GoalManager::State>("navigation_state") == GoalManager::State::IDLE)
  {
    stop(nav_state);
    return;
  }
  if (!nav_state.has("path") || !nav_state.has("robot_pose")) {
    return;
  }
  const auto & path = nav_state.get_safe<nav_msgs::msg::Path>("path");
  if (path.poses.empty()) {
    reference_.reset();
    stop(nav_state);
    return;
  }

  // Limits may change at runtime (e.g. a safely limited speed).
  if (auto * provider = dynamic_cast<RobotLimitsProvider *>(get_node().get())) {
    const auto limits = provider->get_robot_limits();
    if (limits.max_linear_vel != limits_.max_linear_vel ||
      limits.min_linear_vel != limits_.min_linear_vel ||
      limits.max_lateral_vel != limits_.max_lateral_vel ||
      limits.max_vertical_vel != limits_.max_vertical_vel ||
      limits.max_angular_vel != limits_.max_angular_vel)
    {
      apply_limits(limits);
    }
  }

  // The robot's state: pose, and velocity (the commands, lagging as the model says, or the
  // odometry's)
  const auto & odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
  const auto & pp = odom.pose.pose;
  Eigen::VectorXd pose(layout_.np);
  pose(0) = pp.position.x;
  pose(1) = pp.position.y;
  if (layout_.aerial()) {pose(2) = pp.position.z;}
  pose(layout_.yaw) = yaw_of(pp.orientation);
  if (!pose.allFinite()) {
    stop(nav_state);
    return;
  }
  if (use_odometry_velocity_) {
    const auto & t = odom.twist.twist;
    velocity_(0) = t.linear.x;
    if (layout_.vy >= 0) {velocity_(layout_.vy) = t.linear.y;}
    if (layout_.vz >= 0) {velocity_(layout_.vz) = t.linear.z;}
    velocity_(layout_.wz) = t.angular.z;
  } else {
    const double a = params.time_constant > 0.0 ? std::exp(-elapsed / params.time_constant) : 0.0;
    velocity_ = a * velocity_ + (1.0 - a) * last_cmd_;
  }
  if (!velocity_.allFinite()) {
    velocity_.setZero();
  }
  if (latency_ > 0.0) {
    pose = mpc::step(layout_, pose, velocity_, latency_);  // where the command will act
  }

  // At the goal: turn to its heading (if the robot can turn in place), then stop
  const auto & goal = path.poses.back().pose;
  double xy_tol = xy_goal_tolerance_, yaw_tol = yaw_goal_tolerance_;
  if (nav_state.has("goal_tolerance.position")) {
    xy_tol = std::min(xy_tol, nav_state.get_safe<double>("goal_tolerance.position"));
  }
  if (nav_state.has("goal_tolerance.yaw")) {
    yaw_tol = std::min(yaw_tol, nav_state.get_safe<double>("goal_tolerance.yaw"));
  }
  Eigen::Vector3d to_goal(goal.position.x - pose(0), goal.position.y - pose(1), 0.0);
  if (layout_.aerial()) {to_goal.z() = goal.position.z - pose(2);}
  const double goal_dist = to_goal.norm();
  const auto & q = goal.orientation;
  const bool goal_has_yaw = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5;
  if (goal_dist <= xy_tol || (aligning_ && goal_dist <= 2.0 * xy_tol)) {
    const double yaw_error = goal_has_yaw ? mpc::angle_diff(yaw_of(q), pose(layout_.yaw)) : 0;
    optimizer_.reset();
    rotating_ = false;
    if (layout_.turns_in_place() && std::abs(yaw_error) > yaw_tol) {
      aligning_ = true;
      Eigen::VectorXd v = Eigen::VectorXd::Zero(layout_.nv);
      v(layout_.wz) = std::clamp(
        goal_align_gain_ * yaw_error, -limits_.max_angular_vel, limits_.max_angular_vel);
      publish_command(nav_state, v);
    } else {
      aligning_ = false;
      publish_command(nav_state, Eigen::VectorXd::Zero(layout_.nv));
    }
    return;
  }
  aligning_ = false;
  last_result_ = mpc::MPCResult();

  reference_.set_path(path);
  if (reference_.empty()) {
    stop(nav_state);
    return;
  }

  obstacles_.set(obstacle_points(nav_state, pose), layout_.aerial());
  const double speed = layout_.aerial() ?
    std::hypot(velocity_(0), velocity_(layout_.vz)) : std::abs(velocity_(0));
  const Eigen::Vector3d position(pose(0), pose(1), layout_.aerial() ? pose(2) : 0.0);
  const auto ref = reference_.build(
    position, pose(layout_.yaw), speed, params.horizon_steps, params.dt, speed_limits_,
    &obstacles_, shape_, params.safety_margin);

  // A differential robot facing away from the path turns in place first
  if (model_ == mpc::MotionModel::Diff && !ref.poses.empty()) {
    const double heading_error = mpc::angle_diff(ref.poses[0](layout_.yaw), pose(layout_.yaw));
    if ((std::abs(heading_error) > rotate_to_heading_angle_ ||
      (rotating_ && std::abs(heading_error) > 0.5 * rotate_to_heading_angle_)) &&
      std::abs(velocity_(0)) < 0.25 * std::max(limits_.max_linear_vel, 0.1))
    {
      rotating_ = true;
      optimizer_.reset();
      Eigen::VectorXd v = Eigen::VectorXd::Zero(layout_.nv);
      v(layout_.wz) = std::clamp(
        goal_align_gain_ * heading_error, -limits_.max_angular_vel, limits_.max_angular_vel);
      publish_command(nav_state, v);
      return;
    }
  }
  rotating_ = false;

  mpc::MPCInput input;
  input.pose = pose;
  input.velocity = velocity_;
  input.last_cmd = last_cmd_;
  input.reference = &ref;
  input.obstacles = &obstacles_;
  input.shape = &shape_;
  last_result_ = optimizer_.compute(input);

  const auto & logger = get_node()->get_logger();
  auto & clock = *get_node()->get_clock();
  if (last_result_.status == mpc::QPStatus::Solved ||
    last_result_.status == mpc::QPStatus::Inaccurate)
  {
    publish_command(nav_state, last_result_.cmd);
  } else {
    RCLCPP_WARN_THROTTLE(
      logger, clock, 1000, "[%s] QP %s: braking", get_plugin_name().c_str(),
      status_name(last_result_.status));
    publish_command(nav_state, braking(elapsed));
  }
  if (last_result_.solve_ms > 1000.0 * params.dt) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, 5000, "[%s] the QP took %.1f ms (%d variables, %d constraints)",
      get_plugin_name().c_str(), last_result_.solve_ms, last_result_.variables,
      last_result_.constraints);
  }

  if (predicted_pub_->get_subscription_count() > 0 ||
    reference_pub_->get_subscription_count() > 0)
  {
    publish_paths(
      last_result_.predicted, ref.poses, RTTFBuffer::getInstance()->get_tf_info().map_frame);
  }
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MPCController, easynav::ControllerMethodBase)
