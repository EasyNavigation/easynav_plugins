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
/// \brief Implementation of the MPCController class.

#include "easynav_mpc_controller/MPCController.hpp"

namespace easynav
{

MPCController::MPCController() {}

MPCController::~MPCController() = default;

std::expected<void, std::string>
MPCController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->declare_parameter<double>(plugin_name + ".dt", dt_);
  node->declare_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->declare_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);
  node->declare_parameter<bool>(plugin_name + ".verbose", verbose_);

  node->get_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->get_parameter<double>(plugin_name + ".dt", dt_);
  node->get_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->get_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);
  node->get_parameter<bool>(plugin_name + ".verbose", verbose_);

  optimizer_ = std::make_unique<MPCOptimizer>();

  mpc_path_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/mpc/path", 10);

  return {};
}

void
MPCController::publish_mpc_path(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & orientation, const std::vector<double> & best_vel)
{
  visualization_msgs::msg::MarkerArray path;
  visualization_msgs::msg::Marker points;
  points.header.frame_id = "map";
  points.header.stamp = rclcpp::Clock().now();
  points.ns = "mpc_path";
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::LINE_STRIP;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.05;
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;
  points.color.a = 0.8;

  Eigen::Vector3d state;
  for (size_t i = 0; i + 1 < best_vel.size(); i += 2) {
    double v = best_vel[i];
    double w = best_vel[i + 1];
    geometry_msgs::msg::Point p;
    state = optimizer_->kinematic_model(position, orientation, v, w, dt_);
    p.x = state[0];
    p.y = state[1];
    p.z = position[2] + 0.5;
    points.points.push_back(p);
  }

  path.markers.push_back(points);
  mpc_path_pub_->publish(path);
}

void
MPCController::update_rt(NavState & nav_state)
{
  if (!nav_state.has("path") || !nav_state.has("robot_pose") || !nav_state.has("points")) {
    if(verbose_) {
      std::cout << "No Path, No Points or No Robot Pose" << std::endl;
    }
    return;
  }

  const auto path = nav_state.get<nav_msgs::msg::Path>("path");
  if (path.poses.empty()) {
    // If the path is empty, stop the robot
    cmd_vel_.header.frame_id = path.header.frame_id;
    cmd_vel_.header.stamp = get_node()->now();
    cmd_vel_.twist.linear.x = 0.0;
    cmd_vel_.twist.angular.z = 0.0;
    nav_state.set("cmd_vel", cmd_vel_);
    return;
  }

  int num_elements = path.poses.size();
  size_t local_horizon;
  if (num_elements > horizon_steps_) {
    local_horizon = horizon_steps_;
  } else {
    local_horizon = num_elements - 1;
  }
  const auto & last_pose = path.poses[local_horizon].pose.position;

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  const auto & filtered = PointPerceptionsOpsView(perceptions)
    .filter({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0})
    .fuse("map")
    .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .collapse({NAN, NAN, 0.1})
    .downsample(0.1)
    .as_points();

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
    Q_,
    R_,
    Rd_,
    qtheta_,
    static_cast<int>(horizon_steps_),
    dt_,
    filtered);

  NLoptCallbackData cbdata{ optimizer_.get(), &params };

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

  // Publish the computed velocity command
  cmd_vel_.header.frame_id = path.header.frame_id;
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.twist.linear.x = u[0];
  cmd_vel_.twist.angular.z = u[1];

  nav_state.set("cmd_vel", cmd_vel_);

  // Publish the path
  publish_mpc_path(params.x0, params.theta0, u);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MPCController, easynav::ControllerMethodBase)
