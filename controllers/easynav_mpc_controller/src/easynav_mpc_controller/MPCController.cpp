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

Eigen::Vector3d
kinematic_model(const Eigen::Vector3d &x, double v, double w, double dt) 
{
  Eigen::Vector3d x_k1;
  x_k1(0) = x(0) + v * cos(x(2)) * dt;
  x_k1(1) = x(1) + v * sin(x(2)) * dt;
  x_k1(2) = x(2) + w * dt;
  return x_k1;
}

double 
cost_function(const std::vector<double> &u, std::vector<double> &grad, void *data)
{
  MPCParameters *params = reinterpret_cast<MPCParameters*>(data);
  Eigen::Vector3d x = params->x0;
  int N = params->N;
  double dt = params->dt;
  double cost = 0.0;
  if (!grad.empty()) {
    grad[0] = 0.0;
    grad[1] = 0.5 / sqrt(x[1]);
  }

  for (int i = 0; i < N; ++i) {
    double v = u[2*i];
    double w = u[2*i + 1];

    x = kinematic_model(x, v, w, dt);
    Eigen::Vector2d pos = x.head<2>();
    Eigen::Vector2d error = pos - params->goal;
    cost += error.squaredNorm() + 0.1 * (v*v + w*w); // ToDo quadratic function
  }

  return cost;

}

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

  node->get_parameter<int>(plugin_name + ".horizon_steps", horizon_steps_);
  node->get_parameter<double>(plugin_name + ".dt", dt_);
  node->get_parameter<double>(plugin_name + ".max_linear_velocity", max_lin_vel_);
  node->get_parameter<double>(plugin_name + ".max_angular_velocity", max_ang_vel_);



  return {};
}

void
MPCController::update_rt(NavState & nav_state)
{
  if (!nav_state.has("path") || !nav_state.has("robot_pose")) {
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

  MPCParameters params;
  params.x0 = {pose.position.x, pose.position.y, yaw_};
  const auto &last_pose = path.poses[num_elements - 1].pose.position;
  params.goal = Eigen::Vector2d(static_cast<double>(last_pose.x),
                                static_cast<double>(last_pose.y));
  params.N = horizon_steps_;
  params.dt = dt_;
  double minf;

  nlopt::opt opt(nlopt::LD_SLSQP, 2*horizon_steps_);
  opt.set_min_objective(cost_function, &params);

  std::vector<double> lb(2*horizon_steps_, -max_lin_vel_);
  std::vector<double> ub(2*horizon_steps_, max_lin_vel_);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_xtol_rel(1e-4);

  std::vector<double> u(2*horizon_steps_, 0.0);

  // nlopt::result result = opt.optimize(u, minf);
  
  try {
      nlopt::result result = opt.optimize(u, minf);
      if (result != nlopt::SUCCESS)
      {
        std::cerr << "Optimization Error: " << std::endl;
      }
  } catch (std::exception &e) {
      std::cerr << "Optimization Error: " << e.what() << std::endl;
  }

  // Publish the computed velocity command
  cmd_vel_.header.frame_id = path.header.frame_id;
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.twist.linear.x = u[0];
  cmd_vel_.twist.angular.z = u[1];

  nav_state.set("cmd_vel", cmd_vel_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MPCController, easynav::ControllerMethodBase)
