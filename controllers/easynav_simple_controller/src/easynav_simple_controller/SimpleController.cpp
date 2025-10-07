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
/// \brief Implementation of the SimpleController class.

#include <expected>

#include "tf2/utils.hpp"

#include "easynav_simple_controller/SimpleController.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{


SimpleController::SimpleController()
{
}

SimpleController::~SimpleController() = default;

std::expected<void, std::string>
SimpleController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".max_linear_speed", max_linear_speed_);
  node->declare_parameter<double>(plugin_name + ".max_angular_speed", max_angular_speed_);
  node->declare_parameter<double>(plugin_name + ".max_linear_acc", max_linear_acc_);
  node->declare_parameter<double>(plugin_name + ".max_angular_acc", max_angular_acc_);
  node->declare_parameter<double>(plugin_name + ".look_ahead_dist", look_ahead_dist_);
  node->declare_parameter<double>(plugin_name + ".tolerance_dist", tolerance_dist_);
  node->declare_parameter<double>(plugin_name + ".k_rot", k_rot_);

  node->get_parameter<double>(plugin_name + ".max_linear_speed", max_linear_speed_);
  node->get_parameter<double>(plugin_name + ".max_angular_speed", max_angular_speed_);
  node->get_parameter<double>(plugin_name + ".max_linear_acc", max_linear_acc_);
  node->get_parameter<double>(plugin_name + ".max_angular_acc", max_angular_acc_);
  node->get_parameter<double>(plugin_name + ".look_ahead_dist", look_ahead_dist_);
  node->get_parameter<double>(plugin_name + ".tolerance_dist", tolerance_dist_);
  node->get_parameter<double>(plugin_name + ".k_rot", k_rot_);

  linear_pid_ = std::make_shared<PIDController>(0.01, look_ahead_dist_, 0.1, max_linear_speed_);
  angular_pid_ = std::make_shared<PIDController>(0.01, M_PI, 0.1, max_angular_speed_);

  last_vlin_ = 0.0;
  last_vrot_ = 0.0;
  last_update_ts_ = node->now();

  return {};
}


void
SimpleController::update_rt(NavState & nav_state)
{
  double dt = (get_node()->now() - last_update_ts_).seconds();
  last_update_ts_ = get_node()->now();

  if (!nav_state.has("path")) {return;}
  if (!nav_state.has("robot_pose")) {return;}
  if (!nav_state.has("path")) {return;}

  const auto path = nav_state.get<nav_msgs::msg::Path>("path");

  if (path.poses.empty()) {
    twist_stamped_.header.frame_id = path.header.frame_id;
    twist_stamped_.header.stamp = get_node()->now();
    twist_stamped_.twist.linear.x = 0.0;
    twist_stamped_.twist.angular.z = 0.0;
    nav_state.set("cmd_vel", twist_stamped_);
    return;
  }

  auto ref_pose = get_ref_pose(path, look_ahead_dist_);
  const auto pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;

  double dist = get_distance(pose, ref_pose);


  double angle = get_angle(pose.position, ref_pose.position) - tf2::getYaw(pose.orientation);
  while(angle > M_PI) {angle -= 2.0 * M_PI;}
  while(angle < -M_PI) {angle += 2.0 * M_PI;}

  double vlin = 0.0;
  double vrot = 0.0;


  if (dist < tolerance_dist_) {
    double diff_angle = get_diff_angle(pose.orientation, ref_pose.orientation);
    vrot = angular_pid_->get_output(diff_angle);
  } else {
    vrot = angular_pid_->get_output(angle);
    vlin = std::max(0.0, linear_pid_->get_output(dist) - k_rot_ * abs(vrot));
  }


  double max_dvlin = max_linear_acc_ * dt;
  double max_dvrot = max_angular_acc_ * dt;
  vlin = std::clamp(vlin, last_vlin_ - max_dvlin, last_vlin_ + max_dvlin);
  vrot = std::clamp(vrot, last_vrot_ - max_dvrot, last_vrot_ + max_dvrot);
  vlin = std::clamp(vlin, -max_linear_speed_, max_linear_speed_);
  vrot = std::clamp(vrot, -max_angular_speed_, max_angular_speed_);


  last_vlin_ = vlin;
  last_vrot_ = vrot;

  twist_stamped_.header.frame_id = path.header.frame_id;
  twist_stamped_.header.stamp = get_node()->now();
  twist_stamped_.twist.linear.x = vlin;
  twist_stamped_.twist.angular.z = vrot;

  nav_state.set("cmd_vel", twist_stamped_);
}


geometry_msgs::msg::Pose
SimpleController::get_ref_pose(const nav_msgs::msg::Path & path, double look_ahead)
{
  double accumulated = 0.0;
  const auto & poses = path.poses;
  if (poses.empty()) {return geometry_msgs::msg::Pose();}

  for (size_t i = 1; i < poses.size(); ++i) {
    accumulated += get_distance(poses[i - 1].pose, poses[i].pose);
    if (accumulated >= look_ahead) {
      return poses[i].pose;
    }
  }

  return poses.back().pose;
}

double
SimpleController::get_distance(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  double dx = b.position.x - a.position.x;
  double dy = b.position.y - a.position.y;
  return std::hypot(dx, dy);
}

double
SimpleController::get_angle(
  const geometry_msgs::msg::Point & from,
  const geometry_msgs::msg::Point & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

double
SimpleController::get_diff_angle(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  double yaw_a = tf2::getYaw(a);
  double yaw_b = tf2::getYaw(b);
  double diff = yaw_b - yaw_a;
  while (diff > M_PI) {diff -= 2 * M_PI;}
  while (diff < -M_PI) {diff += 2 * M_PI;}
  return diff;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SimpleController, easynav::ControllerMethodBase)
