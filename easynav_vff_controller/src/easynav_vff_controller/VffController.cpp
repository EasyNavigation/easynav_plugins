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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the VffController class.

#include <expected>

#include "easynav_vff_controller/VffController.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace easynav
{

std::expected<void, std::string> VffController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<float>(plugin_name + ".distance_obstacle_detection", 3.0);
  node->declare_parameter<float>(plugin_name + ".distance_to_goal", 1.0);
  node->declare_parameter<float>(plugin_name + ".obstacle_detection_x_min", 0.5);
  node->declare_parameter<float>(plugin_name + ".obstacle_detection_x_max", 10.0);
  node->declare_parameter<float>(plugin_name + ".obstacle_detection_y_min", -10.0);
  node->declare_parameter<float>(plugin_name + ".obstacle_detection_y_max", 10.0);
  node->declare_parameter<float>(plugin_name + ".obstacle_detection_z_min", 0.10);
  node->declare_parameter<float>(plugin_name + ".obstacle_detection_z_max", 1.00);
  node->declare_parameter<double>(plugin_name + ".max_speed", 0.8);
  node->declare_parameter<double>(plugin_name + ".max_angular_speed", 1.5);

  node->get_parameter<float>(plugin_name + ".distance_obstacle_detection",
      distance_obstacle_detection_);
  node->get_parameter<float>(plugin_name + ".distance_to_goal", distance_to_goal_);
  node->get_parameter<float>(plugin_name + ".obstacle_detection_x_min", obstacle_detection_x_min_);
  node->get_parameter<float>(plugin_name + ".obstacle_detection_x_max", obstacle_detection_x_max_);
  node->get_parameter<float>(plugin_name + ".obstacle_detection_y_min", obstacle_detection_y_min_);
  node->get_parameter<float>(plugin_name + ".obstacle_detection_y_max", obstacle_detection_y_max_);
  node->get_parameter<float>(plugin_name + ".obstacle_detection_z_min", obstacle_detection_z_min_);
  node->get_parameter<float>(plugin_name + ".obstacle_detection_z_max", obstacle_detection_z_max_);
  node->get_parameter<double>(plugin_name + ".max_speed", max_speed_);
  node->get_parameter<double>(plugin_name + ".max_angular_speed", max_angular_speed_);

  // Initialize the odometry message
  cmd_vel_.header.stamp = node->now();
  cmd_vel_.header.frame_id = "base_link";
  cmd_vel_.twist.linear.x = 0.0;
  cmd_vel_.twist.linear.y = 0.0;
  cmd_vel_.twist.linear.z = 0.0;
  cmd_vel_.twist.angular.x = 0.0;
  cmd_vel_.twist.angular.y = 0.0;
  cmd_vel_.twist.angular.z = 0.0;

  // Publisher for visualization markers
  marker_array_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vff/markers_vff", 10);

  return {};
}

geometry_msgs::msg::TwistStamped VffController::get_cmd_vel()
{
  return cmd_vel_;
}

visualization_msgs::msg::MarkerArray
VffController::get_debug_vff(const VFFVectors & vff_vectors, std::string frame_id)
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE, frame_id));
  marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED, frame_id));
  marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN, frame_id));

  return marker_array;
}

visualization_msgs::msg::Marker
VffController::make_marker(
  const std::vector<double> & vector, VFFColor vff_color,
  std::string frame_id)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = get_node()->now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  geometry_msgs::msg::Point end;
  start.x = vector[0];
  start.y = vector[1];
  marker.points = {end, start};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;

  switch (vff_color) {
    case RED:
      marker.id = 0;
      marker.color.r = 1.0;
      break;
    case GREEN:
      marker.id = 1;
      marker.color.g = 1.0;
      break;
    case BLUE:
      marker.id = 2;
      marker.color.b = 1.0;
      break;
  }
  marker.color.a = 1.0;

  return marker;
}

double VffController::normalize_angle(double angle)
{
  angle = fmod(angle + M_PI, 2.0 * M_PI); // range [0, 2π)
  if (angle < 0) {
    angle += 2.0 * M_PI; // ensure positive
  }
  return angle - M_PI;
}

VFFVectors VffController::get_vff(
  double angle_error,
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud_,
  std::string frame_id)
{
  // Init vectors
  VFFVectors vff_vector;
  // Attractive vector points toward the goal
  vff_vector.attractive = {std::cos(angle_error), std::sin(angle_error)};
  vff_vector.repulsive = {0.0, 0.0};
  vff_vector.result = {0.0, 0.0};

  // Find the closest obstacle in the point cloud
  pcl::PointXYZ closest_point;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & point : pointcloud_) {
    const double distance = std::hypot(point.x, point.y);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = point;
    }
  }

  // Compute repulsive vector if an obstacle is within the threshold
  if (min_distance < distance_obstacle_detection_) {
    const double angle = std::atan2(closest_point.y, closest_point.x);
    const double opposite_angle = angle + M_PI;
    const double complementary_dist = distance_obstacle_detection_ - min_distance;

    vff_vector.repulsive[0] = std::cos(opposite_angle) * complementary_dist;
    vff_vector.repulsive[1] = std::sin(opposite_angle) * complementary_dist;
  }

  // Combine attractive and repulsive vectors
  vff_vector.result[0] = vff_vector.attractive[0] + vff_vector.repulsive[0];
  vff_vector.result[1] = vff_vector.attractive[1] + vff_vector.repulsive[1];


  if (marker_array_pub_->get_subscription_count() > 0) {
    // Publish debug markers
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cmd_vel_.header.frame_id;
    marker.header.stamp = get_node()->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.id = 456;
    marker.pose.position.x = closest_point.x;
    marker.pose.position.y = closest_point.y;
    marker.pose.position.z = closest_point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    auto marker_array = get_debug_vff(vff_vector, frame_id);
    marker_array.markers.push_back(marker);

    marker_array_pub_->publish(marker_array);
  }

  return vff_vector;
}

void VffController::update_rt(const NavState & nav_state)
{
  nav_msgs::msg::Goals all_goals = nav_state.goals;
  nav_msgs::msg::Odometry odom = nav_state.odom;

  // Current position
  double current_x_ = odom.pose.pose.position.x;
  double current_y_ = odom.pose.pose.position.y;

  // If a goal is set
  if (!all_goals.goals.empty()) {
    // Nuevo goal recibido
    geometry_msgs::msg::Point new_goal;
    new_goal.x = all_goals.goals[0].pose.position.x;
    new_goal.y = all_goals.goals[0].pose.position.y;

    // Comprobar si es diferente al anterior
    bool new_goal_received = !has_previous_goal_ ||
      std::hypot(new_goal.x - previous_goal_.x, new_goal.y - previous_goal_.y) > 0.01;

    if (new_goal_received) {
      RCLCPP_INFO(get_node()->get_logger(), "New goal [%f, %f]", new_goal.x, new_goal.y);
      target_reached_ = false;
      previous_goal_ = new_goal;
      has_previous_goal_ = true;
    }

    goal_ = new_goal;

    // Calculate the difference in position
    double dx = goal_.x - current_x_;
    double dy = goal_.y - current_y_;

    // Calculate the Euclidean distance to the goal
    double distance = std::hypot(dx, dy);

    // Calculate the angle to the goal
    double bearing = normalize_angle(std::atan2(dy, dx));

    // Extract the yaw angle from the NavState
    tf2::Quaternion q(
      nav_state.odom.pose.pose.orientation.x, nav_state.odom.pose.pose.orientation.y,
      nav_state.odom.pose.pose.orientation.z, nav_state.odom.pose.pose.orientation.w);

    tf2::Matrix3x3 imu(q);
    double roll, pitch, yaw;
    imu.getRPY(roll, pitch, yaw);

    // Calculate the angle error
    double angle_error = normalize_angle(bearing - yaw);

    if (distance < distance_to_goal_) {
      if (target_reached_) {return;}
      // Target reached → stop the robot
      RCLCPP_INFO(get_node()->get_logger(), "Target reached");
      cmd_vel_.twist.linear.x = 0.0;
      cmd_vel_.twist.angular.z = 0.0;
      goal_ = geometry_msgs::msg::Point();
      target_reached_ = true;
      return;
    } else {
      target_reached_ = false;
    }

    auto fused =
      PerceptionsOpsView(nav_state.perceptions)
      .filter({-10.0, -10.0, -10.0}, {10.0, 10.0, 10.0})
      .fuse(cmd_vel_.header.frame_id)
      ->filter({obstacle_detection_x_min_, obstacle_detection_y_min_, obstacle_detection_z_min_},
        {obstacle_detection_x_max_, obstacle_detection_y_max_,
          obstacle_detection_z_max_}).as_points();

    // Get VFF vectors
    const VFFVectors & vff = get_vff(angle_error, fused, cmd_vel_.header.frame_id);

    // Use result vector to calculate output speed
    const auto & v = vff.result;
    double angle = atan2(v[1], v[0]);
    double module = sqrt(v[0] * v[0] + v[1] * v[1]);

    // Calculate the linear and angular velocities
    cmd_vel_.header.stamp = nav_state.timestamp;
    cmd_vel_.twist.linear.x = std::clamp(module, 0.0, max_speed_);
    cmd_vel_.twist.angular.z = std::clamp(angle, -max_angular_speed_, max_angular_speed_);
    RCLCPP_INFO(get_node()->get_logger(), "[distance: %.2f, yaw_error: %.2f]", distance,
        angle_error);
  }
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::VffController, easynav::ControllerMethodBase)
