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
/// \brief Implementation of the GpsLocalizer class.

#include <expected>
#include "easynav_gps_localizer/GpsLocalizer.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

std::expected<void, std::string> GpsLocalizer::on_initialize()
{
  auto node = get_node();

  // Initialize the odometry message
  odom_.header.stamp = get_node()->now();
  odom_.header.frame_id = get_tf_prefix() + "map";
  odom_.child_frame_id = get_tf_prefix() + "base_link";

  // Create subscriber to GPS data
  gps_subscriber_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "robot/gps/fix", rclcpp::SensorDataQoS().reliable(),
    std::bind(&GpsLocalizer::gps_callback, this, std::placeholders::_1));

  // Create static broadcaster
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  // Create subscriber to IMU data
  imu_subscriber_ = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS().reliable(),
    std::bind(&GpsLocalizer::imu_callback, this, std::placeholders::_1));

  // Create publisher
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
    "robot/odom_gps", rclcpp::SensorDataQoS().reliable());

  // Create static transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = get_tf_prefix() + "map";
  transform.child_frame_id = get_tf_prefix() + "odom";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  static_broadcaster_->sendTransform(transform);

  RTTFBuffer::getInstance()->setTransform(transform, "easynav", true);

  time_1_ = get_node()->now().seconds();
  alpha_ = 0.99;

  return {};
}

void GpsLocalizer::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  gps_msg_ = std::move(*msg);
}


void GpsLocalizer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_msg_ = std::move(*msg);
}


void GpsLocalizer::update_rt(NavState & nav_state)
{
}

void GpsLocalizer::update(NavState & nav_state)
{
  // Convert GPS coordinates to UTM
  double lat = gps_msg_.latitude;
  double lon = gps_msg_.longitude;
  double utm_x, utm_y, roll, pitch, yaw_imu, yaw_gyro, yaw_filtered;
  int zone;
  bool northp;

  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, utm_x, utm_y);
  std::string utm_zone = std::to_string(zone) + (northp ? "N" : "S");

  if (origin_utm_ == geometry_msgs::msg::Point() &&
    gps_msg_ != sensor_msgs::msg::NavSatFix())
  {
    // Get first UTM coordinates
    origin_utm_.x = utm_x;
    origin_utm_.y = utm_y;
  }

  // Get XY cartesian coordinates respect to the origin
  odom_.header.stamp = gps_msg_.header.stamp;
  odom_.header.frame_id = get_tf_prefix() + "map";
  odom_.child_frame_id = get_tf_prefix() + "base_link";
  odom_.pose.pose.position.x = utm_x - origin_utm_.x;
  odom_.pose.pose.position.y = utm_y - origin_utm_.y;

  // Extract the yaw angle from the IMU data
  dt_ = get_node()->now().seconds() - time_1_;
  yaw_gyro = yaw_1_ + imu_msg_.angular_velocity.z * dt_;

  tf2::Quaternion q(
    imu_msg_.orientation.x,
    imu_msg_.orientation.y,
    imu_msg_.orientation.z,
    imu_msg_.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw_imu);

  // Yaw angle filtered using a complementary filter
  yaw_filtered = (yaw_gyro * alpha_) + (yaw_imu * (1 - alpha_));

  // Extract the yaw angle from the IMU data
  tf2::Quaternion q_filtered;
  q_filtered.setRPY(roll, pitch, yaw_filtered);
  odom_.pose.pose.orientation.x = q_filtered.x();
  odom_.pose.pose.orientation.y = q_filtered.y();
  odom_.pose.pose.orientation.z = q_filtered.z();
  odom_.pose.pose.orientation.w = q_filtered.w();
  yaw_1_ = yaw_gyro;
  time_1_ = get_node()->now().seconds();

  nav_state.set("robot_pose", odom_);
  odom_pub_->publish(odom_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GpsLocalizer, easynav::LocalizerMethodBase)
