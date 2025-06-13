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
/// \brief Declaration of the GpsLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_

#include <expected>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include <GeographicLib/UTMUPS.hpp>

namespace easynav
{

/**
 * @class GpsLocalizer
 * @brief A default "do-nothing" implementation of the LocalizerMethodBase.
 *
 * This class implements the interface required by the Easy Navigation framework
 * for localization but does not perform any actual computation. It is useful as
 * a placeholder, example, or fallback when no real localization plugin is configured.
 */
class GpsLocalizer : public easynav::LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  GpsLocalizer() = default;

  /**
   * @brief Default destructor.
   */
  ~GpsLocalizer() = default;

  /**
   * @brief Initialize the localization method.
   *
   * This override may be used to set up internal resources. By default, it simply succeeds.
   *
   * @return std::expected<void, std::string> Returns success or an error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Retrieve the current robot odometry.
   *
   * Returns the last stored odometry message representing the estimated robot pose.
   * This method should not perform any actual localization computation.
   *
   * @return nav_msgs::msg::Odometry Current localization estimate.
   */
  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this implementation, the method updates the odometry based on the latest GPS data.
   * GPS data is used to set the robot's position in the UTM coordinate system.
   * The origin of the UTM coordinates is set when the first GPS message is received.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update_rt(const NavState & nav_state) override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this implementation, the method updates the odometry based on the latest GPS data.
   * GPS data is used to set the robot's position in the UTM coordinate system.
   * The origin of the UTM coordinates is set when the first GPS message is received.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update(const NavState & nav_state) override;

private:
  /**
   * @brief Internal representation of the robot's current odometry.
   *
   * Stores the estimated position and velocity of the robot.
   */
  nav_msgs::msg::Odometry odom_;

  /**
   * @brief Origin UTM coordinates.
   *
   * Stores the first UTM coordinates obtained from the GPS data.
   * It is used to calculate the relative position of the robot in the UTM coordinate system.
   */
  geometry_msgs::msg::Point origin_utm_;

  /**
   * @brief Subscriber for GPS data.
   *
   * This subscriber listens to the GPS data published on the "/robot/gps/fix" topic.
   * It is used to obtain the robot's position in the UTM coordinate system.
   */
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

  /**
   * @brief Subscriber to the IMU data.
   *
   * This subscriber listens to the IMU data published on the "imu/data" topic.
   * It is used to obtain the robot's orientation.
   * The IMU data is used to calculate the yaw angle of the robot.
   */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  /**
   * @brief GPS message.
   *
   * Stores the latest GPS data received from the subscriber.
   * It contains information about the robot's position and status.
   */
  sensor_msgs::msg::NavSatFix gps_msg_;

  /**
   * @brief IMU message.
   *
   * Stores the latest IMU data received from the subscriber.
   * It contains information about the robot's orientation.
   */
  sensor_msgs::msg::Imu imu_msg_;

  /**
   * @brief Static broadcaster for static transforms.
   *
   * This broadcaster is used to broadcast static transforms between coordinate frames.
   * It is used to broadcast the static transform between the "map" frame and the "odom" frame.
   */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  /**
   * @brief Callback function for GPS data.
   *
   * This function is called whenever a new GPS message is received.
   * It updates the internal GPS message with the latest data.
   *
   * @param msg The received GPS message.
   */
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Callback function for the IMU data.
   *
   * This function is called whenever a new IMU data message is received.
   * It updates the internal yaw angle of the robot based on the IMU data.
   *
   * @param msg The IMU data message.
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_
