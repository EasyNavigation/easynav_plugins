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
/// \brief Declaration of the GpsLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
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
   * @throws std::runtime_error if initialization fails.
   */
  virtual void on_initialize() override;

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
  virtual void update_rt(NavState & nav_state) override;

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
  virtual void update(NavState & nav_state) override;

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
   * @brief odometry publisher
   *
   * This publisher publsh in a topic the odom value
   * It is used to validate the localization
  */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

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

  double alpha_, dt_, yaw_1_, time_1_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_
