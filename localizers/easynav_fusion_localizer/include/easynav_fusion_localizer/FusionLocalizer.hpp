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

#pragma once

#include <optional>
#include <memory>
#include <vector>
#include <string>

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_fusion_localizer/ukf_wrapper.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

constexpr double kNoOrientationCovariance = 1e5;

/**
 * @class FusionLocalizer
 * @brief Plugin de localización para EasyNav que integra el UKF de
 * robot_localization para la fusión de sensores.
 */
class FusionLocalizer : public easynav::LocalizerMethodBase
{
public:
  FusionLocalizer() = default;
  virtual ~FusionLocalizer() = default;

protected:
  /// Subscriber for the initial pose.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

  /// Callback for /initialpose.
  void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /// @brief Starts from the last known pose after a reconfiguration
  /// ("initial_pose.use_last_known").
  void on_last_known_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) override;

  /// @brief Whether to start from the last known pose after a reconfiguration.
  bool use_last_known_pose_ {true};

  /// @brief Last known pose, applied once the global filter has an estimate.
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_last_pose_;

protected:
  /**
   * @brief Hook de inicialización de MethodBase.
   *
   * Loads and initializes the UkfWrapper, which will load parameters
   * and create all robot_localization subscribers/publishers.
   *
   * @throws std::runtime_error if initialization fails.
   */
  void on_initialize() override;

  /**
   * @brief Hook de actualización RT (alta frecuencia) de LocalizerMethodBase.
   *
   * Esta función actúa como nuestro "timer" manual. Llama al ciclo
   * principal (periodicUpdate) del filtro UKF y actualiza el NavState
   * de easynav con la pose filtrada.
   */
  void update_rt(NavState & nav_state) override;

  /**
   * @brief Hook de actualización no-RT (baja frecuencia).
   *
   * En esta implementación, toda la lógica principal reside en update_rt.
   */
  void update(NavState & nav_state) override;

private:
  std::unique_ptr<robot_localization::UkfWrapper> ukf_local_{nullptr};
  std::unique_ptr<robot_localization::UkfWrapper> ukf_global_{nullptr};

  bool has_global_filter_{false};
  bool has_local_filter_{false};

  int n_gps_sensors_{0};

  geometry_msgs::msg::PoseWithCovarianceStamped
  navsatfix_to_pose(const sensor_msgs::msg::NavSatFix & navsat_msg);

  sensor_msgs::msg::NavSatFix
  odom_to_navsatfix(const nav_msgs::msg::Odometry & odom_msg);

  double latitude_origin_{0.0};
  double longitude_origin_{0.0};
  double altitude_origin_{0.0};
  double UTM_origin_x_{0.0};
  double UTM_origin_y_{0.0};
  double UTM_origin_z_{0.0};
  std::string UTM_zone_;
  int UTM_zone_number_{0};
  bool UTM_zone_northp_{true};

  std::vector<rclcpp::Time> last_gps_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_{nullptr};
  std::string navsatfix_topic_;

  bool first_pose_received_{false};
};

}  // namespace easynav
