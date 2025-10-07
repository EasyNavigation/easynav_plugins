// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.

#ifndef EASYNAV_SIMPLE_CONTROLLER__SIMPLECONTROLLER_HPP_
#define EASYNAV_SIMPLE_CONTROLLER__SIMPLECONTROLLER_HPP_

#include <memory>
#include <expected>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_simple_controller/PIDController.hpp"

namespace easynav
{

/// \brief A simple path-following controller using PID and look-ahead strategy.
class SimpleController : public ControllerMethodBase
{
public:
  SimpleController();

  /// \brief Destructor.
  ~SimpleController() override;

  /// \brief Initializes parameters and PID controllers.
  /// \return std::expected<void, std::string> success or error message.
  std::expected<void, std::string> on_initialize() override;

  /// \brief Updates the controller using the given NavState.
  /// \param nav_state Current navigation state, including odometry and planned path.
  void update_rt(NavState & nav_state) override;

protected:
  std::shared_ptr<PIDController> linear_pid_;   ///< PID controller for linear velocity.
  std::shared_ptr<PIDController> angular_pid_;  ///< PID controller for angular velocity.

  double max_linear_speed_{1.0};   ///< Maximum linear speed in m/s.
  double max_angular_speed_{1.0};  ///< Maximum angular speed in rad/s.
  double max_linear_acc_{0.3};     ///< Maximum linear acceleration in m/s².
  double max_angular_acc_{0.3};    ///< Maximum angular acceleration in rad/s².
  double look_ahead_dist_{1.0};    ///< Distance ahead of the robot to track in meters.
  double tolerance_dist_{0.05};    ///< Distance threshold to switch to orientation tracking.
  double k_rot_{0.5};

  double last_vlin_{0.0};          ///< Previous linear velocity for acceleration limiting.
  double last_vrot_{0.0};          ///< Previous angular velocity for acceleration limiting.

  rclcpp::Time last_update_ts_;    ///< Timestamp of the last control update.
  geometry_msgs::msg::TwistStamped twist_stamped_;  ///< Current velocity command.

  /// \brief Gets the reference pose at look-ahead distance on the path.
  /// \param path The planned path.
  /// \param look_ahead Distance to look ahead in meters.
  /// \return PoseStamped representing the goal reference.
  geometry_msgs::msg::Pose get_ref_pose(
    const nav_msgs::msg::Path & path,
    double look_ahead);

  /// \brief Computes the Euclidean distance between two poses.
  /// \param a First pose.
  /// \param b Second pose.
  /// \return Distance in meters.
  double get_distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);

  /// \brief Computes the angle between two points.
  /// \param from Start point.
  /// \param to End point.
  /// \return Angle in radians.
  double get_angle(const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to);

  /// \brief Computes the angular difference between two quaternions (yaw).
  /// \param a First quaternion.
  /// \param b Second quaternion.
  /// \return Angle difference in radians within [-π, π].
  double get_diff_angle(
    const geometry_msgs::msg::Quaternion & a,
    const geometry_msgs::msg::Quaternion & b);
};

}  // namespace easynav

#endif  // EASYNAV_SIMPLE_CONTROLLER__SIMPLECONTROLLER_HPP_
