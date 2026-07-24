// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// and is a derivative work of nav2_regulated_pure_pursuit_controller,
// ported to the EasyNav plugin architecture.
//
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

#ifndef EASYNAV_REGULATED_PP_CONTROLLER__REGULATEDPUREPURSUITCONTROLLER_HPP_
#define EASYNAV_REGULATED_PP_CONTROLLER__REGULATEDPUREPURSUITCONTROLLER_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

namespace easynav
{

/// \brief Port of nav2_regulated_pure_pursuit_controller to EasyNav.
///
/// Implements the Regulated Pure Pursuit algorithm described in:
/// S. Macenski, S. Singh, F. Martin, J. Gines, "Regulated Pure Pursuit for Robot Path Tracking",
/// Autonomous Robots, 2023 (https://arxiv.org/abs/2305.20026), plus the optional Dynamic Window
/// Pure Pursuit (DWPP) extension. See the package README for the adaptations made to fit
/// EasyNav's design (no costmap, no separate goal-checker/TF-transform plugins).
class RegulatedPurePursuitController : public ControllerMethodBase
{
public:
  RegulatedPurePursuitController();

  /// \brief Destructor.
  ~RegulatedPurePursuitController() override;

  /// \brief Declares and reads parameters.
  /// \throws std::runtime_error on initialization failure.
  void on_initialize() override;

  /// \brief Computes the next velocity command from the current NavState.
  /// \param nav_state Current navigation state, including odometry and planned path.
  void update_rt(NavState & nav_state) override;

protected:
  // --- Lookahead ---
  double lookahead_dist_{0.6};              ///< Fixed lookahead distance (m).
  double min_lookahead_dist_{0.3};          ///< Minimum velocity-scaled lookahead distance (m).
  double max_lookahead_dist_{0.9};          ///< Maximum velocity-scaled lookahead distance (m).
  double lookahead_time_{1.5};              ///< Lookahead gain: seconds projected at current speed.
  bool use_velocity_scaled_lookahead_dist_{false};  ///< Use velocity-scaled lookahead distance.

  // --- Velocity / acceleration limits ---
  double max_linear_vel_{0.5};    ///< Maximum linear velocity (m/s).
  double min_linear_vel_{-0.5};   ///< Minimum linear velocity, used when use_dynamic_window is true (m/s).
  double max_angular_vel_{2.5};   ///< Maximum angular velocity (rad/s).
  double min_angular_vel_{-2.5};  ///< Minimum angular velocity, used when use_dynamic_window is true (rad/s).
  double max_linear_accel_{2.5};  ///< Maximum linear acceleration (m/s^2).
  double max_linear_decel_{2.5};  ///< Maximum linear deceleration (m/s^2).
  double max_angular_accel_{3.2}; ///< Maximum angular acceleration (rad/s^2).
  double max_angular_decel_{3.2}; ///< Maximum angular deceleration (rad/s^2).
  bool use_dynamic_window_{false};  ///< Use the Dynamic Window Pure Pursuit (DWPP) extension.
  bool allow_reversing_{false};     ///< Allow driving backwards when the carrot is behind the robot.

  // --- Rotate to heading ---
  bool use_rotate_to_heading_{true};             ///< Enable rotate-in-place behaviors.
  double rotate_to_heading_angular_vel_{1.8};    ///< Angular velocity used while rotating in place.
  double rotate_to_heading_min_angle_{0.785};    ///< Angle to path beyond which to rotate in place.

  // --- Curvature regulation ---
  bool use_regulated_linear_velocity_scaling_{true};  ///< Enable curvature-based regulation.
  double regulated_linear_scaling_min_radius_{0.9};   ///< Turning radius that triggers regulation.
  double regulated_linear_scaling_min_speed_{0.25};   ///< Minimum speed kept under regulation.
  bool use_fixed_curvature_lookahead_{false};         ///< Use a separate lookahead for curvature.
  double curvature_lookahead_dist_{1.0};              ///< Distance of the fixed curvature lookahead.
  bool interpolate_curvature_after_goal_{false};      ///< Extrapolate curvature carrot past the goal.

  // --- Obstacle-proximity regulation (adapts the costmap-based term to EasyNav's perception) ---
  bool use_obstacle_regulated_linear_velocity_scaling_{false};  ///< Enable obstacle-proximity regulation.
  double obstacle_scaling_dist_{0.3};   ///< Distance below which obstacle regulation is triggered (m).
  double obstacle_scaling_gain_{1.0};   ///< Gain (<=1.0) applied when scaling down the velocity.

  // --- Approach to goal ---
  double min_approach_linear_velocity_{0.05};  ///< Minimum linear velocity while approaching goal.
  double approach_velocity_scaling_dist_{1.0}; ///< Remaining-path distance at which to start slowing.

  // --- Goal tolerances (fallback values if "goal_tolerance.*" is not present in NavState) ---
  double xy_goal_tolerance_{0.25};   ///< Fallback positional tolerance to the goal (m).
  double yaw_goal_tolerance_{0.25};  ///< Fallback angular tolerance to the goal (rad).

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr curvature_lookahead_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_rotating_to_heading_pub_;

  rclcpp::Time last_update_ts_;              ///< Timestamp of the last control update.
  double last_linear_vel_{0.0};              ///< Last commanded linear velocity (open-loop feedback).
  double last_angular_vel_{0.0};             ///< Last commanded angular velocity (open-loop feedback).
  bool is_rotating_to_heading_{false};       ///< Whether the controller is currently rotating in place.
  geometry_msgs::msg::TwistStamped cmd_vel_; ///< Current velocity command.

  /// \brief Publishes a zero-velocity command and resets the open-loop velocity feedback.
  void stop(NavState & nav_state, const std_msgs::msg::Header & header);

  /// \brief Computes the lookahead distance to search for the carrot pose.
  /// \param linear_vel Linear velocity used to scale the lookahead distance, if enabled.
  double getLookAheadDistance(double linear_vel) const;

  /// \brief Index of the path pose closest to \p robot_position.
  ///
  /// Since EasyNav controllers receive the full, un-pruned global path (see the README), the
  /// search for the carrot pose must start from this index instead of from the beginning of the
  /// path: otherwise, once the robot has advanced more than a lookahead distance away from the
  /// path's start pose, that start pose itself would (incorrectly) be picked up as "farther than
  /// the lookahead distance" and returned as the carrot, even though it lies behind the robot.
  static std::size_t findClosestPoseIndex(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & robot_position);

  /// \brief Finds the pose on the path at (approximately) \p lookahead_dist from \p robot_position.
  ///
  /// Walks the path (in the same global frame as the robot pose), starting from the pose closest
  /// to the robot (see \ref findClosestPoseIndex), looking for the first pose from there onwards
  /// that is farther than \p lookahead_dist from the robot, then interpolates the exact carrot
  /// position on the segment leading to it via a circle/segment intersection. If the remainder of
  /// the path is closer than \p lookahead_dist, the path's last pose is returned (extrapolated one
  /// extra segment if \p interpolate_after_end is true and the path has at least two poses).
  static geometry_msgs::msg::Point getLookAheadPoint(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & robot_position,
    double lookahead_dist,
    bool interpolate_after_end = false);

  /// \brief Intersection of the segment [p1, p2] with the circle of radius \p r centered at \p center.
  /// \return The intersection point closer to \p p2, or \p p2 itself if there is no real intersection.
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r,
    const geometry_msgs::msg::Point & center);

  /// \brief Expresses a global-frame point in the robot's local frame (robot at origin, facing +x).
  static geometry_msgs::msg::Point toRobotFrame(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Pose & robot_pose,
    double robot_yaw);

  /// \brief Whether the robot should rotate in place towards \p angle_to_path.
  bool shouldRotateToPath(double angle_to_path) const;

  /// \brief Computes a kinematically-feasible rotate-in-place command towards \p angle_to_target.
  void rotateToHeading(
    double & linear_vel, double & angular_vel, double angle_to_target, double dt) const;

  /// \brief Applies curvature and obstacle-proximity regulation, then approach-to-goal scaling.
  void applyConstraints(
    double curvature,
    double min_obstacle_distance,
    double remaining_path_distance,
    double euclidean_dist_to_goal,
    double & linear_vel) const;

  /// \brief Distance (m) from the robot to the nearest fused perception point in front of it,
  /// within \p max_range. Returns +infinity if there is no perception data or no nearby point.
  double computeMinObstacleDistance(NavState & nav_state, double max_range) const;

  /// \brief Integrated path length from the robot position to the end of \p path.
  static double remainingPathDistance(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & robot_position);

  /// \brief Publishes a PointStamped marker for a carrot pose, if the publisher has subscribers.
  void publishCarrot(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & pub,
    const geometry_msgs::msg::Point & carrot,
    const std_msgs::msg::Header & header) const;
};

}  // namespace easynav

#endif  // EASYNAV_REGULATED_PP_CONTROLLER__REGULATEDPUREPURSUITCONTROLLER_HPP_
