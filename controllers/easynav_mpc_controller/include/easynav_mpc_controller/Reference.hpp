// Copyright 2026 Intelligent Robotics Lab
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
/// \brief The reference the MPC tracks: poses and velocities sampled along the path ahead of the
/// robot, at the speed the path's curvature, the goal and the obstacles allow.

#ifndef EASYNAV_MPC_CONTROLLER__REFERENCE_HPP_
#define EASYNAV_MPC_CONTROLLER__REFERENCE_HPP_

#include <vector>

#include "Eigen/Core"
#include "nav_msgs/msg/path.hpp"

#include "easynav_mpc_controller/MotionModel.hpp"
#include "easynav_mpc_controller/Obstacles.hpp"

namespace easynav
{
namespace mpc
{

/// @brief What limits the reference speed.
struct SpeedLimits
{
  double max_vel {0.5};           ///< Along the path, horizontal (m/s).
  double max_vertical_vel {0.0};  ///< Aerial robots (m/s).
  double acc {0.5};               ///< Speeding up (m/s^2).
  double decel {1.0};             ///< Slowing down (m/s^2).
  double max_lateral_acc {1.0};   ///< v^2 * curvature (m/s^2).
  double max_angular_vel {1.0};   ///< v * curvature (rad/s).
  double approach_speed {0.1};    ///< Lowest speed while the goal is not reached (m/s).
};

/// @brief The reference over the horizon: N + 1 poses (from the robot's projection) and the N
/// velocities between them.
struct ReferenceTrajectory
{
  std::vector<Eigen::VectorXd> poses;
  std::vector<Eigen::VectorXd> velocities;
  double s0 {0.0};             ///< Robot's projection on the path (m).
  double remaining {0.0};      ///< Path left from the projection (m).
  double free_distance {0.0};  ///< Path ahead free of obstacles (m), up to the horizon.
};

/// @brief The path, prepared once per path: arc length, heading, curvature and slope, and the
/// robot's progress along it (searched forward only, so a path that turns back on itself is
/// followed in order).
class Reference
{
public:
  explicit Reference(MotionModel model = MotionModel::Diff);

  /// Uses \p path; progress starts over only if it is a different path.
  void set_path(const nav_msgs::msg::Path & path);
  bool empty() const {return pts_.size() < 2;}
  void reset() {pts_.clear(); progress_ = 0.0; speed_ = 0.0;}

  /// Projects \p position on the path ahead of the last projection; arc length.
  double project(const Eigen::Vector3d & position);

  /// The reference for \p steps of \p dt from the robot at \p position with speed \p speed (along
  /// the path). \p obstacles (if any) stop it before the first pose of the path where the robot
  /// (\p shape grown by \p margin) would touch one.
  ReferenceTrajectory build(
    const Eigen::Vector3d & position, double yaw, double speed, int steps, double dt,
    const SpeedLimits & limits, const Obstacles * obstacles, const RobotShape & shape,
    double margin);

  /// Path point, heading (unwrapped, averaged over curvature_window on each side) and curvature
  /// (the change of that heading) at arc length \p s (clamped to the path).
  Eigen::Vector3d point_at(double s) const;
  double heading_at(double s) const;
  double curvature_at(double s) const;
  double length() const {return s_.empty() ? 0.0 : s_.back();}

  /// Half length of the window the heading is averaged over (m): planners' paths zigzag on their
  /// grid, and turn at corners.
  double curvature_window {1.0};

  /// Half length of the moving average the path's points are smoothed with (m), 0: none. A
  /// robot with a turning radius R cannot follow a planner's corner: 1.45 R rounds a 90 deg one
  /// to R.
  double path_smoothing {0.0};

private:
  std::size_t segment(double s) const;
  double interp(const std::vector<double> & values, double s) const;
  double raw_heading(double s) const;
  void smooth();
  double free_distance(
    double s0, double horizon, const Obstacles & obstacles, const RobotShape & shape,
    double margin) const;

  ModelLayout layout_;
  std::vector<Eigen::Vector3d> pts_;
  std::vector<double> s_;        ///< Arc length at each point (3D for aerial, 2D otherwise).
  std::vector<double> heading_;  ///< Unwrapped heading of the segment that starts at each point.
  std::vector<double> slope_;    ///< dz/ds of that segment (aerial).
  std::vector<double> cum_heading_;  ///< Integral of the heading along the path, at each point.
  double progress_ {0.0};
  /// The reference's own speed (what it planned for now last cycle): the robot slowing down
  /// (e.g. to round a corner) must not slow the reference down with it, or neither moves on.
  double speed_ {0.0};
  nav_msgs::msg::Path last_path_;
};

/// @brief a - b wrapped to [-pi, pi].
double angle_diff(double a, double b);

}  // namespace mpc
}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__REFERENCE_HPP_
