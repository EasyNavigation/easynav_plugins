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

/// \file
/// \brief Pure regulation heuristics for the Regulated Pure Pursuit controller.
///
/// These are the same regulation terms described in:
/// S. Macenski, S. Singh, F. Martin, J. Gines, "Regulated Pure Pursuit for Robot Path Tracking",
/// Autonomous Robots, 2023 (https://arxiv.org/abs/2305.20026), ported from
/// nav2_regulated_pure_pursuit_controller. Since EasyNav controllers have no costmap available,
/// the cost-based regulation term is replaced by an equivalent one driven by the distance to the
/// nearest point in the fused point-cloud perception (see \ref easynav::RegulatedPurePursuitController).

#ifndef EASYNAV_REGULATED_PP_CONTROLLER__REGULATION_FUNCTIONS_HPP_
#define EASYNAV_REGULATED_PP_CONTROLLER__REGULATION_FUNCTIONS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace easynav
{

namespace heuristics
{

/// \brief Curvature of the arc from the robot (at the local origin, facing +x) to a lookahead point.
/// \param lookahead_x X coordinate of the lookahead point in the robot frame.
/// \param lookahead_y Y coordinate of the lookahead point in the robot frame.
/// \return Signed curvature (1 / turning radius) of the arc.
inline double calculateCurvature(double lookahead_x, double lookahead_y)
{
  const double carrot_dist2 = lookahead_x * lookahead_x + lookahead_y * lookahead_y;
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_y / carrot_dist2;
  } else {
    return 0.0;
  }
}

/// \brief Apply curvature-based regulation on the linear velocity.
///
/// Slows the robot down when the path curvature radius is smaller than \p min_radius,
/// so sharp turns are taken more conservatively.
/// \param raw_linear_vel Desired linear velocity before regulation.
/// \param curvature Curvature of the path at the regulation lookahead point.
/// \param min_radius Turning radius below which regulation kicks in.
/// \return Regulated linear velocity.
inline double curvatureConstraint(
  const double raw_linear_vel, const double curvature, const double min_radius)
{
  const double radius = std::fabs(1.0 / curvature);
  if (radius < min_radius) {
    return raw_linear_vel * (1.0 - (std::fabs(radius - min_radius) / min_radius));
  } else {
    return raw_linear_vel;
  }
}

/// \brief Apply obstacle-proximity regulation on the linear velocity.
///
/// EasyNav controllers do not have access to a costmap, so this heuristic replaces
/// nav2_regulated_pure_pursuit_controller's cost-based regulation with an equivalent term based on
/// the distance to the closest obstacle point observed by the robot's perception.
/// \param raw_linear_vel Desired linear velocity before regulation.
/// \param min_obstacle_distance Distance (m) to the closest known obstacle ahead of the robot.
/// \param obstacle_scaling_dist Distance below which the regulation is triggered.
/// \param obstacle_scaling_gain Gain (<= 1.0) applied when scaling down the velocity.
/// \return Regulated linear velocity.
inline double obstacleConstraint(
  const double raw_linear_vel,
  const double min_obstacle_distance,
  const double obstacle_scaling_dist,
  const double obstacle_scaling_gain)
{
  if (!std::isfinite(min_obstacle_distance)) {
    return raw_linear_vel;
  }

  if (min_obstacle_distance < obstacle_scaling_dist) {
    const double clamped_dist = std::max(0.0, min_obstacle_distance);
    return raw_linear_vel * (obstacle_scaling_gain * clamped_dist / obstacle_scaling_dist);
  }

  return raw_linear_vel;
}

/// \brief Compute the scale factor to apply for linear velocity regulation on approach to goal.
/// \param remaining_path_distance Integrated distance left to travel along the path.
/// \param euclidean_dist_to_goal Straight-line distance from the robot to the goal.
/// \param approach_velocity_scaling_dist Distance away from goal at which to apply the heuristic.
/// \return A scale factor in [0.0, 1.0].
inline double approachVelocityScalingFactor(
  const double remaining_path_distance,
  const double euclidean_dist_to_goal,
  const double approach_velocity_scaling_dist)
{
  // Waiting to apply the threshold based on integrated distance ensures we don't erroneously
  // apply approach scaling on curvy paths still far from the goal in a straight line.
  if (remaining_path_distance < approach_velocity_scaling_dist) {
    return euclidean_dist_to_goal / approach_velocity_scaling_dist;
  } else {
    return 1.0;
  }
}

/// \brief Velocity on approach to goal heuristic regulation term.
/// \param constrained_linear_vel Linear velocity already constrained by other heuristics.
/// \param remaining_path_distance Integrated distance left to travel along the path.
/// \param euclidean_dist_to_goal Straight-line distance from the robot to the goal.
/// \param min_approach_velocity Minimum velocity to use on approach to goal.
/// \param approach_velocity_scaling_dist Distance away from goal at which to apply the heuristic.
/// \return Velocity after regulation via approach-to-goal slow-down.
inline double approachVelocityConstraint(
  const double constrained_linear_vel,
  const double remaining_path_distance,
  const double euclidean_dist_to_goal,
  const double min_approach_velocity,
  const double approach_velocity_scaling_dist)
{
  const double velocity_scaling = approachVelocityScalingFactor(
    remaining_path_distance, euclidean_dist_to_goal, approach_velocity_scaling_dist);
  double approach_vel = constrained_linear_vel * velocity_scaling;

  if (approach_vel < min_approach_velocity) {
    approach_vel = min_approach_velocity;
  }

  return std::min(constrained_linear_vel, approach_vel);
}

}  // namespace heuristics

}  // namespace easynav

#endif  // EASYNAV_REGULATED_PP_CONTROLLER__REGULATION_FUNCTIONS_HPP_
