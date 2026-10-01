// Copyright 2026 Intelligent Robotics Lab
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
/// \brief Declaration of compute_nearest_obstacle(), a static (velocity-independent) proximity
/// query shared by recovery evaluators and mitigators.

#ifndef EASYNAV_DIAGNOSTIC_RECOVERY__OBSTACLEPROXIMITY_HPP_
#define EASYNAV_DIAGNOSTIC_RECOVERY__OBSTACLEPROXIMITY_HPP_

#include <limits>

#include "easynav_common/types/NavState.hpp"

namespace easynav_diagnostic_recovery
{

/**
 * @struct ObstacleProximity
 * @brief Distance and bearing (both in the robot frame) of the nearest point-cloud perception.
 */
struct ObstacleProximity
{
  /// @brief Distance to the nearest perceived point (m). +infinity if there is no perception.
  double distance {std::numeric_limits<double>::infinity()};

  /// @brief Bearing to the nearest perceived point (rad), 0 = straight ahead, atan2 convention.
  double bearing {0.0};

  /// @brief Whether any point perception had data. If not, distance = +infinity means "unknown",
  /// not "nothing near".
  bool perceived {false};
};

/**
 * @brief Finds the nearest point-cloud perception to the robot, regardless of "cmd_vel".
 *
 * Unlike CollisionSafetyReflex::check() (which forward-projects the commanded velocity to decide
 * whether continuing would cause a collision), this is a simple static proximity query: "how
 * close is the nearest obstacle right now". Used by level-1 evaluators/mitigators that need to
 * reason about proximity independently of the current motion (e.g. deciding whether it is safe
 * to resume, or in which direction to retreat).
 *
 * @param nav_state Current navigation state.
 * @param z_min Points below this height (robot frame) are ignored, e.g. the ground.
 * @param z_max Points above this height (robot frame) are ignored, e.g. above the robot.
 * @return The nearest perception's distance and bearing, or distance = +infinity if no point is
 * in range (see ObstacleProximity::perceived).
 */
ObstacleProximity compute_nearest_obstacle(
  easynav::NavState & nav_state,
  double z_min = -std::numeric_limits<double>::infinity(),
  double z_max = std::numeric_limits<double>::infinity());

}  // namespace easynav_diagnostic_recovery

#endif  // EASYNAV_DIAGNOSTIC_RECOVERY__OBSTACLEPROXIMITY_HPP_
