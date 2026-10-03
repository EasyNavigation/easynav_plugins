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
/// \brief Implementation of compute_nearest_obstacle().

#include <algorithm>
#include <cmath>
#include <limits>

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_diagnostic_recovery/ObstacleProximity.hpp"

namespace easynav_diagnostic_recovery
{

ObstacleProximity compute_nearest_obstacle(
  easynav::NavState & nav_state, double z_min, double z_max)
{
  ObstacleProximity result;

  const auto & perceptions = nav_state.get_by_type<easynav::PointPerception>();
  for (const auto & perception : perceptions) {
    result.perceived = result.perceived || perception->valid;
  }
  if (!result.perceived) {
    return result;
  }

  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();

  auto view = easynav::PointPerceptionsOpsView(perceptions);
  view.fuse(tf_info.robot_frame);
  const auto & cloud = view.as_points();

  double min_dist_sq = std::numeric_limits<double>::infinity();
  double nearest_x = 0.0;
  double nearest_y = 0.0;

  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    if (p.z < z_min || p.z > z_max) {continue;}

    const double d_sq = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
      nearest_x = p.x;
      nearest_y = p.y;
    }
  }

  if (std::isfinite(min_dist_sq)) {
    result.distance = std::sqrt(min_dist_sq);
    result.bearing = std::atan2(nearest_y, nearest_x);
  }

  return result;
}

double free_distance_along_x(
  easynav::NavState & nav_state, int direction, double robot_radius, double z_min, double z_max)
{
  const auto & perceptions = nav_state.get_by_type<easynav::PointPerception>();
  bool perceived = false;
  for (const auto & perception : perceptions) {
    perceived = perceived || perception->valid;
  }
  if (!perceived) {
    return 0.0;
  }

  auto view = easynav::PointPerceptionsOpsView(perceptions);
  view.fuse(easynav::RTTFBuffer::getInstance()->get_tf_info().robot_frame);
  const auto & cloud = view.as_points();

  double free = std::numeric_limits<double>::infinity();
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    if (p.z < z_min || p.z > z_max) {continue;}
    const double ahead = direction * static_cast<double>(p.x);  // Along the motion.
    const double lateral = std::abs(static_cast<double>(p.y));
    if (lateral >= robot_radius || ahead <= 0.0) {continue;}  // Outside the swept corridor.
    // The robot's disk touches the point once its center is this far along x from it.
    const double reach = std::sqrt(robot_radius * robot_radius - lateral * lateral);
    free = std::min(free, std::max(0.0, ahead - reach));
  }
  return free;
}

}  // namespace easynav_diagnostic_recovery
