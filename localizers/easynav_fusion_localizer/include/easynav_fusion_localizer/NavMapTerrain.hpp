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

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "navmap_core/NavMap.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace easynav
{

/// @brief Places a planar (x, y, yaw) estimate on the surface of a NavMap.
/**
 * The height is the surface's under (x, y), following the surface the robot
 * is on (a level of a multi-level map is kept while it continues). Roll and
 * pitch are the IMU's when given, the surface's otherwise; yaw is kept.
 */
class NavMapTerrain
{
public:
  /// @brief The pose placed on the surface.
  struct Result
  {
    double z {0.0};
    tf2::Quaternion orientation;
    /// @brief Whether (x, y) is on the map; if not, z is the last known one.
    bool on_map {false};
  };

  /// @brief Places (x, y, yaw) on `map`.
  /// @param map The NavMap.
  /// @param x, y Position in the map frame.
  /// @param yaw Heading in the map frame.
  /// @param imu Absolute IMU orientation, if any (roll and pitch are used).
  /// @return The height and attitude, or nothing if never on the map yet.
  std::optional<Result> project(
    const ::navmap::NavMap & map, double x, double y, double yaw,
    const std::optional<tf2::Quaternion> & imu);

  /// @brief Height to look around for the first time (e.g. an initial pose's).
  void set_height_hint(double z);

  /// @brief Forgets the surface it was following (e.g. after a new map).
  void reset();

private:
  std::optional<::navmap::NavCelId> last_cid_;
  std::optional<std::size_t> last_surface_;
  std::optional<double> last_z_;
  double height_hint_ {0.0};
};

}  // namespace easynav
