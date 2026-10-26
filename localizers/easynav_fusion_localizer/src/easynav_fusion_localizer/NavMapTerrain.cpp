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

#include "easynav_fusion_localizer/NavMapTerrain.hpp"

#include <cmath>
#include <limits>

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Vector3.hpp"

namespace easynav
{

namespace
{

/// Attitude with the surface's normal as z axis and the given heading.
tf2::Quaternion frame_from_normal_and_yaw(const tf2::Vector3 & n, double yaw)
{
  const tf2::Vector3 heading(std::cos(yaw), std::sin(yaw), 0.0);
  tf2::Vector3 x_axis = heading - n * heading.dot(n);
  tf2::Quaternion flat;
  flat.setRPY(0.0, 0.0, yaw);
  if (x_axis.length() < 1e-9) {return flat;}
  x_axis.normalize();
  tf2::Vector3 y_axis = n.cross(x_axis);
  if (y_axis.length() < 1e-9) {return flat;}
  y_axis.normalize();
  const tf2::Matrix3x3 R(
    x_axis.x(), y_axis.x(), n.x(),
    x_axis.y(), y_axis.y(), n.y(),
    x_axis.z(), y_axis.z(), n.z());
  tf2::Quaternion q;
  R.getRotation(q);
  return q;
}

}  // namespace

void NavMapTerrain::set_height_hint(double z)
{
  height_hint_ = z;
  reset();
}

void NavMapTerrain::reset()
{
  last_cid_.reset();
  last_surface_.reset();
  last_z_.reset();
}

std::optional<NavMapTerrain::Result> NavMapTerrain::project(
  const ::navmap::NavMap & map, double x, double y, double yaw,
  const std::optional<tf2::Quaternion> & imu)
{
  std::size_t surface = 0;
  ::navmap::NavCelId cid = std::numeric_limits<::navmap::NavCelId>::max();
  Eigen::Vector3f bary, hit;
  bool found = false;

  if (!map.navcels.empty()) {
    // Following the surface it is on, within a step of its last height.
    if (last_cid_ && last_z_ && *last_cid_ < map.navcels.size()) {
      ::navmap::NavMap::LocateOpts opts;
      opts.hint_cid = last_cid_;
      opts.hint_surface = last_surface_;
      opts.height_eps = 0.5f;
      found = map.locate_navcel(
        Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(*last_z_)),
        surface, cid, bary, &hit, opts);
    }
    // Otherwise the surface nearest to the last (or hinted) height.
    if (!found) {
      ::navmap::NavMap::LocateOpts opts;
      opts.height_eps = std::numeric_limits<float>::infinity();
      const double z0 = last_z_.value_or(height_hint_);
      found = map.locate_navcel(
        Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z0)),
        surface, cid, bary, &hit, opts);
    }
  }

  Result result;
  double roll = 0.0, pitch = 0.0, imu_yaw = 0.0;
  if (imu) {
    tf2::Matrix3x3(*imu).getRPY(roll, pitch, imu_yaw);
  }
  result.orientation.setRPY(roll, pitch, yaw);

  if (found) {
    last_cid_ = cid;
    last_surface_ = surface;
    last_z_ = hit.z();
    result.on_map = true;
    result.z = hit.z();
    if (!imu) {
      tf2::Vector3 n(map.navcels[cid].normal.x(), map.navcels[cid].normal.y(),
        map.navcels[cid].normal.z());
      if (n.length() > 1e-9) {
        n.normalize();
        if (n.z() < 0.0) {n = -n;}
        result.orientation = frame_from_normal_and_yaw(n, yaw);
      }
    }
    return result;
  }
  if (!last_z_) {
    return std::nullopt;
  }
  // Off the map: the last known height.
  result.z = *last_z_;
  return result;
}

}  // namespace easynav
