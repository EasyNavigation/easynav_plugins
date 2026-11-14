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
/// \brief Gauss-Newton on SE(3) of a scan against the distance field, with a Gaussian prior
/// and absolute measurements (position, depth, attitude): one solver for the whole posterior.

#ifndef EASYNAV_ORBIT_LOCALIZER__MATCHER_HPP_
#define EASYNAV_ORBIT_LOCALIZER__MATCHER_HPP_

#include <array>
#include <vector>

#include "easynav_orbit_localizer/DistanceField.hpp"
#include "easynav_orbit_localizer/Lie.hpp"

namespace easynav
{
namespace orbit
{

/// @brief Which degrees of freedom the pose has: planar (x, y, yaw), planar on a terrain (the
/// same, z/roll/pitch set afterwards from the surface) or all six.
enum class Mode { Planar, PlanarTerrain, Free };

/// @brief A point of the body (lever) seen at p_map (GPS antenna, a beacon...).
struct PositionMeasurement
{
  Eigen::Vector3d p_map;
  Eigen::Vector3d lever {Eigen::Vector3d::Zero()};
  Eigen::Vector3d sigma {Eigen::Vector3d::Ones()};
  bool use_z {true};
};

/// @brief The body origin's height in the map (a depth gauge, an altimeter).
struct DepthMeasurement
{
  double z {0.0};
  double sigma {0.1};
};

/// @brief The body's rotation in the map (an IMU): roll and pitch always, yaw optionally.
struct AttitudeMeasurement
{
  Eigen::Matrix3d R_map_body;
  double sigma_roll_pitch {0.02};
  bool use_yaw {false};
  double sigma_yaw {0.05};
};

struct MatchOptions
{
  Mode mode {Mode::Planar};
  int max_iterations {12};
  double step_tolerance {1e-3};      // stop when |delta| (m, rad) is below it
  double sigma_point {0.1};          // m, the noise of a point's distance
  double cauchy_c {0.3};             // m, where a point stops counting (dynamic obstacles)
  double inlier_distance {0.2};      // m, a point this near the map counts for the score
  double damping {1e-3};             // Levenberg-Marquardt term, relative to diag(H)
  double fixed_variance {1e-8};      // of the fixed degrees of freedom
};

struct MatchResult
{
  Pose pose {Pose::Identity()};
  Mat6 cov {Mat6::Zero()};
  Mat6 info {Mat6::Zero()};
  double score {0.0};                // fraction of points within inlier_distance of the map
  double mean_distance {0.0};        // m, mean point distance (inliers and outliers)
  int iterations {0};
  bool converged {false};
  std::size_t n_points {0};
  Vec6 correction {Vec6::Zero()};    // Log(X0^-1 pose): how far the solution moved
};

/// @brief Aligns body points with the map from a starting pose, with the prior and the absolute
/// measurements as extra terms. Thread-safe: it holds no state.
class Matcher
{
public:
  Matcher(const DistanceField & field, const MatchOptions & options);

  /// @brief MAP pose: points against the map, plus the prior (info on the right perturbation
  /// of X0, none if null), positions, depths and attitudes.
  MatchResult match(
    const std::vector<Eigen::Vector3d> & body_points, const Pose & X0,
    const Mat6 * prior_info = nullptr,
    const std::vector<PositionMeasurement> & positions = {},
    const std::vector<DepthMeasurement> & depths = {},
    const std::vector<AttitudeMeasurement> & attitudes = {}) const;

  /// @brief Inlier fraction of the points at X, without optimizing.
  double score(const std::vector<Eigen::Vector3d> & body_points, const Pose & X) const;

  const MatchOptions & options() const {return options_;}
  const DistanceField & field() const {return field_;}

  /// @brief Indices of the tangent vector the mode leaves free.
  std::vector<int> freeDofs() const;

private:
  const DistanceField & field_;
  MatchOptions options_;
};

}  // namespace orbit
}  // namespace easynav

#endif  // EASYNAV_ORBIT_LOCALIZER__MATCHER_HPP_
