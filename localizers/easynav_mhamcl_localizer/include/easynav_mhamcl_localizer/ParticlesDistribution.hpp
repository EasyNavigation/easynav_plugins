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
/// \brief Declaration of ParticlesDistribution, a single pose hypothesis of MH-AMCL.

#ifndef EASYNAV_MHAMCL_LOCALIZER__PARTICLESDISTRIBUTION_HPP_
#define EASYNAV_MHAMCL_LOCALIZER__PARTICLESDISTRIBUTION_HPP_

#include <array>
#include <cstddef>
#include <random>
#include <vector>

#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"

namespace easynav
{
namespace mhamcl
{

/// \brief A single particle of a hypothesis.
struct Particle
{
  tf2::Transform pose{tf2::Transform::getIdentity()};   ///< Pose of the particle in the map frame.
  double weight {0.0};   ///< Normalized importance weight.
  float hits {0.0f};     ///< Fraction of the last perception that matches an obstacle in the map.
};

/// \brief A point of a perception, ready to be scored.
struct ScanPoint
{
  tf2::Vector3 point;      ///< Point in the robot frame.
  tf2::Vector3 direction;  ///< Unit vector (in the robot frame) along which the point was observed.
};

/// \brief Parameters of a hypothesis (a particle set).
struct ParticlesParams
{
  int max_particles {200};                  ///< Upper bound of the size of the particle set.
  int min_particles {30};                   ///< Lower bound of the size of the particle set.
  int particles_step {30};                  ///< Particles added/removed per reseed when adapting.
  double noise_translation {0.05};          ///< Motion noise proportional to the translation.
  double noise_rotation {0.05};             ///< Motion noise proportional to the rotation.
  double noise_translation_to_rotation {0.05};  ///< Rotation noise induced by the translation.
  double distance_perception_error {0.05};  ///< Sensor precision (sigma) in meters.
  double reseed_percentage_losers {0.8};    ///< Fraction of particles replaced on every reseed.
  double reseed_percentage_winners {0.03};  ///< Fraction of particles used as parents on reseed.
  double reseed_noise_xy {0.05};            ///< Std dev of the xy noise of a reseeded particle (m).
  double reseed_noise_yaw {0.05};           ///< Std dev of the yaw noise of a reseeded one (rad).
  double good_quality_threshold {0.6};      ///< Above it, the set is shrunk on reseed.
  double low_quality_threshold {0.25};      ///< Below it, the set is grown on reseed.
};

/**
 * @brief A set of particles representing one hypothesis about the robot pose.
 *
 * Each hypothesis is an independent particle filter (prediction, correction and reseed),
 * as described in "Multi-Hypothesis AMCL". Besides the weight of each particle, every
 * hypothesis has a *quality* in [0, 1]: the best fraction of the last perception that
 * matches an obstacle of the map, from any of its particles. Unlike the covariance, it
 * tells how well the hypothesis explains what the robot sees.
 *
 * This class does not depend on ROS nodes, so it can be used and tested standalone.
 */
class ParticlesDistribution
{
public:
  /// \brief Build an empty hypothesis. \c init must be called before use.
  explicit ParticlesDistribution(const ParticlesParams & params, unsigned int seed = 0);

  /**
   * @brief Sample the particles around a pose.
   *
   * @param pose Mean pose (map frame).
   * @param l00 Cholesky factor of the xy covariance: L = [l00 0; l10 l11].
   * @param l10 Cholesky factor of the xy covariance.
   * @param l11 Cholesky factor of the xy covariance.
   * @param std_yaw Standard deviation of the yaw.
   * @param quality Initial quality of the hypothesis.
   */
  void init(
    const tf2::Transform & pose, double l00, double l10, double l11, double std_yaw,
    double quality);

  /// \brief Same as above, with an isotropic xy deviation.
  void init(const tf2::Transform & pose, double std_xy, double std_yaw, double quality);

  /// \brief Move all the particles with the odometry increment (robot frame), plus noise.
  void predict(const tf2::Transform & delta);

  /**
   * @brief Update weights and quality with a perception.
   *
   * @param points Points in the robot frame (base footprint).
   * @param costmap Static map. Obstacles are the LETHAL_OBSTACLE cells.
   */
  void correct(const std::vector<ScanPoint> & points, const easynav::Costmap2D & costmap);

  /// \brief Replace the low-weight particles with noisy copies of the best ones.
  void reseed();

  /// \brief Absorb the best particles of \p other, keeping the size of this set.
  void merge(const ParticlesDistribution & other);

  /// \brief Weighted mean pose of the particles.
  tf2::Transform get_pose() const;

  /// \brief Covariance of (x, y, yaw) in the 6x6 row-major ROS layout.
  std::array<double, 36> get_covariance() const;

  /// \brief Quality of the hypothesis, in [0, 1].
  double get_quality() const {return quality_;}

  /// \brief Overwrite the quality (e.g. to give a fresh hypothesis a fair start).
  void set_quality(double quality) {quality_ = quality;}

  /// \brief The particles.
  const std::vector<Particle> & get_particles() const {return particles_;}

protected:
  /// \brief Distance from the expected obstacle position to the closest obstacle along a ray.
  double error_to_obstacle(
    const easynav::Costmap2D & costmap, double x, double y, double dx, double dy) const;

  /// \brief Scale weights to sum one (uniform if they are all zero).
  void normalize();

  ParticlesParams params_;
  std::mt19937 rng_;
  std::vector<Particle> particles_;
  double quality_ {0.0};
};

/// \brief Yaw of a transform.
double yaw_of(const tf2::Transform & tf);

/// \brief Wrap an angle to [-pi, pi].
double normalize_angle(double angle);

}  // namespace mhamcl
}  // namespace easynav

#endif  // EASYNAV_MHAMCL_LOCALIZER__PARTICLESDISTRIBUTION_HPP_
