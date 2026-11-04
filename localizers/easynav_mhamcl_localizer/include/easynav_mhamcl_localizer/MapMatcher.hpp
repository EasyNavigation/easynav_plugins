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
/// \brief Declaration of MapMatcher, the cascade brute-force map matching of MH-AMCL.

#ifndef EASYNAV_MHAMCL_LOCALIZER__MAPMATCHER_HPP_
#define EASYNAV_MHAMCL_LOCALIZER__MAPMATCHER_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"

namespace easynav
{
namespace mhamcl
{

/// \brief A candidate pose in the map and how well the perception fits it.
struct TransformWeighted
{
  double weight {0.0};
  tf2::Transform transform;
};

/**
 * @brief Finds the poses of the map from which the last perception could have been obtained.
 *
 * The map is stored in a pyramid of resolutions, each level halving the previous one. The
 * search starts at the coarsest level, evaluating every free cell at angle_step; each finer
 * level refines the best candidates of the previous one (its 4 sub-cells, and half the angle
 * step around their heading), keeping the best `beam_width`, down to the original map.
 *
 * A pose scores the mean of exp(-d^2 / 2 sigma^2) over the perception points, d being the
 * distance from each point to the nearest obstacle of the map. Each level widens sigma with its
 * cell size and angle step, so a candidate a few centimeters or degrees off the true pose still
 * scores well and is not lost before it is refined. The final scores (level 0) are in [0, 1].
 *
 * Once built, it is immutable, so \c get_matches can safely run in another thread.
 */
class MapMatcher
{
public:
  /**
   * @brief Build the resolution pyramid and the distance field of the map.
   *
   * @param map Base map.
   * @param num_levels Number of levels of the pyramid, including the original map.
   * @param angle_step Angular resolution of the search at the coarsest level (rad).
   * @param sigma Sensor precision (m) at the original resolution.
   * @param beam_width Candidates kept on every level.
   */
  explicit MapMatcher(
    const easynav::Costmap2D & map, int num_levels = 4, double angle_step = M_PI / 8.0,
    double sigma = 0.05, std::size_t beam_width = 500);

  /**
   * @brief Get the candidate poses.
   *
   * @param points Perception points in the robot frame.
   * @param min_weight Candidates of the finest level at or below this weight are discarded.
   * @return Candidates in the finest level, sorted from best to worst.
   */
  std::vector<TransformWeighted> get_matches(
    const std::vector<tf2::Vector3> & points, double min_weight) const;

  /// \brief Number of levels of the pyramid.
  int num_levels() const {return static_cast<int>(levels_.size());}

  /// \brief Map of a level of the pyramid (0 is the original resolution).
  const easynav::Costmap2D & level(int idx) const {return *levels_[idx];}

  /// \brief Angle step of a level (rad): angle_step at the coarsest, halved on each finer one.
  double angle_step(int level) const;

  /// \brief Distance (m) from a point of the map frame to the nearest obstacle (inf: none, or
  /// out of the map).
  double distance_to_obstacle(double x, double y) const;

  /**
   * @brief Score of a pose at a level.
   * @param level Level of the pyramid (its tolerance).
   * @param points Perception points in the robot frame.
   * @param pose Pose of the robot in the map.
   * @param typical_range Typical range (m) of the points, to widen the tolerance with the angle
   *   step of the level.
   * @return Mean likelihood of the points, in [0, 1].
   */
  double match(
    int level, const std::vector<tf2::Vector3> & points, const tf2::Transform & pose,
    double typical_range) const;

protected:
  static std::shared_ptr<easynav::Costmap2D> half_scale(const easynav::Costmap2D & in);

  void build_distance_field(const easynav::Costmap2D & map);

  double sigma_of(int level, double typical_range) const;

  std::vector<std::shared_ptr<easynav::Costmap2D>> levels_;
  double angle_step_;
  double sigma_;
  std::size_t beam_width_;

  /// Distance (m) to the nearest obstacle per cell of the original map.
  std::vector<float> distance_;
};

}  // namespace mhamcl
}  // namespace easynav

#endif  // EASYNAV_MHAMCL_LOCALIZER__MAPMATCHER_HPP_
