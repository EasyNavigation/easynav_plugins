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
 * search starts at the coarsest level, evaluating every free cell with a fixed angular
 * resolution. Then, only the cells that are promising are explored at the next finer level,
 * down to the original map. The metric is the fraction of perception points that fall on an
 * obstacle of the map.
 *
 * Once built, it is immutable, so \c get_matches can safely run in another thread.
 */
class MapMatcher
{
public:
  /**
   * @brief Build the resolution pyramid.
   *
   * @param map Base map.
   * @param num_levels Number of levels of the pyramid, including the original map.
   * @param angle_step Angular resolution of the search (rad).
   */
  MapMatcher(const easynav::Costmap2D & map, int num_levels = 4, double angle_step = M_PI / 8.0);

  /**
   * @brief Get the candidate poses.
   *
   * @param points Perception points in the robot frame.
   * @param min_weight Candidates below this weight are discarded on every level.
   * @return Candidates in the finest level, sorted from best to worst.
   */
  std::vector<TransformWeighted> get_matches(
    const std::vector<tf2::Vector3> & points, double min_weight) const;

  /// \brief Number of levels of the pyramid.
  int num_levels() const {return static_cast<int>(levels_.size());}

  /// \brief Map of a level of the pyramid (0 is the original resolution).
  const easynav::Costmap2D & level(int idx) const {return *levels_[idx];}

protected:
  static std::shared_ptr<easynav::Costmap2D> half_scale(const easynav::Costmap2D & in);

  double match(
    int level, const std::vector<tf2::Vector3> & points, const tf2::Transform & pose) const;

  std::vector<std::shared_ptr<easynav::Costmap2D>> levels_;
  double angle_step_;
};

}  // namespace mhamcl
}  // namespace easynav

#endif  // EASYNAV_MHAMCL_LOCALIZER__MAPMATCHER_HPP_
