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

#ifndef EASYNAV_COSTMAP_LOCALIZER__LOCALIZATIONQUALITY_HPP_
#define EASYNAV_COSTMAP_LOCALIZER__LOCALIZATIONQUALITY_HPP_

#include <cstddef>

namespace easynav
{

/// @brief How well the particles explain what the sensors see, in absolute terms.
/**
 * Each correction gives two scores in [0, 1]: the best particle's and the mean of all
 * particles' (the fraction of points that land on obstacles of the map). The covariance only
 * tells how spread the particles are; a lost filter can be compact and wrong, and these scores
 * drop when it is.
 *
 *  - match_score(): the best particle's score, smoothed (fast average).
 *  - quality_ratio(): fast / slow average of the mean score (Augmented MCL, Thrun et al.,
 *    Probabilistic Robotics, 8.3.5). Near 1 while the scores keep their usual level; well
 *    below 1 right after they drop, e.g. when the robot is kidnapped.
 *  - injection_probability(): max(0, 1 - fast / slow), the share of particles to replace with
 *    random ones.
 *
 * Each average starts as the plain mean of the scores so far (until its alpha is smaller), so
 * the first, still unconverged corrections do not set the level for long.
 */
class LocalizationQuality
{
public:
  /// @param alpha_slow, alpha_fast Smoothing per correction, 0 < alpha_slow < alpha_fast <= 1.
  ///   Either 0 disables the slow/fast averages (ratio 1, no injection).
  explicit LocalizationQuality(double alpha_slow = 0.001, double alpha_fast = 0.1);

  /// @brief Takes the scores of a correction.
  /// @param mean_score Mean score of all particles.
  /// @param best_score Best particle's score.
  void add(double mean_score, double best_score);

  /// @brief Whether any correction was taken.
  bool has_data() const {return has_data_;}
  double match_score() const {return best_;}
  double slow() const {return slow_;}
  double fast() const {return fast_;}
  double quality_ratio() const;
  double injection_probability() const;

  /// @brief Forgets the scores (e.g. after a new initial pose).
  void reset();

private:
  bool averaging() const {return alpha_slow_ > 0.0 && alpha_fast_ > 0.0;}

  double alpha_slow_;
  double alpha_fast_;
  bool has_data_ {false};
  /// Corrections taken: the first ones use their plain mean (warm-up).
  std::size_t count_ {0};
  double best_ {0.0};
  double slow_ {0.0};
  double fast_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__LOCALIZATIONQUALITY_HPP_
