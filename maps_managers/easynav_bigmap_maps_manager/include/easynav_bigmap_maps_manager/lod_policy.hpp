// Copyright 2026 José Miguel Guerrero
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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__LOD_POLICY_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__LOD_POLICY_HPP_

#include <algorithm>
#include <cmath>

namespace bigmap_maps_manager
{

/// Distance-driven LOD policy (all lengths in metres).
///
/// A triangle is split when `leg > min_size` and (`error > tolerance(d)` or `leg > size_cap(d)`),
/// where d is the *effective* distance from the robot to the triangle's split vertex.
///
/// Crack-freeness: RTIN is T-junction free iff "split(child vertex) => split(parent vertex)".
/// That holds if tolerance() and size_cap() are non-decreasing in d AND d is monotone along the
/// vertex hierarchy. The raw distance is not (a parent can be farther than its child), so we use
/// d_eff = max(0, dist - kReach * hypotenuse): the child's bounding disc is nested in the
/// parent's one for kReach >= (1 + sqrt(2)) / 2 ~= 1.207, hence d_eff(parent) <= d_eff(child).
struct LodParams
{
  double high_res_radius{10.0};    ///< full resolution (min_size leaves) while d_eff <= this
  double low_res_distance{200.0};  ///< from this d_eff on, leaves may be as large as max_size
  double min_size{1.0};            ///< smallest leaf leg (clamped by the DEM resolution)
  double max_size{32.0};           ///< largest leaf leg anywhere
  double base_error{0.02};         ///< height tolerance (m) inside the high-res disc
  double error_growth{0.05};       ///< extra tolerance per metre beyond high_res_radius
  /// Ground the robot has driven over (KnownMap) keeps full resolution up to this distance from
  /// the traversed nodes, wherever the robot is now.
  double known_margin{1.0};
  /// Beyond known_margin, resolution fades `known_falloff` times faster with the distance to the
  /// traversed ground than it does with the distance to the robot (1: the same).
  double known_falloff{1.0};
  /// Obstacle borders the robot has seen from near (observed) keep full resolution up to this
  /// distance, with the same falloff beyond. Negative: known_margin.
  double observed_margin{-1.0};
};

constexpr double kReach = 1.25;

inline double effective_distance(double dist_to_vertex, double hypotenuse)
{
  return std::max(0.0, dist_to_vertex - kReach * hypotenuse);
}

inline double tolerance(const LodParams & p, double d_eff)
{
  return p.base_error + p.error_growth * std::max(0.0, d_eff - p.high_res_radius);
}

/// Largest leaf leg allowed at effective distance d: min_size up to high_res_radius, max_size from
/// low_res_distance on, and in between geometric (log-linear): the cell size is multiplied by the
/// same factor for every metre of distance, so detail fades smoothly. Non-decreasing in d.
inline double size_cap(const LodParams & p, double d_eff)
{
  if (d_eff <= p.high_res_radius) {
    return p.min_size;
  }
  if (d_eff >= p.low_res_distance || p.low_res_distance <= p.high_res_radius) {
    return p.max_size;
  }
  const double t = (d_eff - p.high_res_radius) / (p.low_res_distance - p.high_res_radius);
  return p.min_size * std::pow(p.max_size / p.min_size, t);
}

/// Effective distance for a vertex at `dist_to_known` from ground the robot has driven over:
/// full resolution (high_res_radius) up to known_margin, then the policy's distance grows
/// known_falloff times faster. `dist_to_known - kReach * hypotenuse` is monotone along the vertex
/// hierarchy like the robot's own formula; a constant shift (known_margin), a max with 0 and a
/// positive scale keep that, so the minimum of the two stays crack-free.
/// (The margin is subtracted before the scale: before, it was divided by known_falloff, and with
/// the node's falloff of ~40 a 10 m margin was a 0.25 m strip.)
inline double known_effective_distance(const LodParams & p, double dist_to_known, double hypotenuse)
{
  return p.high_res_radius +
         p.known_falloff * std::max(0.0, dist_to_known - p.known_margin - kReach * hypotenuse);
}

/// The policy for the observed ground: the driven-over one's with observed_margin as its margin
inline LodParams observed_params(const LodParams & p)
{
  LodParams o = p;
  if (p.observed_margin >= 0.0) {o.known_margin = p.observed_margin;}
  return o;
}

inline bool should_split_at(const LodParams & p, double error, double leg, double d_eff)
{
  if (leg <= p.min_size) {
    return false;
  }
  return error > tolerance(p, d_eff) || leg > size_cap(p, d_eff);
}

inline bool should_split(
  const LodParams & p, double error, double leg, double dist_to_vertex, double hypotenuse)
{
  return should_split_at(p, error, leg, effective_distance(dist_to_vertex, hypotenuse));
}

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__LOD_POLICY_HPP_
