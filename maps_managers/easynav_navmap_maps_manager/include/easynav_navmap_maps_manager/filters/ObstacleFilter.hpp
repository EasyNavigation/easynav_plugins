// Copyright 2025 Intelligent Robotics Lab
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


#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

/**
 * @class ObstacleFilter
 * @brief Marks as obstacles the NavCels under the points that rise above the NavMap surface.
 *
 * Points within max_range of the robot and below max_height (robot frame) are grouped in
 * 0.3 m columns; a column is an obstacle when its highest point is more than min_height (plus
 * min_height_per_meter for each meter from the robot) above the NavMap surface under it. So a 2D
 * laser sees obstacles too, and the ground or a ramp, being on the surface, is not one.
 *
 * Incremental: the layer is rebuilt from the static map only when the map changes; each cycle
 * writes just the cells that become obstacles or stop being one (recorded as changed).
 */
class ObstacleFilter : public NavMapFilter
{
public:
  ObstacleFilter();

  void on_initialize() override;
  void update(::easynav::NavState & nav_state) override;

  bool is_adding_layer() override {return true;}
  std::string get_layer_name() override {return "obstacles";}

private:
  // Memory mode: evidence accumulated in log-odds, cleared by rays, faded by decay
  void update_memory(
    ::navmap::NavMap & nm, const Eigen::Vector3f & robot, bool robot_known,
    const std::vector<::navmap::NavCelId> & hits,
    const std::vector<std::pair<::navmap::NavCelId, Eigen::Vector3f>> & ends);

  bool memory_ {false};        ///< Remember obstacles (log-odds) instead of this cycle only
  bool clearing_ {true};       ///< Memory: rays from the robot clear the cells they cross
  double decay_time_ {0.0};    ///< Memory: s for a certain cell to fade to unknown; 0: never
  double p_hit_ {0.7};         ///< Memory: occupancy probability of a hit
  double p_miss_ {0.4};        ///< Memory: occupancy probability of a cell a ray crosses
  std::vector<::navmap::NavCelId> active_;  // cells with evidence (non-zero log-odds)
  std::vector<uint8_t> in_active_, flag_;
  std::vector<std::uint32_t> stamp_;
  std::uint32_t cycle_ {0};
  double last_time_ {-1.0}, decay_acc_ {0.0};

  // Incremental state, bound to the layer it was built on
  std::weak_ptr<::navmap::LayerViewBase> last_layer_;  // replaced or gone: rebuild
  std::uint64_t last_occ_hash_{0};
  std::vector<uint8_t> base_;                 // static value of each cell
  std::vector<::navmap::NavCelId> marked_;    // obstacle cells of the last cycle
  std::vector<uint8_t> seen_;                 // 1: in marked_, 2: marked this cycle
  double max_range_ {10.0};   ///< Points farther than this from the robot (x or y) are ignored.
  double min_height_ {0.1};   ///< Above the NavMap surface: lower points are the ground.
  double max_height_ {std::numeric_limits<double>::quiet_NaN()};  ///< Robot frame; NaN: no limit.
  double downsample_resolution_ {0.3};  ///< Voxel (m) the points are reduced to first; <= 0: off.
  double min_height_per_meter_ {0.0};   ///< min_height grows this much per meter from the robot.
};

}  // namespace navmap
}  // namespace easynav
#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__OBSTACLEFILTER_HPP_
