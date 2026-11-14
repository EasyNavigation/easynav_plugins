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
/// \brief NavMap filter that keeps a merged/split copy of the NavMap for planning.

#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__FRACTALFILTER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__FRACTALFILTER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "easynav_common/types/NavState.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_patch.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"
#include "easynav_navmap_maps_manager/filters/rtin_lod.hpp"

namespace easynav
{
namespace navmap
{

/**
 * @class FractalFilter
 * @brief Simplified copy of "map.navmap" for planning: same-cost NavCels merged into large
 *        triangles, split back where the cost changes.
 *
 * Goes after the filters that write its `layer` (filters: [obstacles, inflation, fractal]).
 * "map.navmap" is not touched: the filters keep working on it at full resolution. The result is
 * left in `output_key` ("map.navmap.lod"), where A* reads it with its `map_key`. Only the costs
 * around the robot (`window_radius`) and a slice of the map (`sweep`) are read each cycle. See
 * RtinLod.
 */
class FractalFilter : public NavMapFilter
{
public:
  void on_initialize() override;
  void update(::easynav::NavState & nav_state) override;

private:
  /// The whole map to a receiver without a copy, else a patch with what changed since.
  void publish(const rclcpp::Time & now);

  const std::string input_key_{"map.navmap"};  ///< the maps manager's NavMap
  std::string output_key_{"map.navmap.lod"};
  std::string layer_{"inflated_obstacles"};
  double rebuild_period_{0.5};
  double window_radius_{8.0};      ///< m around the robot read every cycle
  int sweep_{100000};              ///< NavCels of the whole map read per cycle, in turns
  double publish_period_{2.0};     ///< s between two publications for RViz (map or patch)
  double full_period_{0.0};        ///< s between two whole maps (resync); 0: only when needed
  double log_period_{10.0};        ///< s between two logs of the map's statistics (0: every one)
  RtinLod::Params params_;

  RtinLod lod_;
  std::uint64_t source_geometry_{0};
  std::size_t source_navcels_{0};
  bool dirty_{false};
  rclcpp::Time last_extract_;
  bool extracted_once_{false};
  std::shared_ptr<::navmap::NavMap> out_;
  rclcpp::Time last_publish_;
  rclcpp::Time last_full_;
  bool sent_once_{false};
  bool need_full_{true};
  std::size_t last_subscribers_{0};
  std::uint64_t last_sent_id_{0};   ///< geometry of the last map or patch sent

  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr map_pub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapPatch>::SharedPtr patch_pub_;
};

}  // namespace navmap
}  // namespace easynav

#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__FILTERS__FRACTALFILTER_HPP_
