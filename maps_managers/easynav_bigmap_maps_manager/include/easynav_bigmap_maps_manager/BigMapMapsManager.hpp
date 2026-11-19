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

/**
 * @file BigMapMapsManager.hpp
 * @brief A large NavMap, fine around the robot and coarse far from it.
 *
 * Loads a NavMap, cuts it into tiles and simplifies them at several levels of detail (TiledLod,
 * once). Every cycle it puts together, around the robot, the coarsest tile each place allows;
 * near the robot it refines the borders of the map and the obstacles the sensors see (which it
 * remembers until the robot is forget_distance away from them). The result is "map.navmap";
 * the filters (e.g. inflation) run on it as in the NavMap maps manager.
 */

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__BIGMAPMAPSMANAGER_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__BIGMAPMAPSMANAGER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "easynav_bigmap_maps_manager/sensed_obstacles.hpp"
#include "easynav_bigmap_maps_manager/tiled_lod.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easynav
{
namespace bigmap
{

class BigMapMapsManager : public easynav::MapsManagerBase
{
public:
  BigMapMapsManager();
  ~BigMapMapsManager() override;

  void on_initialize() override;
  void update(::easynav::NavState & nav_state) override;

  /// The level-of-detail engine (for tests)
  const TiledLod & lod() const {return lod_;}
  /// The obstacles remembered (for tests)
  const SensedObstacles & sensed() const {return sensed_;}

private:
  /// Robot position (map frame) and heading; false if unknown
  bool robot_pose(const ::easynav::NavState & nav_state, Eigen::Vector3d & p, double & yaw) const;
  /// The obstacles in this cycle's perceptions, into sensed_. True if any is new.
  bool sense(const ::easynav::NavState & nav_state, const Eigen::Vector3d & robot, double yaw);
  /// The map around the robot, into map_
  void rebuild(const std::vector<std::uint32_t> & tiles, const Eigen::Vector3d & robot);
  /// The map (as the filters left it) and the markers of its levels
  void publish(const ::navmap::NavMap & nm);

  TiledLod lod_;
  SensedObstacles sensed_;
  ::navmap::NavMap map_;
  bool has_map_{false};
  bool map_changed_{false};
  std::vector<std::uint32_t> tiles_;     ///< those in map_
  Eigen::Vector2d built_at_{0.0, 0.0};   ///< robot position when map_ was made
  double built_z_{0.0};
  double last_build_{-1e18};             ///< s
  double last_publish_{-1e18};           ///< s
  bool publish_pending_{false};
  bool sensed_changed_{false};
  /// For measurements, in the NavState as "map.bigmap_stats" every cycle: rebuilds so far, and
  /// the last rebuild's select, assemble, detail (ms) and NavCels; this cycle's sensing and
  /// filters (ms)
  std::vector<double> stats_ = std::vector<double>(7, 0.0);
  double select_ms_{0.0};

  // Parameters
  double forget_distance_{60.0};   ///< m: the sensed obstacles farther than this are forgotten
  double margin_{0.0};             ///< m: fine around obstacles (the filters' inflation radius)
  float known_{0.5f};              ///< m: a point this close to no ground is the map's obstacle
  double min_height_{0.1};         ///< m above the map's surface: lower points are the ground
  double min_height_per_m_{0.02};  ///< m: min_height grows this much per metre from the robot
  double max_height_{2.0};         ///< m above it: higher points do not touch the robot
  double sensor_range_{15.0};      ///< m: points farther from the robot are not used
  double rebuild_period_{0.5};     ///< s: the map is made again at most this often
  double publish_period_{1.0};     ///< s: and published at most this often

  std::unique_ptr<pluginlib::ClassLoader<easynav::navmap::NavMapFilter>> filters_loader_;
  std::vector<std::shared_ptr<easynav::navmap::NavMapFilter>> filters_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr map_pub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr layer_updates_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  /// stats_ every cycle, with the time (s) first: what a cycle costs, for measurements
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_pub_;
  std::size_t last_subscribers_{0};
};

}  // namespace bigmap
}  // namespace easynav

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__BIGMAPMAPSMANAGER_HPP_
