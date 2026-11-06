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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__BIGMAPMAPSMANAGER_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__BIGMAPMAPSMANAGER_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <mutex>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"
#include "pluginlib/class_loader.hpp"
#include "easynav_bigmap_maps_manager/dynamic_layer.hpp"
#include "easynav_bigmap_maps_manager/obstacle_memory.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "easynav_bigmap_maps_manager/known_map.hpp"
#include "easynav_bigmap_maps_manager/profiler.hpp"
#include "easynav_bigmap_maps_manager/update_watchdog.hpp"
#include "easynav_bigmap_maps_manager/navmap_dem.hpp"
#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/surface_lod.hpp"

namespace easynav
{
namespace bigmap
{

/**
 * @class BigMapMapsManager
 * @brief EasyNav MapsManagerBase plugin ("bigmap"): a NavMap of a large terrain or building whose
 * triangle size grows with the distance to the robot.
 *
 * WHAT IT OWNS (parameters under `maps_manager_node.bigmap`, documented in bigmap_bringup/config):
 *   terrain, multilevel, lengths, lod, traversed, obstacles (detector, removal, memory, tint),
 *   dynamic_map, output. It publishes `<node>/bigmap/map` (NavMap), `.../markers`, `.../dynamic_map`
 *   and sets "map.navmap" (and "map.dynamic") in the NavState.
 *
 * WHAT IT BORROWS: the NavMapFilter plugins of easynav_navmap_maps_manager, loaded through the
 *   `filters` list (see below), and `system_node.robot_geometry` (read by the inflation filter).
 *
 * Every update it takes the robot position and re-meshes when the robot moved `lengths.remesh_step`:
 *  - grid engine (outdoors, any single-floor NavMap): RTIN over a height grid (LodMesher);
 *  - hierarchy engine (`multilevel.enabled`): clustering of the NavMap's own triangles
 *    (SurfaceLodMesher), for floors over each other and ramps.
 * The same rule decides split and merge in both (lod_policy.hpp). The NavMap in the NavState has:
 *  - "occupancy" (uint8): the static map, LETHAL_OBSTACLE on NavCels steeper than `max_slope_deg`;
 *  - "obstacles": occupancy plus what the sensors detect (who writes that: `obstacles.detector`);
 *  - "inflated_obstacles": written by the inflation filter if it is in `filters`;
 *  - "obstacle_hits": how many times an obstacle was detected on each NavCel (`obstacles.memory`);
 *  - "traversed", "elevation", "lod_size"; vertex colours if the map has imagery.
 *
 * OBSTACLES, in the order they are applied (their dependencies are checked at start, see
 * pipeline_check.hpp, and the result is logged):
 *  1. detector `bigmap_dynamic` (default): the DynamicLayer finds sensor points above a NavCel
 *     (heights relative to the robot: independent of the localization's z). Detector
 *     `navmap_filter`: the ObstacleFilter plugin does (it measures through the map -> robot TF
 *     and needs a good z); it must be in `filters`, under an instance name other than `obstacles`.
 *  2. `obstacles.memory` (bigmap_dynamic only): a detection that lasts `min_time` over a NavCel is
 *     an obstacle and one hit; `max_hits` hits fix it for good.
 *  3. `obstacles.removal` (bigmap_dynamic: it is what puts the detections in "obstacles"):
 *     those NavCels, and the ones within `lengths.radii.obstacle_clearance`, are removed from the
 *     published NavMap and set LETHAL for the planner.
 *  4. `filters`: each listed filter runs, and ONLY the listed ones (the list is their on/off
 *     switch). The inflation filter spreads the cost around LETHAL NavCels; A* reads
 *     "inflated_obstacles" if it exists, else "obstacles".
 *  5. `obstacles.tint`: paints the "obstacles" NavCels in RViz (display only).
 * The dynamic map (`dynamic_map`) raises the NavCels where the sensors saw something and publishes
 * them; it is analysed if `dynamic_map.enabled` or `obstacles.memory.enabled`.
 *
 * RELIABILITY AND LATENCY:
 *  - `lod.hole_error` / `lod.observed`: the holes of the map (its obstacles) keep their shape away
 *    from the robot, by an error on their border and by keeping at full resolution the borders the
 *    robot has seen from near (so a far plan does not go through an obstacle it has already seen,
 *    and the robot does not come back and forth between two meshes).
 *  - `recovery`: with a goal and no path for some cycles, the full-resolution circle is doubled
 *    (up to a cap) and the map re-meshed, until the goal is reached or dropped.
 *  - `lod.async_remesh`: the re-mesh runs in a worker thread and the cycle that asked for it does
 *    not wait: the previous map is used until the new one is ready.
 *
 * Parameter names used before the reorganisation are still read, with a warning (param_any).
 */
class BigMapMapsManager : public easynav::MapsManagerBase
{
public:
  BigMapMapsManager() = default;
  void on_initialize() override;

protected:
  void update(::easynav::NavState & nav_state) override;

private:
  bool robot_position(
    const ::easynav::NavState & nav_state, double & x, double & y, double & z,
    std::array<double, 4> & q_xyzw);
  // Re-meshing: prepare (driven-over / observed ground, in this thread), make (extraction and
  // NavMap: in this thread or in the worker) and commit (in this thread)
  struct Remesh
  {
    bigmap_maps_manager::LodMesh mesh;
    ::navmap::NavMap navmap;
    double x{0.0}, y{0.0}, z{0.0};
    std::chrono::steady_clock::time_point t0;
  };
  void prepare_remesh(double x, double y, double z);
  Remesh make_remesh(
    double x, double y, double z, const bigmap_maps_manager::LodParams & params,
    bigmap_maps_manager::Profiler * prof) const;
  void commit_remesh(Remesh && r);
  void mark_observed(double x, double y, double z);   // the obstacle borders seen from near
  void mark_trail(double x, double y, double z);
  void check_recovery(const ::easynav::NavState & nav_state);
  bool is_traversed(const bigmap_maps_manager::LodMesh & mesh, std::size_t t) const;
  ::navmap::NavMap build_navmap(const bigmap_maps_manager::LodMesh & mesh) const;

  // Obstacle removal: NavCels with sensor obstacles (and those within a radius) are cut out of the
  // published NavMap and blocked (LETHAL) for the planner
  void restore_removed(::easynav::NavState & nav_state);   // our marks -> base, before the filter
  // Dynamic map: NavCels raised to what the sensors measure (see DynamicLayer)
  void update_dynamic_map(
    ::easynav::NavState & nav_state, double x, double y, double z,
    const std::array<double, 4> & q_xyzw);
  navmap_ros_interfaces::msg::NavMap make_dynamic_msg(
    const std::vector<bigmap_maps_manager::RaisedTile> & tiles, const rclcpp::Time & stamp) const;
  // Obstacle memory (layer `obstacle_hits`): how many times an obstacle has been detected on each
  // NavCel, from the hits of ObstacleMemory (which outlive the re-meshes)
  bool memory_active() const;   // enabled now (runtime parameter obstacles.memory.enabled)
  void write_hits(::navmap::NavMap & nm) const;   // the memory's counts onto the NavCels
  void reset_hits(::navmap::NavMap & nm) const;   // the layer without the memory (steep only)
  void fixed_seeds(std::vector<std::uint32_t> & seeds) const;   // NavCels of fixed obstacles
  // nothing analyses the sensors any more: what they blocked is released
  void release_sensor_obstacles(::easynav::NavState & nav_state);
  // every NavCel of the current mesh inside a memory cell (its square metre, at its height)
  void triangles_of(
    const bigmap_maps_manager::ObstacleMemory::Cell & cell,
    std::vector<std::uint32_t> & out) const;
  void apply_obstacle_removal(::easynav::NavState & nav_state);  // seeds: the ObstaclesFilter's
  void apply_removal(::easynav::NavState & nav_state, const std::vector<std::uint32_t> & seeds);
  navmap_ros_interfaces::msg::NavMap make_map_msg(
    const std::vector<char> & removed, const ::navmap::NavMap & layers_from,
    const rclcpp::Time & stamp, bool tint = true) const;
  void publish_map(const std::vector<char> & removed, const ::navmap::NavMap & layers_from);

  // Saving the map as it is (`all`, or only the `traversed` NavCels), asked for through services
  // and done by update(), the thread that owns the mesh
  struct SaveRequest
  {
    bool traversed_only{false};
    std::string path;  // decided when asked, written by the next update()
  };
  void process_save(::easynav::NavState & nav_state);
  void create_services(const std::string & base);   // save_map, save_traversed, clear_obstacles
  std::mutex save_mutex_;
  std::shared_ptr<SaveRequest> pending_save_;
  std::string save_directory_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_all_srv_, save_traversed_srv_;
  /// <name>/clear_obstacles: forgets every obstacle of the memory (queued, done by update())
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_obstacles_srv_;
  std::atomic<bool> clear_obstacles_requested_{false};

  std::unique_ptr<bigmap_maps_manager::LodMesher> mesher_;
  // Multi-level mode (`multilevel.enabled`): the NavMap's own triangles are simplified by
  // clustering, instead of a height grid with RTIN (floors over each other, ramps, any building)
  std::unique_ptr<bigmap_maps_manager::SurfaceLodMesher> surface_;
  bool has_known() const;  // driven-over ground exists (either mode)
  // What RViz shows (runtime parameter `view_max_z`): NavCels above this height are left out of the
  // published map, so a click lands on a lower floor. The planner's map is not touched.
  double view_max_z_{1e9};
  // Timing of the sections of update(), logged every profile.period s when profile.enabled
  bigmap_maps_manager::Profiler profiler_;
  bool profile_enabled_{false};
  // Says (on the wall clock, from the non-real-time executor) when update() stops being called:
  // EasyNav calls it when the node's ROS clock says it is due, so a clock that never moves is
  // otherwise total silence (the throttled logs are throttled on that same clock)
  bigmap_maps_manager::UpdateWatchdog watchdog_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  void check_watchdog();
  bigmap_maps_manager::Profiler * prof() {return profile_enabled_ ? &profiler_ : nullptr;}
  bigmap_maps_manager::MapOrigin origin_;  // where the map's (0, 0, 0) is, if anywhere
  std::unique_ptr<bigmap_maps_manager::KnownMap> known_;  // ground the robot has driven over
  // obstacle borders (holes of the map) the robot has seen at full resolution (grid engine)
  std::unique_ptr<bigmap_maps_manager::KnownMap> observed_;
  bool observed_enabled_{true};
  double observed_mark_radius_{10.0};
  // no path to the goal for recovery_cycles_ cycles: full_res doubled up to recovery_max_factor_
  bool recovery_enabled_{true};
  int recovery_cycles_{3};
  double recovery_max_factor_{4.0};
  int no_path_cycles_{0};
  double base_high_res_{10.0}, base_low_res_{200.0};
  bool force_remesh_{false};
  bool async_remesh_{false};
  std::vector<std::array<double, 3>> pending_marks_;   // poses met while the worker meshes
  bool trail_enabled_{true};
  double trail_mark_radius_{1.0};
  bool trail_persistent_{false};  // mode `persistent`; else `circles`
  double trail_keep_radius_{40.0};  // the larger circle (mode `circles`)
  bigmap_maps_manager::LodParams params_;
  double hysteresis_{2.0};
  double max_slope_deg_{35.0};
  bool publish_markers_{true};
  std::string map_frame_;

  bool dynamic_enabled_{true};
  bigmap_maps_manager::DynamicParams dyn_;
  bigmap_maps_manager::DynamicLayer dyn_layer_;
  std::unique_ptr<bigmap_maps_manager::TriangleLocator> locator_;  // of mesh_
  bool dyn_dirty_{false};
  double last_dyn_publish_{0.0};
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr dynamic_pub_;

  // Memory of where obstacles were detected and how often (obstacles.memory.*)
  std::unique_ptr<bigmap_maps_manager::ObstacleMemory> memory_;
  double memory_min_time_{5.0};
  int memory_max_hits_{3};
  bool memory_was_on_{false};

  bool obstacles_from_filter_{false};  // else: from the dynamic map
  bool removal_enabled_{true};
  double removal_radius_{1.0};
  bool tint_enabled_{true};            // obstacle NavCels are painted over the imagery
  std::array<float, 3> tint_rgb_{255.f, 40.f, 40.f};
  float tint_strength_{0.65f};
  bigmap_maps_manager::LodMesh mesh_;          // the mesh behind navmap_ (same triangle order)
  std::vector<float> centroid_x_, centroid_y_, centroid_z_;
  // the centroids by cells of centroid_cell_ (>= the clearance radius), for the ring of removal
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> centroid_grid_;
  double centroid_cell_{1.0};
  std::vector<std::uint32_t> removed_;       // NavCels currently removed (sorted)
  std::vector<char> pending_flags_;          // removed NavCels of the map published at the end
  bool publish_pending_{false};

  std::optional<std::pair<double, double>> last_meshed_;
  double last_meshed_z_{0.0};
  ::navmap::NavMap navmap_;
  bool map_changed_{false};

  std::unique_ptr<pluginlib::ClassLoader<::easynav::navmap::NavMapFilter>> filters_loader_;
  std::vector<std::shared_ptr<::easynav::navmap::NavMapFilter>> filters_;
  float map_resolution_{1.0f};
  std::size_t last_subscribers_{0};

  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr layer_updates_pub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr navmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // the re-mesh running in the worker (lod.async_remesh). Last member: destroyed first, its
  // destructor waits for the worker, which reads the members above
  std::future<Remesh> remesh_job_;
};

}  // namespace bigmap
}  // namespace easynav

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__BIGMAPMAPSMANAGER_HPP_
