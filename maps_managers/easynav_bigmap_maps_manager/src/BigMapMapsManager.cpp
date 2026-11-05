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

#include "easynav_bigmap_maps_manager/BigMapMapsManager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <future>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "easynav_bigmap_maps_manager/marker_publisher.hpp"
#include "easynav_bigmap_maps_manager/narrow_passages.hpp"
#include "easynav_bigmap_maps_manager/navmap_dem.hpp"
#include "easynav_bigmap_maps_manager/pipeline_check.hpp"
#include "easynav_common/PackageShare.hpp"
#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_bigmap_maps_manager/dem_grid.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navmap_core/Costs.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/header.hpp"

namespace easynav
{
namespace bigmap
{

namespace
{
// "~" and "~/..." are the user's home; "~user/..." is left as it is (not supported)
std::string expand(std::string p)
{
  if (p == "~" || p.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    p = std::string(home ? home : "") + p.substr(1);
  }
  return p;
}
// key of a cell of a 2D grid (indices shifted as unsigned: negative ones are fine)
std::int64_t grid_key(std::int64_t i, std::int64_t j)
{
  return static_cast<std::int64_t>(static_cast<std::uint64_t>(i) << 32) ^ (j & 0xffffffffLL);
}
// The arrays of a NavMap file's message, moved out of it: a big map is never built as a NavMap
// (normals, adjacency, BVH: several times the arrays) only to be read once. Its `traversed` and
// `obstacle_hits` layers, whatever their type (a value > 0 counts).
bigmap_maps_manager::SourceMesh source_from_msg(navmap_ros_interfaces::msg::NavMap && msg)
{
  bigmap_maps_manager::SourceMesh m;
  m.x = std::move(msg.positions_x);
  m.y = std::move(msg.positions_y);
  m.z = std::move(msg.positions_z);
  m.v0 = std::move(msg.navcels_v0);
  m.v1 = std::move(msg.navcels_v1);
  m.v2 = std::move(msg.navcels_v2);
  const std::size_t nv = m.x.size(), nt = m.v0.size();
  if (msg.has_vertex_rgba && msg.colors_r.size() == nv && msg.colors_g.size() == nv &&
    msg.colors_b.size() == nv)
  {
    m.r = std::move(msg.colors_r);
    m.g = std::move(msg.colors_g);
    m.b = std::move(msg.colors_b);
  }
  for (auto & l : msg.layers) {
    if ((l.name != "traversed" && l.name != "obstacle_hits") || !l.indices.empty()) {continue;}
    auto & out = l.name == "traversed" ? m.traversed : m.obstacle_hits;
    const auto take = [&out, nt](const auto & data) {
        if (data.size() != nt) {return;}
        out.resize(nt);
        for (std::size_t t = 0; t < nt; ++t) {
          const double v = static_cast<double>(data[t]);
          out[t] = static_cast<std::uint8_t>(v > 0.0 ? std::min(v, 255.0) : 0.0);
        }
      };
    switch (l.type) {
      case navmap_ros_interfaces::msg::NavMapLayer::U8:
        if (l.data_u8.size() == nt) {out = std::move(l.data_u8);}
        break;
      case navmap_ros_interfaces::msg::NavMapLayer::F32: take(l.data_f32); break;
      case navmap_ros_interfaces::msg::NavMapLayer::F64: take(l.data_f64); break;
      case navmap_ros_interfaces::msg::NavMapLayer::I16: take(l.data_i16); break;
      default: break;
    }
    if (l.name == "traversed") {m.has_traversed_layer = true;}
  }
  msg = navmap_ros_interfaces::msg::NavMap();
  return m;
}
// seconds of the monotonic wall clock (the watchdog of update() must not use the ROS clock)
double wall_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

void BigMapMapsManager::create_services(const std::string & base)
{
  auto node = get_node();
  // The service only queues the request: EasyNav serves it from its non-real-time executor while
  // update() runs in the real-time one, so the file is written by update() (the thread that owns
  // the mesh) in its next cycle, and the result is logged. What the callback reads here (whether
  // anything was traversed) is an atomic count
  auto request_save = [this](bool traversed_only, std_srvs::srv::Trigger::Response & res) {
      if (traversed_only && !has_known()) {
        res.success = false;
        res.message = "the robot has not driven over any ground yet";
        return;
      }
      auto req = std::make_shared<SaveRequest>();
      req->traversed_only = traversed_only;
      char stamp[32];
      const std::time_t tt = std::time(nullptr);
      std::tm local{};
      localtime_r(&tt, &local);
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
      const std::string stem = save_directory_ + "/map_" +
        (traversed_only ? "traversed_" : "all_") +
        stamp;
      req->path = stem + ".navmap";
      // a second save within the same second must not overwrite the first one
      for (int n = 2; std::filesystem::exists(req->path) && n < 1000; ++n) {
        req->path = stem + "_" + std::to_string(n) + ".navmap";
      }
      std::lock_guard<std::mutex> lock(save_mutex_);
      if (pending_save_) {
        res.success = false;
        res.message = "another save is in progress";
        return;
      }
      pending_save_ = req;
      res.success = true;
      res.message = "writing " + req->path + " (see the maps manager log)";
    };
  save_all_srv_ = node->create_service<std_srvs::srv::Trigger>(
  base + "/save_map",
    [request_save](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {request_save(false, *res);});
  save_traversed_srv_ = node->create_service<std_srvs::srv::Trigger>(
  base + "/save_traversed",
    [request_save](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {request_save(true, *res);});
// For a recovery: forget every obstacle the memory holds, fixed ones included. Queued like the
// saves (update() does it in its next cycle); without memory there is nothing to forget
  clear_obstacles_srv_ = node->create_service<std_srvs::srv::Trigger>(
  base + "/clear_obstacles",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success = memory_ != nullptr;
      if (memory_) {
        clear_obstacles_requested_ = true;
        res->message = "the obstacle memory is cleared in the next cycle";
      } else {
        res->message = "this plugin has no obstacle memory (obstacles.detector: navmap_filter)";
      }
  });
}

void BigMapMapsManager::on_initialize()
{
  auto node = get_node();
  const std::string name = get_plugin_name();
  auto param = [&](const std::string & key, auto def) {
      easynav::declare_parameter_if_absent(*node, name + "." + key, def);
      node->get_parameter(name + "." + key, def);
      return def;
    };
  // New parameter names, with the previous ones still accepted (the first one set wins)
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  std::vector<std::string> deprecated;
  auto param_any = [&](std::initializer_list<const char *> keys, auto def) {
      const char * chosen = *keys.begin();
      for (const char * k : keys) {
        if (overrides.count(name + "." + k)) {chosen = k; break;}
      }
      if (chosen != *keys.begin()) {
        deprecated.push_back(std::string(chosen) + " -> " + *keys.begin());
      }
      return param(chosen, def);
    };

  const std::string dem_file = expand(param_any({"terrain.dem_file", "dem_file"}, std::string()));
  const double wait_s = param_any({"terrain.dem_wait_timeout", "dem_wait_timeout"}, 60.0);
  params_.high_res_radius = param_any({"lengths.radii.full_res", "lod_settings.high_res_radius"},
        10.0);
  params_.min_size = param_any({"lengths.cell_size.min", "lod_settings.min_triangle_size"}, 1.0);
  params_.max_size = param_any({"lengths.cell_size.max", "lod_settings.max_triangle_size"}, 32.0);
  params_.base_error = param_any({"lod.base_error", "lod_settings.base_error"}, 0.02);
  params_.error_growth = param_any({"lod.error_growth", "lod_settings.error_growth"}, 0.05);
  params_.low_res_distance = param_any({"lengths.radii.low_res_start",
        "lod_settings.low_res_distance"}, 200.0);
  hysteresis_ = param_any({"lengths.remesh_step", "lod_settings.hysteresis"}, 2.0);
  profile_enabled_ = param_any({"output.profile.enabled", "profile.enabled"}, false);
  profiler_ = bigmap_maps_manager::Profiler(param_any({"output.profile.period", "profile.period"},
        5.0));
  save_directory_ = expand(param_any({"output.save_directory", "save.directory"},
        std::string("~/.ros/bigmap/saved")));
  trail_enabled_ = param_any({"traversed.enabled", "traversed_ground.enabled"}, true);
  trail_mark_radius_ = param_any({"lengths.radii.traversed_mark", "traversed_ground.mark_radius"},
        1.0);
  {
    // circles:    inside the small circle (high_res_radius) everything is at full resolution;
    //             between it and the larger one (keep_radius) only the ground driven over;
    //             beyond: low, and what was driven over there is forgotten until it is driven
    //             over again.
    // persistent: inside the small circle everything; outside it only the ground driven over, for
    //             good, wherever it is.
    const std::string mode = param_any({"traversed.mode", "traversed_ground.mode"},
          std::string("circles"));
    if (mode != "circles" && mode != "persistent") {
      throw std::invalid_argument("[" + name + "] traversed.mode: circles or persistent");
    }
    trail_persistent_ = mode == "persistent";
    trail_keep_radius_ = param_any({"lengths.radii.traversed_keep", "traversed_ground.keep_radius"},
          40.0);
  }
  {
    // radius that stays at full resolution around the ground driven over (0: the same as the
    // robot's own, high_res_radius), and the distance beyond it over which the resolution fades
    const double margin = param_any({"lengths.radii.traversed_full_res", "traversed_ground.radius"},
          0.0);
    params_.known_margin = margin > 0.0 ? margin : params_.high_res_radius;
    const double fade = std::max(0.5,
          param_any({"lengths.traversed_fade", "traversed_ground.falloff_distance"}, 5.0));
    params_.known_falloff = std::max(
      1.0, (params_.low_res_distance - params_.high_res_radius) / fade);
  }
  max_slope_deg_ = param_any({"terrain.max_slope_deg", "max_slope_deg"}, 35.0);
  // the holes of the map (its obstacles): their border counts as this height error (m)
  const double hole_error = param("lod.hole_error", 0.0);
  // obstacle borders seen from near stay at full resolution (within observed_full_res of them)
  observed_enabled_ = param("lod.observed.enabled", true);
  observed_mark_radius_ = param("lengths.radii.observed_mark", params_.high_res_radius);
  params_.observed_margin = param("lengths.radii.observed_full_res", 1.0);
  recovery_enabled_ = param("recovery.enabled", true);
  recovery_cycles_ = static_cast<int>(param("recovery.no_path_cycles", int64_t{3}));
  recovery_max_factor_ = param("recovery.max_full_res_factor", 4.0);
  async_remesh_ = param("lod.async_remesh", false);
  if (hole_error < 0.0 || observed_mark_radius_ < 0.0 || params_.observed_margin < 0.0 ||
    recovery_cycles_ < 1 || recovery_max_factor_ < 1.0)
  {
    throw std::invalid_argument(
            "[" + name + "] lod.hole_error >= 0, lengths.radii.observed_mark >= 0, "
            "lengths.radii.observed_full_res >= 0, recovery.no_path_cycles >= 1 and "
            "recovery.max_full_res_factor >= 1 are required");
  }
  publish_markers_ = param_any({"output.publish_markers", "publish_markers"}, true);
  dynamic_enabled_ = param("dynamic_map.enabled", true);
  {
    // Only what the sensors see inside the small circle (lengths.radii.full_res, the blue
    // one) is used. max_range can make it smaller; 0 = the whole circle.
    const double requested = param_any({"lengths.radii.sensor_range", "dynamic_map.max_range"},
          0.0);
    dyn_.max_range = requested > 0.0 ?
      std::min(requested, params_.high_res_radius) : params_.high_res_radius;
  }
  dyn_.min_height = param("dynamic_map.min_height", 0.3);
  dyn_.min_vertical_extent = param("dynamic_map.min_vertical_extent", 0.2);
  dyn_.confirm_time = param("dynamic_map.confirm_time", 0.3);
  dyn_.max_height = param("dynamic_map.max_height", 3.0);
  dyn_.forget_time = param("dynamic_map.forget_time", 5.0);
  dyn_.cell = param("dynamic_map.cell", 0.25);
  dyn_.voxel = param("dynamic_map.voxel", 0.1);
  dyn_.ground_fit = param("dynamic_map.ground_fit.enabled", true);
  dyn_.window_length = param("dynamic_map.ground_fit.window_length", 4.0);
  dyn_.window_width = param("dynamic_map.ground_fit.window_width", 3.0);
  dyn_.window_z_min = param("dynamic_map.ground_fit.z_min", -1.0);
  dyn_.window_z_max = param("dynamic_map.ground_fit.z_max", 2.5);
  dyn_.ground_tolerance = param("dynamic_map.ground_fit.tolerance", 0.08);
  dyn_.max_ground_tilt_deg = param("dynamic_map.ground_fit.max_tilt_deg", 40.0);
  dyn_.ground_min_cells = static_cast<int>(param("dynamic_map.ground_fit.min_cells", 12));
  if (dyn_.cell <= 0.0 || dyn_.max_range <= 0.0 || dyn_.forget_time < 0.0 ||
    dyn_.voxel < 0.0 || dyn_.ground_min_cells < 3)
  {
    // (a plane needs 3 cells: fewer would make the RANSAC draw from an empty set)
    throw std::invalid_argument(
            "[" + name + "] invalid dynamic_map parameters: cell > 0, forget_time >= 0, "
            "voxel >= 0 and ground_fit.min_cells >= 3 are required");
  }
  view_max_z_ = param_any({"output.view_max_z", "view_max_z"}, 1e9);
  tint_enabled_ = param_any({"obstacles.tint.enabled", "obstacle_tint.enabled"}, true);
  tint_strength_ = static_cast<float>(param_any({"obstacles.tint.strength",
      "obstacle_tint.strength"}, 0.65));
  {
    const auto rgb = param_any({"obstacles.tint.color", "obstacle_tint.color"},
          std::vector<int64_t>{255, 40, 40});
    if (rgb.size() != 3) {
      throw std::invalid_argument("[" + name + "] obstacles.tint.color needs 3 values");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      tint_rgb_[i] = static_cast<float>(std::clamp<int64_t>(rgb[i], 0, 255));
    }
    tint_strength_ = std::clamp(tint_strength_, 0.0f, 1.0f);
  }
  // obstacles.detector (before: obstacles.source / obstacle_source, values dynamic | filter)
  const auto detector = bigmap_maps_manager::parse_detector(
    param_any({"obstacles.detector", "obstacles.source", "obstacle_source"},
    std::string("bigmap_dynamic")));
  obstacles_from_filter_ = detector == bigmap_maps_manager::ObstacleDetector::NavmapFilter;
  removal_enabled_ = param_any({"obstacles.removal.enabled", "obstacle_removal.enabled"}, true);
  removal_radius_ = param_any({"lengths.radii.obstacle_clearance", "obstacle_removal.radius"}, 1.0);
  if (removal_radius_ < 0.0) {
    throw std::invalid_argument("[" + name + "] lengths.radii.obstacle_clearance must be >= 0");
  }
  bool memory_enabled_at_start = false;   // for the pipeline check below
  {
    // Memory of obstacles: a detection that lasts memory.min_time s over a NavCel is a hit; the
    // count is the layer `obstacle_hits`; memory.max_hits hits fix the obstacle for good
    const bool enabled = param_any({"obstacles.memory.enabled"}, true);
    memory_enabled_at_start = enabled;
    memory_min_time_ = param_any({"obstacles.memory.min_time"}, 5.0);
    memory_max_hits_ = static_cast<int>(param_any({"obstacles.memory.max_hits"}, int64_t{3}));
    if (memory_min_time_ <= 0.0 || memory_max_hits_ < 1 || memory_max_hits_ > 255) {
      throw std::invalid_argument(
              "[" + name +
            "] obstacles.memory: min_time > 0 and 1 <= max_hits <= 255 are required");
    }
  }

  if (!deprecated.empty()) {
    std::string list;
    for (const auto & d : deprecated) {
      list += (list.empty() ? "" : ", ") + d;
    }
    RCLCPP_WARN(
      node->get_logger(), "[%s] old parameter names still accepted, please rename: %s",
      name.c_str(), list.c_str());
  }
  if (params_.min_size <= 0.0 || params_.max_size < params_.min_size ||
    params_.high_res_radius < 0.0 || params_.low_res_distance < params_.high_res_radius ||
    hysteresis_ <= 0.0)
  {
    throw std::invalid_argument("[" + name +
          "] invalid lengths: cell_size.min > 0, cell_size.max >= cell_size.min, "
          "radii.full_res >= 0, radii.low_res_start >= radii.full_res and remesh_step > 0 "
          "are required");
  }
  base_high_res_ = params_.high_res_radius;
  base_low_res_ = params_.low_res_distance;
  if (profile_enabled_ && profiler_.period() <= 0.0) {
    throw std::invalid_argument("[" + name + "] output.profile.period must be > 0");
  }
  if (trail_enabled_ && !trail_persistent_ && trail_keep_radius_ < params_.high_res_radius) {
    throw std::invalid_argument(
            "[" + name + "] lengths.radii.traversed_keep must be >= lengths.radii.full_res");
  }
  std::string initial_navmap = expand(param_any({"terrain.initial_navmap", "initial_navmap"},
        std::string()));
  {
    // terrain.package: a relative initial_navmap is in that package's share directory (as the
    // Bonxai map's `package` + `bonxai_path_file`), so the parameter file says it all
    const auto package = param_any({"terrain.package"}, std::string());
    if (!package.empty() && !initial_navmap.empty() &&
      !std::filesystem::path(initial_navmap).is_absolute())
    {
      try {
        initial_navmap = (easynav::get_package_share_path(package) / initial_navmap).string();
      } catch (const std::exception & e) {
        throw std::runtime_error("[" + name + "] terrain.package '" + package + "': " + e.what());
      }
    }
  }
  const bool multilevel = param("multilevel.enabled", false);
  bigmap_maps_manager::SurfaceLodMesher::Config surface_cfg;
  surface_cfg.cluster_height = param("multilevel.cluster_height", 1.2);
  surface_cfg.vertical_tolerance = param("multilevel.vertical_tolerance", 1.5);
  surface_cfg.vertical_weight = param("multilevel.vertical_weight", 4.0);
  const double keep_narrow = param_any(
    {"terrain.keep_narrow_width", "multilevel.keep_narrow_width"}, 0.0);
  surface_cfg.keep_narrow_width = keep_narrow;
  const bool drop_unreachable = param("terrain.drop_unreachable_ground", false);
  surface_cfg.drop_unreachable_ground = drop_unreachable;
  surface_cfg.hole_error = hole_error;
  if (multilevel && initial_navmap.empty()) {
    throw std::invalid_argument("[" + name + "] multilevel.enabled needs terrain.initial_navmap");
  }
  std::optional<bigmap_maps_manager::DemGrid> dem;
  std::vector<std::uint8_t> valid;
  std::vector<std::uint8_t> cell_valid;   // the holes of a NavMap, exactly (see NavMapDem)
  std::vector<std::pair<int, int>> restored;
  std::vector<std::array<float, 4>> saved_hits;   // x, y, z, hits of a saved map's obstacle_hits
  if (!initial_navmap.empty()) {
    // Any NavMap, whatever its resolution: it is turned into the height grid the LOD works from
    // (centred on its own (0, 0)) and the robot's movement refines it from there
    // read as the message's arrays (see source_from_msg): never as a NavMap
    bigmap_maps_manager::SourceMesh loaded;
    {
      navmap_ros_interfaces::msg::NavMap msg;
      std::error_code ec;
      if (!navmap_ros::io::load_msg_from_file(initial_navmap, msg, &ec)) {
        throw std::runtime_error(
                "[" + name + "] cannot load initial_navmap '" + initial_navmap + "' (" +
                ec.message() + ")");
      }
      loaded = source_from_msg(std::move(msg));
    }
    // a corrupt file (a NaN vertex, an index out of range) would reach every mesh, or an index
    // cast: refused here, by name
    try {
      loaded.validate();
    } catch (const std::invalid_argument & e) {
      throw std::runtime_error(
              "[" + name + "] initial_navmap '" + initial_navmap + "': " + e.what());
    }
    if (loaded.triangle_count() == 0) {
      throw std::runtime_error("[" + name + "] initial_navmap '" + initial_navmap +
              "' has no NavCels");
    }
    origin_ = bigmap_maps_manager::load_origin(initial_navmap);
    if (!loaded.obstacle_hits.empty()) {
      // what a saved map counted (steep ground is no obstacle of the memory: it is steep anyway)
      const float min_nz = static_cast<float>(std::cos(max_slope_deg_ * M_PI / 180.0));
      const auto & X = loaded.x;
      const auto & Y = loaded.y;
      const auto & Z = loaded.z;
      for (std::size_t c = 0; c < loaded.triangle_count(); ++c) {
        const auto hits = loaded.obstacle_hits[c];
        if (hits == 0) {continue;}
        const std::array<std::uint32_t, 3> v{loaded.v0[c], loaded.v1[c], loaded.v2[c]};
        const double ux = X[v[1]] - X[v[0]], uy = Y[v[1]] - Y[v[0]], uz = Z[v[1]] - Z[v[0]],
          wx = X[v[2]] - X[v[0]], wy = Y[v[2]] - Y[v[0]], wz = Z[v[2]] - Z[v[0]];
        const double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
        const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-12 || std::abs(nz) / len < min_nz) {continue;}
        saved_hits.push_back({(X[v[0]] + X[v[1]] + X[v[2]]) / 3.0f,
            (Y[v[0]] + Y[v[1]] + Y[v[2]]) / 3.0f, (Z[v[0]] + Z[v[1]] + Z[v[2]]) / 3.0f,
            static_cast<float>(hits)});
      }
    }
    if (multilevel) {
      surface_cfg.min_size = params_.min_size;
      surface_cfg.max_size = params_.max_size;
      surface_ = std::make_unique<bigmap_maps_manager::SurfaceLodMesher>(loaded, surface_cfg);
      if (surface_->dropped_triangles() > 0) {
        RCLCPP_INFO(
          node->get_logger(),
          "[%s] %zu NavCels not joined to the largest piece of the map (the roof...) are dropped",
          name.c_str(), surface_->dropped_triangles());
      }
      RCLCPP_INFO(
        node->get_logger(),
        "[%s] initial NavMap %s (multi-level): %zu NavCels, %zu welded vertices (%zu in narrow "
        "passages, kept at full resolution), %d levels of detail, %.1f MB; %s; %s", name.c_str(),
        initial_navmap.c_str(), loaded.triangle_count(), surface_->vertex_count(),
        surface_->narrow_vertex_count(), surface_->level_count(),
        surface_->memory_bytes() / 1e6,
        origin_.georeferenced ? "georeferenced (origin file found)" :
        "no geolocation (origin 0, 0, 0)",
        surface_->had_traversed_layer() ? "with a traversed layer" : "no traversed layer");
    } else {
      const double res_param = param_any({"terrain.initial_navmap_resolution",
            "initial_navmap_resolution"}, 0.0);
      const double res = res_param > 0.0 ? res_param : params_.min_size;
      auto nd = bigmap_maps_manager::dem_from_navmap(loaded, res);
      nd.dem.origin_lat = origin_.lat;
      nd.dem.origin_lon = origin_.lon;
      nd.dem.reference_altitude = origin_.altitude;
      dem = std::move(nd.dem);
      valid = std::move(nd.valid);
      cell_valid = std::move(nd.cell_valid);
      restored = std::move(nd.traversed_nodes);
      RCLCPP_INFO(
        node->get_logger(),
        "[%s] initial NavMap %s: %zu NavCels -> %dx%d grid @ %.2f m; %s; %s", name.c_str(),
        initial_navmap.c_str(), loaded.triangle_count(), dem->side, dem->side, res,
        origin_.georeferenced ? "georeferenced (origin file found)" :
        "no geolocation (origin 0, 0, 0)",
        nd.has_traversed ? "with a traversed layer" : "no traversed layer");
    }
  } else {
    if (dem_file.empty()) {
      throw std::invalid_argument(
              "[" + name + "] parameter terrain.dem_file (or terrain.initial_navmap) is required");
    }
    // The generator (navmap_gis_tool --dem-out) writes the DEM atomically; wait for it if needed
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_s);
    while (!(dem = bigmap_maps_manager::DemGrid::load(dem_file))) {
      if (std::chrono::steady_clock::now() > deadline) {
        throw std::runtime_error(
                "[" + name + "] cannot load DEM '" + dem_file +
                "' (run navmap_gis_tool --dem-out first)");
      }
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000, "[%s] waiting for DEM '%s'...",
        name.c_str(), dem_file.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    origin_ = {true, dem->origin_lat, dem->origin_lon, dem->reference_altitude};
    if (!dem->cell_valid.empty()) {
      // a DEM with holes (buildings, walls: navmap_gis_tool --dem-holes-mesh): its cells are the
      // ground the robot can be on, as a NavMap's are (exact holes, hole_error, observed borders);
      // a node is valid if one of its cells has ground
      cell_valid = std::move(dem->cell_valid);
      dem->cell_valid.clear();
      const int side = dem->side, nc = side - 1;
      valid.assign(static_cast<std::size_t>(side) * side, 0);
      for (int j = 0; j < nc; ++j) {
        for (int i = 0; i < nc; ++i) {
          if (!cell_valid[static_cast<std::size_t>(j) * nc + i]) {continue;}
          for (const auto & [di, dj] : {std::pair{0, 0}, std::pair{1, 0}, std::pair{0, 1},
              std::pair{1, 1}})
          {
            valid[static_cast<std::size_t>(j + dj) * side + i + di] = 1;
          }
        }
      }
      RCLCPP_INFO(
        node->get_logger(), "[%s] the DEM has holes: %zu of %zu cells without ground",
        name.c_str(),
        static_cast<std::size_t>(std::count(cell_valid.begin(), cell_valid.end(), 0)),
        cell_valid.size());
    }
  }
  if (surface_) {
    if (surface_->traversed_count() > 0) {
      RCLCPP_INFO(
        node->get_logger(), "[%s] restored %zu traversed vertices", name.c_str(),
        surface_->traversed_count());
    }
  } else {
    params_.min_size = std::max(params_.min_size, dem->resolution);  // cannot beat the data
    if (drop_unreachable) {
      std::size_t dropped = 0;
      auto kept = bigmap_maps_manager::keep_main_piece(*dem, valid, max_slope_deg_, &dropped);
      if (!cell_valid.empty()) {
        // the cells with a corner on a dropped piece go with it
        const int side = dem->side, nc = side - 1;
        const auto gone = [&](int i, int j) {
            const auto k = static_cast<std::size_t>(j) * side + i;
            return valid[k] && !kept[k];
          };
        for (int j = 0; j < nc; ++j) {
          for (int i = 0; i < nc; ++i) {
            if (gone(i, j) || gone(i + 1, j) || gone(i, j + 1) || gone(i + 1, j + 1)) {
              cell_valid[static_cast<std::size_t>(j) * nc + i] = 0;
            }
          }
        }
      }
      valid = std::move(kept);
      RCLCPP_INFO(
        node->get_logger(), "[%s] %zu nodes of the grid are on ground the robot cannot get to "
        "(roofs, plateaus with no way up) and are dropped", name.c_str(), dropped);
    }
    const auto pinned = bigmap_maps_manager::narrow_passage_nodes(
      *dem, valid, max_slope_deg_, keep_narrow);
    const auto n_pinned = static_cast<std::size_t>(std::count(pinned.begin(), pinned.end(), 1));
    if (keep_narrow > 0.0) {
      RCLCPP_INFO(
        node->get_logger(), "[%s] %zu of %zu nodes in passages narrower than %.1f m, kept at full "
        "resolution", name.c_str(), n_pinned, dem->z.size(), keep_narrow);
    }
    const int side = dem->side;
    const double resolution = dem->resolution;
    mesher_ = std::make_unique<bigmap_maps_manager::LodMesher>(
      std::move(*dem), std::move(valid), pinned, std::move(cell_valid), hole_error);
    dem.reset();
    known_ = std::make_unique<bigmap_maps_manager::KnownMap>(side, resolution);
    if (observed_enabled_ && !mesher_->cell_valid().empty()) {
      observed_ = std::make_unique<bigmap_maps_manager::KnownMap>(side, resolution);
    }
    RCLCPP_INFO(
      node->get_logger(), "[%s] grid mesher: %.1f MB (height grid, error map, masks)",
      name.c_str(), mesher_->memory_bytes() / 1e6);
    for (const auto & [i, j] : restored) {
      known_->mark_node(i, j);
    }
    if (!restored.empty()) {
      RCLCPP_INFO(
        node->get_logger(), "[%s] restored %zu traversed nodes", name.c_str(), known_->count());
    }
  }
  // with the filter detector the pipeline check says why not
  if (!obstacles_from_filter_) {
    // its cells are NavCels of the finest size the map has (cell_size.min, now final)
    memory_ = std::make_unique<bigmap_maps_manager::ObstacleMemory>(
      std::max(params_.min_size, 0.25), memory_min_time_, memory_max_hits_);
    RCLCPP_INFO(
      node->get_logger(), "[%s] obstacle memory (layer obstacle_hits): %s; a detection of %.1f s "
      "over a NavCel is a hit, %d hits fix the obstacle (cells of %.2f m)", name.c_str(),
      memory_enabled_at_start ? "on" : "off (can be switched on at runtime)", memory_min_time_,
      memory_max_hits_, memory_->cell_size());
    for (const auto & h : saved_hits) {
      memory_->restore(h[0], h[1], h[2], static_cast<int>(h[3]));
    }
    if (!saved_hits.empty()) {
      RCLCPP_INFO(
        node->get_logger(), "[%s] obstacle memory restored from the saved map: %zu cells (%zu "
        "fixed)", name.c_str(), memory_->cells().size(), memory_->fixed_count());
    }
  }
  map_frame_ = RTTFBuffer::getInstance()->get_tf_info().map_frame;

  const std::string base = node->get_node_base_interface()->get_fully_qualified_name() +
    std::string("/") + name;
  navmap_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    base + "/map", rclcpp::QoS(1).transient_local().reliable());
  create_services(base);
  markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    base + "/markers", rclcpp::QoS(1).transient_local().reliable());

  dynamic_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    base + "/dynamic_map", rclcpp::QoS(1).transient_local().reliable());
  layer_updates_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMapLayer>(
    base + "/map_updates", rclcpp::QoS(100));
  map_resolution_ = static_cast<float>(surface_ ? params_.min_size :
    mesher_->dem().resolution);

  // NavMap filters: the same plugins (and parameters) as in NavMapMapsManager. The `filters` list
  // is the only on/off switch of each of them: a filter runs if and only if it is listed (and has
  // a `<name>.plugin`). Their instance names are free; what a filter is is told by the layer it
  // writes.
  std::vector<bigmap_maps_manager::LoadedFilter> loaded_filters;
  filters_loader_ = std::make_unique<pluginlib::ClassLoader<::easynav::navmap::NavMapFilter>>(
  "easynav_navmap_maps_manager", "easynav::navmap::NavMapFilter");
  std::vector<std::string> filter_names;
  easynav::declare_parameter_if_absent(*node, name + ".filters", filter_names);
  node->get_parameter(name + ".filters", filter_names);
  for (const auto & fname : filter_names) {
    std::string plugin;
    easynav::declare_parameter_if_absent(*node, name + "." + fname + ".plugin", plugin);
    node->get_parameter(name + "." + fname + ".plugin", plugin);
    if (plugin.empty()) {
      RCLCPP_WARN(
      node->get_logger(),
      "[%s] filter '%s' is listed in 'filters' but has no '%s.plugin' parameter: skipped%s",
      name.c_str(), fname.c_str(), fname.c_str(),
      fname == "obstacles" ?
      " ('obstacles' is bigmap's own section: name an obstacle filter 'obstacle_filter')" : "");
      continue;
    }
    try {
      auto instance = filters_loader_->createSharedInstance(plugin);
      instance->initialize(node, name + "." + fname);
      filters_.push_back(instance);
      loaded_filters.push_back({fname, instance->get_layer_name()});
      RCLCPP_INFO(
      node->get_logger(), "[%s] loaded NavMapFilter %s [%s] (layer '%s')", name.c_str(),
      fname.c_str(), plugin.c_str(), instance->get_layer_name().c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      throw std::runtime_error(
            "[" + name + "] unable to load NavMapFilter " + fname + " [" + plugin + "]: " +
            ex.what());
    }
  }

  {
  // The options that decide how obstacles reach the planner depend on each other: check them
  // together and say what is on (see pipeline_check.hpp)
    bigmap_maps_manager::PipelineSettings settings;
    settings.detector = detector;
    settings.dynamic_map_enabled = dynamic_enabled_;
    settings.memory_enabled = memory_enabled_at_start;
    settings.removal_enabled = removal_enabled_;
    settings.filters = loaded_filters;
    const auto report = bigmap_maps_manager::check_pipeline(settings);
    for (const auto & n : report.notes) {
      RCLCPP_INFO(node->get_logger(), "[%s] %s", name.c_str(), n.c_str());
    }
    for (const auto & w : report.warnings) {
      RCLCPP_WARN(node->get_logger(), "[%s] %s", name.c_str(), w.c_str());
    }
    if (!report.errors.empty()) {
      throw std::invalid_argument("[" + name + "] " + report.errors.front());
    }
  }


  RCLCPP_INFO(
    node->get_logger(),
    "[%s] lengths (m): radii traversed_mark %.1f, traversed_full_res %.1f, "
    "obstacle_clearance %.1f, "
    "sensor_range %.1f, full_res %.1f, low_res_start %.1f, traversed_keep %.1f (%s); "
    "cell_size %.2f..%.1f", name.c_str(), trail_mark_radius_, params_.known_margin,
    removal_radius_, dyn_.max_range, params_.high_res_radius, params_.low_res_distance,
    trail_keep_radius_, trail_persistent_ ? "unused: persistent" : "circles", params_.min_size,
    params_.max_size);
  if (surface_) {
    RCLCPP_INFO(
      node->get_logger(), "[%s] multi-level LOD radius %.0f m, clusters %.1f..%.0f m, imagery %s",
      name.c_str(), params_.high_res_radius, params_.min_size, params_.max_size,
      surface_->coloured() ? "yes" : "no");
  } else {
    const auto & d = mesher_->dem();
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] DEM %dx%d @ %.1f m (RTIN tile %d), LOD radius %.0f m, leaf %.1f..%.0f m, imagery %s",
      name.c_str(), d.side, d.side, d.resolution, mesher_->error_map().tile_size(),
      params_.high_res_radius, params_.min_size, params_.max_size,
      d.rgb.empty() ? "no" : "yes");
  }
  RCLCPP_INFO(
    node->get_logger(), "[%s] obstacle borders seen from near kept fine: %s (within %.1f m, "
    "%.1f m around them); hole error %.2f m; recovery on no path: %s; re-mesh in a worker: %s",
    name.c_str(), observed_enabled_ ? "yes" : "no", observed_mark_radius_,
    params_.observed_margin, hole_error, recovery_enabled_ ? "yes" : "no",
    async_remesh_ ? "yes" : "no");
  watchdog_.start(wall_seconds());
  watchdog_timer_ = node->create_wall_timer(
    std::chrono::seconds(5), [this]() {check_watchdog();});
}

void BigMapMapsManager::check_watchdog()
{
  auto node = get_node();
  const auto event = watchdog_.check(wall_seconds());
  if (event == bigmap_maps_manager::UpdateWatchdog::Event::None) {return;}
  if (event == bigmap_maps_manager::UpdateWatchdog::Event::Resumed) {
    RCLCPP_INFO(node->get_logger(), "[%s] update() runs again", get_plugin_name().c_str());
    return;
  }
  // not throttled on the ROS clock: that clock may be the one that does not move
  const bool sim_time = node->get_parameter("use_sim_time").as_bool();
  const double ros_now = node->get_clock()->now().seconds();
  std::string why;
  if (sim_time && ros_now == 0.0) {
    why = ": the ROS clock of this node is still at 0 with use_sim_time true, so no simulated "
      "time (/clock) has reached it, and EasyNav only runs a component when its clock says it is "
      "due. Check that the simulation publishes /clock; if it does, restart the application";
  } else if (sim_time) {
    why = ": the ROS clock of this node is at " + std::to_string(ros_now) +
      " s (use_sim_time true): is the simulation paused or stopped?";
  } else {
    why = ": EasyNav is not running the maps manager (is it active? is its cycle stuck?)";
  }
  RCLCPP_WARN(
    node->get_logger(), "[%s] update() has not run for %.0f s (wall time)%s",
    get_plugin_name().c_str(), watchdog_.stalled_s(), why.c_str());
}

bool BigMapMapsManager::has_known() const
{
  if (surface_) {return surface_->traversed_count() > 0;}
  return known_ && !known_->empty();
}

bool BigMapMapsManager::robot_position(
  const ::easynav::NavState & nav_state, double & x, double & y, double & z,
  std::array<double, 4> & q)
{
  // a NaN pose (a localizer that has not converged) would freeze re-meshing for good: it is no pose
  const auto finite = [&]() {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
             std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) &&
             std::isfinite(q[3]);
    };
  if (nav_state.has("robot_pose")) {
    const auto pose = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
    x = pose.position.x; y = pose.position.y; z = pose.position.z;
    q = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    return finite();
  }
  try {
    const auto & info = RTTFBuffer::getInstance()->get_tf_info();
    const auto tf = RTTFBuffer::getInstance()->lookupTransform(
      info.map_frame, info.robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    z = tf.transform.translation.z;
    const auto & r = tf.transform.rotation;
    q = {r.x, r.y, r.z, r.w};
    return finite();
  } catch (const tf2::TransformException &) {
    return false;
  }
}

::navmap::NavMap BigMapMapsManager::build_navmap(const bigmap_maps_manager::LodMesh & mesh) const
{
  ::navmap::NavMap nm;
  const std::size_t nv = mesh.x.size(), nt = mesh.triangle_count();
  nm.positions.x = mesh.x;
  nm.positions.y = mesh.y;
  nm.positions.z = mesh.z;
  if (!mesh.r.empty()) {
    ::navmap::Colors c;
    c.r = mesh.r; c.g = mesh.g; c.b = mesh.b;
    c.a.assign(nv, 255);
    nm.colors = std::move(c);
  }
  nm.navcels.resize(nt);
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = map_frame_;
  nm.surfaces[0].navcels.resize(nt);
  for (std::size_t t = 0; t < nt; ++t) {
    nm.navcels[t].v[0] = mesh.tri[3 * t];
    nm.navcels[t].v[1] = mesh.tri[3 * t + 1];
    nm.navcels[t].v[2] = mesh.tri[3 * t + 2];
    nm.surfaces[0].navcels[t] = static_cast<::navmap::NavCelId>(t);
  }
  nm.rebuild_geometry_accels();

  nm.add_layer<std::uint8_t>(
    "occupancy", "static map: terrain too steep to drive", "", ::navmap::FREE_SPACE);
  nm.add_layer<std::uint8_t>(
    "obstacles", "occupancy plus what the sensors detect", "", ::navmap::FREE_SPACE);
  nm.add_layer<float>("elevation", "mean triangle height above the map origin", "m");
  nm.add_layer<float>("lod_size", "leg length of the LOD triangle", "m");
  nm.add_layer<std::uint8_t>(
    "traversed", "255: ground the robot has driven over (high confidence)", "", 0);
  nm.add_layer<std::uint8_t>(
    "obstacle_hits", "times an obstacle has been detected here (steep ground: obstacles.memory."
    "max_hits); max_hits = fixed obstacle", "", 0);
  const float min_nz = static_cast<float>(std::cos(max_slope_deg_ * M_PI / 180.0));
  for (std::size_t t = 0; t < nt; ++t) {
    const auto & v = nm.navcels[t].v;
    const Eigen::Vector3f a(nm.positions.x[v[0]], nm.positions.y[v[0]], nm.positions.z[v[0]]);
    const Eigen::Vector3f b(nm.positions.x[v[1]], nm.positions.y[v[1]], nm.positions.z[v[1]]);
    const Eigen::Vector3f c(nm.positions.x[v[2]], nm.positions.y[v[2]], nm.positions.z[v[2]]);
    const Eigen::Vector3f n = (b - a).cross(c - a).normalized();
    const auto id = static_cast<::navmap::NavCelId>(t);
    // multi-level: the slope of the source NavCels (clustering makes the geometry steeper)
    const bool steep = mesh.slope.size() == nt ?
      mesh.slope[t] > static_cast<float>(max_slope_deg_) : n.z() < min_nz;
    if (steep) {
      nm.layer_set<std::uint8_t>("occupancy", id, ::navmap::LETHAL_OBSTACLE);
      nm.layer_set<std::uint8_t>("obstacles", id, ::navmap::LETHAL_OBSTACLE);
      // too steep to drive is an obstacle for good: it counts as fixed
      nm.layer_set<std::uint8_t>("obstacle_hits", id, static_cast<std::uint8_t>(memory_max_hits_));
    }
    nm.layer_set<float>("elevation", id, (a.z() + b.z() + c.z()) / 3.0f);
    nm.layer_set<float>("lod_size", id, mesh.leg[t]);
    if (is_traversed(mesh, t)) {nm.layer_set<std::uint8_t>("traversed", id, 255);}
  }
  return nm;
}

bool BigMapMapsManager::memory_active() const
{
  if (!memory_) {return false;}
  bool on = true;
  const std::string key = get_plugin_name() + ".obstacles.memory.enabled";
  if (get_node()->has_parameter(key)) {get_node()->get_parameter(key, on);}
  return on;
}

void BigMapMapsManager::triangles_of(
  const bigmap_maps_manager::ObstacleMemory::Cell & cell, std::vector<std::uint32_t> & out) const
{
  // A memory cell is a square of `cell_size` metres (it was found by one detection, but the whole
  // square is the obstacle): probe it finely enough to reach every NavCel of the mesh in it
  const double size = memory_->cell_size();
  const double x0 = std::floor(cell.x / size) * size, y0 = std::floor(cell.y / size) * size;
  const double step = std::max(0.1, 0.5 * params_.min_size);
  const int n = std::max(1, static_cast<int>(std::ceil(size / step)));
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      const int t = locator_->locate_at(
        x0 + (i + 0.5) * size / n, y0 + (j + 0.5) * size / n, cell.z);
      if (t >= 0) {out.push_back(static_cast<std::uint32_t>(t));}
    }
  }
}

void BigMapMapsManager::reset_hits(::navmap::NavMap & nm) const
{
  // what the layer holds with no memory: steep ground (an obstacle for good) and nothing else
  const auto occ = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    nm.layers.get("occupancy"));
  if (!occ || !nm.layers.get("obstacle_hits") || occ->size() != nm.navcels.size()) {return;}
  for (std::size_t c = 0; c < nm.navcels.size(); ++c) {
    const std::uint8_t v = occ->data()[c] == ::navmap::LETHAL_OBSTACLE ?
      static_cast<std::uint8_t>(memory_max_hits_) : 0;
    const auto id = static_cast<::navmap::NavCelId>(c);
    if (nm.layer_get<std::uint8_t>("obstacle_hits", id, 0) != v) {
      nm.layer_set<std::uint8_t>("obstacle_hits", id, v);
    }
  }
}

void BigMapMapsManager::fixed_seeds(std::vector<std::uint32_t> & seeds) const
{
  // A fixed obstacle blocks the NavCels of its own size scale. Far from the robot the mesh is
  // coarse: one of its NavCels can be tens of metres, and blocking it for a 1 m obstacle would
  // close whole corridors to any plan made from afar. There it waits until the mesh around it is
  // fine.
  const double coarse = std::max(4.0 * params_.min_size, 2.0 * memory_->cell_size());
  std::vector<std::uint32_t> tris;
  for (const auto & kv : memory_->cells()) {
    if (!memory_->fixed(kv.second)) {continue;}
    tris.clear();
    triangles_of(kv.second, tris);
    for (const auto t : tris) {
      if (mesh_.leg[t] <= coarse) {seeds.push_back(t);}
    }
  }
}

void BigMapMapsManager::write_hits(::navmap::NavMap & nm) const
{
  if (!memory_ || !locator_ || !nm.layers.get("obstacle_hits")) {return;}
  std::vector<std::uint32_t> tris;
  for (const auto & kv : memory_->cells()) {
    tris.clear();
    triangles_of(kv.second, tris);
    for (const auto t : tris) {
      const auto id = static_cast<::navmap::NavCelId>(t);
      if (kv.second.hits > nm.layer_get<std::uint8_t>("obstacle_hits", id, 0)) {
        nm.layer_set<std::uint8_t>("obstacle_hits", id, static_cast<std::uint8_t>(kv.second.hits));
      }
    }
  }
}

bool BigMapMapsManager::is_traversed(const bigmap_maps_manager::LodMesh & mesh, std::size_t t) const
{
  if (surface_) {
    return trail_enabled_ && t < mesh.traversed.size() && mesh.traversed[t];
  }
  if (!trail_enabled_ || !known_ || known_->empty()) {return false;}
  const auto & dem = mesher_->dem();
  const int reach = static_cast<int>(std::ceil(std::max(trail_mark_radius_, dem.resolution) /
    dem.resolution));
  const int center = (dem.side - 1) / 2;
  for (int k = 0; k < 3; ++k) {
    const auto v = mesh.tri[3 * t + k];
    const int ix = static_cast<int>(std::lround(mesh.x[v] / dem.resolution)) + center;
    const int iy = static_cast<int>(std::lround(mesh.y[v] / dem.resolution)) + center;
    if (!known_->near_marked(ix, iy, reach)) {return false;}
  }
  return true;
}

navmap_ros_interfaces::msg::NavMap BigMapMapsManager::make_map_msg(
  const std::vector<char> & removed, const ::navmap::NavMap & layers_from,
  const rclcpp::Time & stamp, bool tint) const
{
  navmap_ros_interfaces::msg::NavMap msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = stamp;
  msg.positions_x = mesh_.x;
  msg.positions_y = mesh_.y;
  msg.positions_z = mesh_.z;
  const bool imagery = !mesh_.r.empty();
  if (imagery) {
    msg.colors_r = mesh_.r;
    msg.colors_g = mesh_.g;
    msg.colors_b = mesh_.b;
    msg.colors_a.assign(mesh_.r.size(), 255);
  }
  // Obstacle NavCels (the `obstacles` layer, so the terrain too steep to drive and what the sensors
  // found) are painted with a tint over the imagery. Vertices are shared between NavCels, so each
  // of them gets its own copy of its three vertices to be coloured on its own.
  // the uint8 layers of the NavMap in use, looked up once (not once per NavCel)
  const auto u8 = [&layers_from, this](const char * lname) -> const std::vector<std::uint8_t> * {
      const auto view = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
        layers_from.layers.get(lname));
      return view && view->size() == mesh_.triangle_count() ? &view->data() : nullptr;
    };
  const auto * obstacles = u8("obstacles");
  auto lethal = [&](std::size_t t) {
      return tint && tint_enabled_ && obstacles && (*obstacles)[t] == ::navmap::LETHAL_OBSTACLE;
    };
  auto tinted = [&](std::uint8_t base, int channel) {
      return static_cast<std::uint8_t>(base + (tint_rgb_[channel] - base) * tint_strength_);
    };
  std::vector<std::uint32_t> kept;  // original ids of the NavCels that stay
  kept.reserve(mesh_.triangle_count());
  for (std::size_t t = 0; t < mesh_.triangle_count(); ++t) {
    if (!removed.empty() && removed[t]) {continue;}
    if (tint && view_max_z_ < 1e8) {  // what is shown (a saved map keeps everything)
      const auto a = mesh_.tri[3 * t], b = mesh_.tri[3 * t + 1], c = mesh_.tri[3 * t + 2];
      if ((mesh_.z[a] + mesh_.z[b] + mesh_.z[c]) / 3.0 > view_max_z_) {continue;}
    }
    kept.push_back(static_cast<std::uint32_t>(t));
    if (!lethal(t)) {
      msg.navcels_v0.push_back(mesh_.tri[3 * t]);
      msg.navcels_v1.push_back(mesh_.tri[3 * t + 1]);
      msg.navcels_v2.push_back(mesh_.tri[3 * t + 2]);
      continue;
    }
    if (!imagery && msg.colors_r.empty()) {  // no imagery: a neutral grey to tint
      msg.colors_r.assign(msg.positions_x.size(), 170);
      msg.colors_g = msg.colors_r;
      msg.colors_b = msg.colors_r;
      msg.colors_a.assign(msg.positions_x.size(), 255);
    }
    std::uint32_t id[3];
    for (int k = 0; k < 3; ++k) {
      const auto v = mesh_.tri[3 * t + k];
      id[k] = static_cast<std::uint32_t>(msg.positions_x.size());
      msg.positions_x.push_back(mesh_.x[v]);
      msg.positions_y.push_back(mesh_.y[v]);
      msg.positions_z.push_back(mesh_.z[v]);
      const std::uint8_t r = imagery ? mesh_.r[v] : 170, g = imagery ? mesh_.g[v] : 170,
        b = imagery ? mesh_.b[v] : 170;
      msg.colors_r.push_back(tinted(r, 0));
      msg.colors_g.push_back(tinted(g, 1));
      msg.colors_b.push_back(tinted(b, 2));
      msg.colors_a.push_back(255);
    }
    msg.navcels_v0.push_back(id[0]);
    msg.navcels_v1.push_back(id[1]);
    msg.navcels_v2.push_back(id[2]);
  }
  msg.has_vertex_rgba = !msg.colors_r.empty();
  navmap_ros_interfaces::msg::NavMapSurface surf;
  surf.frame_id = map_frame_;
  surf.navcels.resize(kept.size());
  for (std::size_t k = 0; k < kept.size(); ++k) {
    surf.navcels[k] = static_cast<uint32_t>(k);
  }
  msg.surfaces.push_back(std::move(surf));

  auto float_layer = [&](const std::string & lname, const std::string & desc, auto value) {
      navmap_ros_interfaces::msg::NavMapLayer l;
      l.header = msg.header;
      l.name = lname;
      l.type = navmap_ros_interfaces::msg::NavMapLayer::F32;
      l.unit = "m";
      l.description = desc;
      l.data_f32.resize(kept.size());
      for (std::size_t k = 0; k < kept.size(); ++k) {
        l.data_f32[k] = value(kept[k]);
      }
      msg.layers.push_back(std::move(l));
    };
  float_layer(
    "elevation", "mean triangle height above the map origin", [&](std::uint32_t t) {
      return (mesh_.z[mesh_.tri[3 * t]] + mesh_.z[mesh_.tri[3 * t + 1]] +
             mesh_.z[mesh_.tri[3 * t + 2]]) / 3.0f;
    });
  float_layer("lod_size", "leg length of the LOD triangle", [&](std::uint32_t t) {
      return mesh_.leg[t];
    });
  if (trail_enabled_ && has_known()) {
    navmap_ros_interfaces::msg::NavMapLayer l;
    l.header = msg.header;
    l.name = "traversed";
    l.type = navmap_ros_interfaces::msg::NavMapLayer::U8;
    l.description = "255: ground the robot has driven over (high confidence)";
    l.data_u8.resize(kept.size());
    for (std::size_t k = 0; k < kept.size(); ++k) {
      l.data_u8[k] = is_traversed(mesh_, kept[k]) ? 255 : 0;
    }
    msg.layers.push_back(std::move(l));
  }
  for (const char * lname : {"obstacles", "inflated_obstacles", "obstacle_hits"}) {
    const auto * data = u8(lname);
    if (!data) {continue;}
    navmap_ros_interfaces::msg::NavMapLayer l;
    l.header = msg.header;
    l.name = lname;
    l.type = navmap_ros_interfaces::msg::NavMapLayer::U8;
    l.description = std::string(lname) == "obstacle_hits" ?
      "times an obstacle was detected here (max_hits or more: fixed), snapshot when the map last "
      "changed" : "cost layer (254 = lethal), snapshot when the map last changed";
    l.data_u8.resize(kept.size());
    for (std::size_t k = 0; k < kept.size(); ++k) {
      l.data_u8[k] = (*data)[kept[k]];
    }
    msg.layers.push_back(std::move(l));
  }
  return msg;
}

void BigMapMapsManager::publish_map(
  const std::vector<char> & removed, const ::navmap::NavMap & layers_from)
{
  if (!navmap_pub_) {return;}
  const auto msg = make_map_msg(removed, layers_from, get_node()->now());
  if (tint_enabled_) {
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "[%s] obstacle NavCels painted: %zu of %zu", get_plugin_name().c_str(),
      (msg.positions_x.size() - mesh_.x.size()) / 3, msg.navcels_v0.size());
  }
  navmap_pub_->publish(msg);
}

void BigMapMapsManager::restore_removed(::easynav::NavState & nav_state)
{
  if (removed_.empty()) {return;}
  const auto map = nav_state.get_ptr<::navmap::NavMap>("map.navmap");
  auto & nm = *map;
  const auto occ = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    nm.layers.get("occupancy"));
  if (!nm.layers.get("obstacles") || !occ || occ->size() != nm.navcels.size()) {return;}
  for (const auto c : removed_) {
    nm.layer_set<std::uint8_t>(
      "obstacles", static_cast<::navmap::NavCelId>(c), occ->data()[c]);
  }
}

void BigMapMapsManager::apply_removal(
  ::easynav::NavState & nav_state, const std::vector<std::uint32_t> & seeds)
{
  const auto map = nav_state.get_ptr<::navmap::NavMap>("map.navmap");
  auto & nm = *map;
  const auto obs = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    nm.layers.get("obstacles"));
  const auto occ = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    nm.layers.get("occupancy"));
  const std::size_t n = nm.navcels.size();
  if (!obs || !occ || obs->size() != n || centroid_x_.size() != n) {return;}
  const auto & base = occ->data();

  // The NavCels with an obstacle and all those whose centre is within the radius of one of them.
  // With the obstacle memory on, each of them also shows in `obstacle_hits` the count of the
  // obstacle it belongs to (a red NavCel is an obstacle: it must have its place in that layer)
  const bool hits_on = memory_ && memory_was_on_ && nm.layers.get("obstacle_hits");
  std::vector<std::uint8_t> ring_hits(hits_on ? n : 0, 0);
  auto hits_of = [&](std::uint32_t c) {
      return nm.layer_get<std::uint8_t>("obstacle_hits", static_cast<::navmap::NavCelId>(c), 0);
    };
  std::vector<char> in_removed(n, 0);
  for (const auto s : seeds) {
    in_removed[s] = 1;
    if (hits_on) {ring_hits[s] = std::max(ring_hits[s], hits_of(s));}
  }
  if (removal_radius_ > 0.0 && !seeds.empty()) {
    // the centroids within the radius, through the grid of centroids built with the mesh
    // (cells >= the radius: the 3 x 3 cells around a seed hold all of them)
    const float r2 = static_cast<float>(removal_radius_ * removal_radius_);
    for (const auto s : seeds) {
      const auto sx = static_cast<std::int64_t>(std::floor(centroid_x_[s] / centroid_cell_));
      const auto sy = static_cast<std::int64_t>(std::floor(centroid_y_[s] / centroid_cell_));
      for (std::int64_t j = sy - 1; j <= sy + 1; ++j) {
        for (std::int64_t i = sx - 1; i <= sx + 1; ++i) {
          const auto it = centroid_grid_.find(grid_key(i, j));
          if (it == centroid_grid_.end()) {continue;}
          for (const auto c : it->second) {
            const float dx = centroid_x_[c] - centroid_x_[s], dy = centroid_y_[c] - centroid_y_[s];
            // another floor over the obstacle is not next to it
            if (dx * dx + dy * dy <= r2 && std::abs(centroid_z_[c] - centroid_z_[s]) <= 1.0f) {
              in_removed[c] = 1;
              if (hits_on) {ring_hits[c] = std::max(ring_hits[c], hits_of(s));}
            }
          }
        }
      }
    }
  }

  std::vector<std::uint32_t> now_removed;
  for (std::size_t c = 0; c < n; ++c) {
    if (in_removed[c]) {now_removed.push_back(static_cast<std::uint32_t>(c));}
  }
  // released NavCels get their base value back; the removed ones are blocked for the planner
  for (const auto c : removed_) {
    if (!in_removed[c]) {
      nm.layer_set<std::uint8_t>("obstacles", static_cast<::navmap::NavCelId>(c), base[c]);
      // no longer next to an obstacle: its count goes (steep ground keeps its own; the memory's
      // cells are written back by write_hits)
      if (hits_on && base[c] != ::navmap::LETHAL_OBSTACLE && hits_of(c) > 0) {
        nm.layer_set<std::uint8_t>("obstacle_hits", static_cast<::navmap::NavCelId>(c), 0);
      }
    }
  }
  if (hits_on) {
    for (const auto c : now_removed) {
      if (ring_hits[c] > hits_of(c)) {
        nm.layer_set<std::uint8_t>(
          "obstacle_hits", static_cast<::navmap::NavCelId>(c), ring_hits[c]);
      }
    }
  }
  const auto & o = obs->data();
  for (const auto c : now_removed) {
    if (o[c] != ::navmap::LETHAL_OBSTACLE) {
      nm.layer_set<std::uint8_t>(
        "obstacles", static_cast<::navmap::NavCelId>(c), ::navmap::LETHAL_OBSTACLE);
    }
  }
  if (now_removed != removed_) {
    removed_ = now_removed;
    // published at the end of update(), when the inflation of this very cycle is done: the map
    // carries its layers, and published here it would show the previous cycle's inflated_obstacles
    pending_flags_ = in_removed;
    publish_pending_ = true;
  }
}

void BigMapMapsManager::apply_obstacle_removal(::easynav::NavState & nav_state)
{
  // obstacle_source: filter -> the NavCels the ObstaclesFilter marked (not the terrain's own)
  const auto map = nav_state.get_ptr<::navmap::NavMap>("map.navmap");
  auto & nm = *map;
  const auto obs = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    nm.layers.get("obstacles"));
  const auto occ = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    nm.layers.get("occupancy"));
  const std::size_t n = nm.navcels.size();
  if (!obs || !occ || obs->size() != n) {return;}
  std::vector<std::uint32_t> seeds;
  for (std::size_t c = 0; c < n; ++c) {
    if (obs->data()[c] == ::navmap::LETHAL_OBSTACLE &&
      occ->data()[c] != ::navmap::LETHAL_OBSTACLE)
    {
      seeds.push_back(static_cast<std::uint32_t>(c));
    }
  }
  apply_removal(nav_state, seeds);
}

navmap_ros_interfaces::msg::NavMap BigMapMapsManager::make_dynamic_msg(
  const std::vector<bigmap_maps_manager::RaisedTile> & tiles, const rclcpp::Time & stamp) const
{
  navmap_ros_interfaces::msg::NavMap msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = stamp;
  const bool coloured = !mesh_.r.empty();
  msg.has_vertex_rgba = coloured;
  navmap_ros_interfaces::msg::NavMapSurface surf;
  surf.frame_id = map_frame_;
  navmap_ros_interfaces::msg::NavMapLayer elevation, height, size;
  for (auto * l : {&elevation, &height, &size}) {
    l->header = msg.header;
    l->type = navmap_ros_interfaces::msg::NavMapLayer::F32;
    l->unit = "m";
  }
  elevation.name = "elevation";
  height.name = "obstacle_height";
  height.description = "height of the NavCel above the terrain, as measured by the sensors";
  size.name = "lod_size";
  for (std::size_t k = 0; k < tiles.size(); ++k) {
    const auto & t = tiles[k];
    // own vertices (not shared with the terrain): the tile is detached and raised as a whole
    for (int c = 0; c < 3; ++c) {
      const auto v = mesh_.tri[3 * t.triangle + c];
      msg.positions_x.push_back(mesh_.x[v]);
      msg.positions_y.push_back(mesh_.y[v]);
      msg.positions_z.push_back(t.z_top);
      if (coloured) {
        msg.colors_r.push_back(mesh_.r[v]);
        msg.colors_g.push_back(mesh_.g[v]);
        msg.colors_b.push_back(mesh_.b[v]);
        msg.colors_a.push_back(255);
      }
    }
    const auto base = static_cast<std::uint32_t>(3 * k);
    msg.navcels_v0.push_back(base);
    msg.navcels_v1.push_back(base + 1);
    msg.navcels_v2.push_back(base + 2);
    surf.navcels.push_back(static_cast<std::uint32_t>(k));
    elevation.data_f32.push_back(t.z_top);
    height.data_f32.push_back(t.above_surface);
    size.data_f32.push_back(mesh_.leg[t.triangle]);
  }
  msg.surfaces.push_back(std::move(surf));
  msg.layers = {elevation, height, size};
  return msg;
}

void BigMapMapsManager::update_dynamic_map(
  ::easynav::NavState & nav_state, double x, double y, double z,
  const std::array<double, 4> & q)
{
  const double now = get_node()->now().seconds();
  // Multi-level: there is no single height under an (x, y); the NavCels (picked by the robot's
  // height) are the reference, and the robot's own z only backs up a miss
  const bigmap_maps_manager::DynamicLayer::Surface terrain = surface_ ?
    bigmap_maps_manager::DynamicLayer::Surface([z](double, double) {return z;}) :
    bigmap_maps_manager::DynamicLayer::Surface(
    [dem = &mesher_->dem()](double px, double py) {return dem->height_at(px, py);});

  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  if (!perceptions.empty()) {
    try {
      const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
      const float r = static_cast<float>(dyn_.max_range);
      // in the robot's footprint frame (z = 0 at the ground contact): the heights are measured
      // relative to the robot, not through the map -> robot TF, whose z is not reliable
      std::vector<std::array<float, 3>> pts;
      {
        bigmap_maps_manager::ProfScope s(prof(), "dyn: sensor clouds (total)");
        // Cut to the box around the robot FIRST and thin the survivors afterwards: the voxel
        // filter hashes every point it is given, and a depth camera gives hundreds of thousands,
        // nearly all of them outside the box (that order took ~47 ms per cycle, the cut is cheap)
        PointPerceptionsOpsView view(perceptions);
        {
          bigmap_maps_manager::ProfScope sf(prof(), "dyn:   fuse (TF lookups)");
          view.fuse(tf_info.robot_footprint_frame);
        }
        {
          bigmap_maps_manager::ProfScope sf(prof(), "dyn:   filter (cut to the box)");
          view.filter({-r, -r, NAN}, {r, r, static_cast<float>(dyn_.max_height)}, false);
        }
        {
          bigmap_maps_manager::ProfScope sf(prof(), "dyn:   downsample (voxel)");
          view.downsample(dyn_.voxel);
        }
        pcl::PointCloud<pcl::PointXYZ> cloud;
        {
          bigmap_maps_manager::ProfScope sf(prof(), "dyn:   as_points");
          cloud = view.as_points();
        }
        pts.reserve(cloud.points.size());
        for (const auto & p : cloud.points) {
          pts.push_back({p.x, p.y, p.z});
        }
      }
      bigmap_maps_manager::ProfScope s_ingest(prof(), "dyn: ingest (cells, plane fit)");
      if (locator_ &&
        dyn_layer_.ingest(pts, x, y, q, now, terrain, dyn_, &mesh_, locator_.get(), z))
      {
        dyn_dirty_ = true;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "[%s] dynamic map: %s", get_plugin_name().c_str(), e.what());
    }
  }
  {
    bigmap_maps_manager::ProfScope s(prof(), "dyn: expire");
    if (dyn_layer_.expire(now, dyn_)) {dyn_dirty_ = true;}
  }

  // Memory of obstacles: every min_time s a detection goes on over a spot is a hit there
  const bool memory_on = memory_active();
  if (clear_obstacles_requested_.exchange(false) && memory_) {
    const std::size_t n = memory_->cells().size(), fixed = memory_->fixed_count();
    memory_->clear();
    if (nav_state.has("map.navmap")) {
      reset_hits(*nav_state.get_ptr<::navmap::NavMap>("map.navmap"));
    }
    dyn_dirty_ = true;   // the fixed obstacles are released below
    RCLCPP_INFO(
      get_node()->get_logger(), "[%s] obstacle memory cleared: %zu cells (%zu fixed) forgotten",
      get_plugin_name().c_str(), n, fixed);
  }
  if (memory_on) {
    const auto hits = dyn_layer_.sustained(now, memory_min_time_);
    if (!hits.empty()) {
      memory_->add(hits, now);
      dyn_dirty_ = true;   // a new obstacle (5 s over a NavCel) or a new hit
    }
  }
  if (memory_on != memory_was_on_) {
    // switched off: the counts stay in the memory, but the layer stops showing them (they no
    // longer block) and the fixed obstacles are released below (dirty)
    if (!memory_on && nav_state.has("map.navmap")) {
      reset_hits(*nav_state.get_ptr<::navmap::NavMap>("map.navmap"));
    }
    memory_was_on_ = memory_on;
    dyn_dirty_ = true;
  }

  // at most 5 Hz, and only when something changed
  if (!dyn_dirty_ || !locator_ || now - last_dyn_publish_ < 0.2) {return;}
  dyn_dirty_ = false;
  last_dyn_publish_ = now;
  // every confirmed detection: the dynamic map, and the obstacles when the memory is off
  const bool want_tiles = dynamic_enabled_ || !memory_on;
  const auto tiles = want_tiles ? dyn_layer_.tiles(mesh_, *locator_) :
    std::vector<bigmap_maps_manager::RaisedTile>{};
  if (removal_enabled_ && !obstacles_from_filter_) {
    bigmap_maps_manager::ProfScope s(prof(), "dyn: remove NavCels (apply_removal)");
    // what the sensors measured above the terrain (relative to the robot: no TF z involved)
    // is what is removed from the map and blocked for the planner. With the memory on, only what
    // has been over a NavCel for min_time s counts, and fixed obstacles (max_hits) stay for good
    std::vector<std::uint32_t> seeds;
    if (memory_on) {
      if (nav_state.has("map.navmap")) {
        write_hits(*nav_state.get_ptr<::navmap::NavMap>("map.navmap"));
      }
      for (const auto & t : dyn_layer_.tiles(mesh_, *locator_, memory_min_time_)) {
        seeds.push_back(t.triangle);
      }
      fixed_seeds(seeds);
      std::sort(seeds.begin(), seeds.end());
      seeds.erase(std::unique(seeds.begin(), seeds.end()), seeds.end());
    } else {
      for (const auto & t : tiles) {
        seeds.push_back(t.triangle);
      }
    }
    apply_removal(nav_state, seeds);
  }
  if (memory_on && nav_state.has("map.navmap")) {
    write_hits(*nav_state.get_ptr<::navmap::NavMap>("map.navmap"));
  }
  if (!dynamic_enabled_) {return;}
  bigmap_maps_manager::ProfScope s(prof(), "dyn: publish dynamic map");
  auto msg = make_dynamic_msg(tiles, get_node()->now());
  dynamic_pub_->publish(msg);
  nav_state.set(
    "map.dynamic", tiles.empty() ? ::navmap::NavMap() : navmap_ros::from_msg(msg));
}

void BigMapMapsManager::release_sensor_obstacles(::easynav::NavState & nav_state)
{
  // Nothing analyses the sensors (dynamic_map and memory both off, e.g. a recovery switched the
  // memory off): what they blocked, fixed obstacles included, is released at once
  if (memory_was_on_) {
    reset_hits(*nav_state.get_ptr<::navmap::NavMap>("map.navmap"));
    memory_was_on_ = false;
  }
  if (!removed_.empty() && removal_enabled_ && !obstacles_from_filter_) {
    apply_removal(nav_state, {});
  }
}

void BigMapMapsManager::process_save(::easynav::NavState & nav_state)
{
  std::shared_ptr<SaveRequest> req;
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    req = pending_save_;
    pending_save_.reset();
  }
  if (!req) {return;}
  try {
    if (mesh_.triangle_count() == 0) {throw std::runtime_error("no map yet");}
    // the layers (obstacles...) as the NavState holds them now
    const auto nav_map = nav_state.has("map.navmap") ?
      nav_state.get_ptr<::navmap::NavMap>("map.navmap") : nullptr;
    const ::navmap::NavMap & layers = nav_map ? *nav_map : navmap_;
    std::vector<char> removed;
    if (req->traversed_only) {
      removed.assign(mesh_.triangle_count(), 1);
      std::size_t n = 0;
      for (std::size_t t = 0; t < mesh_.triangle_count(); ++t) {
        if (is_traversed(mesh_, t)) {removed[t] = 0; ++n;}
      }
      if (n == 0) {throw std::runtime_error("the robot has not driven over any ground yet");}
    }
    // untinted: the saved colours are the imagery's
    const auto msg = make_map_msg(removed, layers, get_node()->now(), false);
    std::filesystem::create_directories(save_directory_);
    const std::string path = req->path;
    if (!navmap_ros::io::save_to_file(navmap_ros::from_msg(msg), path)) {
      throw std::runtime_error("could not write " + path);
    }
    bigmap_maps_manager::save_origin(path, origin_, params_.min_size);
    RCLCPP_INFO(
      get_node()->get_logger(), "[%s] saved %zu NavCels to %s", get_plugin_name().c_str(),
      msg.navcels_v0.size(), path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "[%s] could not save %s: %s", get_plugin_name().c_str(),
      req->path.c_str(), e.what());
  }
}

void BigMapMapsManager::mark_trail(double x, double y, double z)
{
  if (surface_) {
    surface_->mark(x, y, z, trail_mark_radius_);
  } else if (known_) {
    known_->mark(x, y, trail_mark_radius_);
  }
}

void BigMapMapsManager::mark_observed(double x, double y, double z)
{
  if (!observed_enabled_) {return;}
  if (surface_) {
    surface_->mark_observed(x, y, z, observed_mark_radius_);
    return;
  }
  if (!observed_) {return;}
  // the nodes on the border of a hole of the map (some of their cells have ground, some not)
  // within the radius
  const auto & cv = mesher_->cell_valid();
  const auto & dem = mesher_->dem();
  const int side = dem.side, nc = side - 1, center = (side - 1) / 2;
  const double res = dem.resolution;
  const int reach = static_cast<int>(std::ceil(observed_mark_radius_ / res));
  const int cx = static_cast<int>(std::lround(x / res)) + center;
  const int cy = static_cast<int>(std::lround(y / res)) + center;
  for (int j = std::max(0, cy - reach); j <= std::min(side - 1, cy + reach); ++j) {
    for (int i = std::max(0, cx - reach); i <= std::min(side - 1, cx + reach); ++i) {
      if (std::hypot((i - center) * res - x, (j - center) * res - y) > observed_mark_radius_) {
        continue;
      }
      bool some = false, none = false;
      for (int b = j - 1; b <= j; ++b) {
        for (int a = i - 1; a <= i; ++a) {
          if (a < 0 || b < 0 || a >= nc || b >= nc) {continue;}
          (cv[static_cast<std::size_t>(b) * nc + a] ? some : none) = true;
        }
      }
      if (some && none) {observed_->mark_node(i, j);}
    }
  }
}

void BigMapMapsManager::prepare_remesh(double x, double y, double z)
{
  mark_observed(x, y, z);
  if (surface_) {
    if (trail_enabled_ && !trail_persistent_) {
      surface_->forget_outside(x, y, z, trail_keep_radius_);
    }
    if (!trail_persistent_) {surface_->forget_observed_outside(x, y, z, trail_keep_radius_);}
    return;
  }
  if (trail_enabled_ && known_) {
    bigmap_maps_manager::ProfScope s(prof(), "remesh: traversed ground (tree of nodes)");
    if (!trail_persistent_) {
      // beyond the large circle the ground driven over is forgotten (as resolution and as the
      // `traversed` layer) until the robot drives over it again
      known_->forget_outside(x, y, trail_keep_radius_);
    }
    known_->update();
  }
  if (observed_) {
    if (!trail_persistent_) {observed_->forget_outside(x, y, trail_keep_radius_);}
    observed_->update();
  }
}

BigMapMapsManager::Remesh BigMapMapsManager::make_remesh(
  double x, double y, double z, const bigmap_maps_manager::LodParams & params,
  bigmap_maps_manager::Profiler * prof) const
{
  Remesh r;
  r.t0 = std::chrono::steady_clock::now();
  r.x = x;
  r.y = y;
  r.z = z;
  {
    bigmap_maps_manager::ProfScope s(prof, "remesh: extract (RTIN)");
    r.mesh = surface_ ? surface_->extract(x, y, z, params) :
      mesher_->extract(x, y, params, trail_enabled_ ? known_.get() : nullptr, observed_.get());
  }
  {
    bigmap_maps_manager::ProfScope s(prof, "remesh: build NavMap");
    r.navmap = build_navmap(r.mesh);
  }
  return r;
}

void BigMapMapsManager::check_recovery(const ::easynav::NavState & nav_state)
{
  // With a goal and no path for recovery_cycles_ cycles the map may be too coarse where the way
  // is (a door, a gap between obstacles merged into one): the full-resolution circle is doubled,
  // up to recovery_max_factor_ times its value, and the map re-meshed. Back to normal when there
  // is no goal any more (reached or dropped): changing it while a path exists could lose it.
  const bool has_goal = nav_state.has("goals") &&
    !nav_state.get<nav_msgs::msg::Goals>("goals").goals.empty();
  const bool has_path = nav_state.has("path") &&
    !nav_state.get<nav_msgs::msg::Path>("path").poses.empty();
  if (!has_goal) {
    no_path_cycles_ = 0;
    if (params_.high_res_radius != base_high_res_) {
      params_.high_res_radius = base_high_res_;
      params_.low_res_distance = base_low_res_;
      force_remesh_ = true;
      RCLCPP_INFO(
        get_node()->get_logger(), "[%s] recovery over: full resolution back to %.1f m",
        get_plugin_name().c_str(), base_high_res_);
    }
    return;
  }
  if (has_path) {no_path_cycles_ = 0; return;}
  if (++no_path_cycles_ < recovery_cycles_) {return;}
  no_path_cycles_ = 0;
  const double cap = base_high_res_ * recovery_max_factor_;
  if (params_.high_res_radius >= cap) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "[%s] no path to the goal even at full resolution within %.1f m", get_plugin_name().c_str(),
      params_.high_res_radius);
    return;
  }
  params_.high_res_radius = std::min(cap, std::max(params_.high_res_radius * 2.0, 1.0));
  params_.low_res_distance = std::max(base_low_res_, params_.high_res_radius);
  force_remesh_ = true;
  RCLCPP_WARN(
    get_node()->get_logger(), "[%s] no path to the goal for %d cycles: full resolution within "
    "%.1f m (recovery)", get_plugin_name().c_str(), recovery_cycles_, params_.high_res_radius);
}

void BigMapMapsManager::commit_remesh(Remesh && r)
{
  const double x = r.x, y = r.y, z = r.z;
  const auto t0 = r.t0;
  auto & new_mesh = r.mesh;
  auto & new_navmap = r.navmap;
  // the locator (which points into mesh_) is dropped before and rebuilt after
  locator_.reset();
  mesh_ = std::move(new_mesh);
  navmap_ = std::move(new_navmap);
  const auto & mesh = mesh_;
  map_changed_ = true;

  // NavCel ids change with every mesh: the previous removals do not apply any more
  removed_.clear();
  publish_pending_ = false;
  {
    bigmap_maps_manager::ProfScope s(prof(), "remesh: locator + centroids");
    locator_ = std::make_unique<bigmap_maps_manager::TriangleLocator>(mesh_);
    dyn_dirty_ = true;  // NavCel ids changed: the raised ones must be found again
    last_dyn_publish_ = -1e9;   // and at once, not after the 5 Hz throttle
    centroid_x_.resize(mesh.triangle_count());
    centroid_y_.resize(mesh.triangle_count());
    centroid_z_.resize(mesh.triangle_count());
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
      const auto a = mesh.tri[3 * t], b = mesh.tri[3 * t + 1], c = mesh.tri[3 * t + 2];
      centroid_x_[t] = (mesh.x[a] + mesh.x[b] + mesh.x[c]) / 3.0f;
      centroid_y_[t] = (mesh.y[a] + mesh.y[b] + mesh.y[c]) / 3.0f;
      centroid_z_[t] = (mesh.z[a] + mesh.z[b] + mesh.z[c]) / 3.0f;
    }
    // grid of the centroids for the clearance ring of apply_removal
    centroid_cell_ = std::max(removal_radius_, 0.5);
    centroid_grid_.clear();
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
      centroid_grid_[grid_key(
          static_cast<std::int64_t>(std::floor(centroid_x_[t] / centroid_cell_)),
          static_cast<std::int64_t>(std::floor(centroid_y_[t] / centroid_cell_)))].push_back(
        static_cast<std::uint32_t>(t));
    }
    if (memory_active()) {
      write_hits(navmap_);   // what was detected before this mesh, on its NavCels
    }
  }

  const auto stamp = get_node()->now();
  {
    bigmap_maps_manager::ProfScope s(prof(), "remesh: publish map");
    publish_map({}, navmap_);
  }
  if (publish_markers_) {
    bigmap_maps_manager::ProfScope s(prof(), "remesh: markers");
    markers_pub_->publish(
      bigmap_maps_manager::make_markers(
        mesh, params_, x, y, z, params_.max_size, map_frame_, stamp,
        trail_enabled_ && !trail_persistent_ ? trail_keep_radius_ : 0.0));
  }
  std::size_t traversed = 0;
  {
    bigmap_maps_manager::ProfScope s(prof(), "remesh: count traversed (log)");
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
      traversed += is_traversed(mesh, t);
    }
  }
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 5000,
    "[%s] LOD @ (%.1f, %.1f): %zu triangles (%zu on traversed ground) in %.2f ms",
    get_plugin_name().c_str(), x, y, mesh.triangle_count(), traversed,
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
}

void BigMapMapsManager::update(::easynav::NavState & nav_state)
{
  watchdog_.ran(wall_seconds());
  bigmap_maps_manager::ProfScope total_scope(prof(), "update (total)");
  double x = 0.0, y = 0.0, z = 0.0;
  std::array<double, 4> q = {0.0, 0.0, 0.0, 1.0};
  bool have_robot = false;
  {
    bigmap_maps_manager::ProfScope s(prof(), "pose + mark");
    have_robot = robot_position(nav_state, x, y, z, q);
    if (have_robot && trail_enabled_) {
      if (remesh_job_.valid()) {
        pending_marks_.push_back({x, y, z});   // the worker reads the ground: marked after it
      } else {
        mark_trail(x, y, z);
      }
    }
  }

  if (!have_robot) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "[%s] no robot pose yet (neither 'robot_pose' from the localizer nor TF map -> base_link): "
      "the map is not published until there is one", get_plugin_name().c_str());
  }
  if (recovery_enabled_) {check_recovery(nav_state);}

  // A re-mesh the worker finished is committed first
  if (remesh_job_.valid() &&
    remesh_job_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    try {
      commit_remesh(remesh_job_.get());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "[%s] re-mesh failed, the previous map stays: %s",
        get_plugin_name().c_str(), e.what());
    }
    for (const auto & m : pending_marks_) {
      mark_trail(m[0], m[1], m[2]);
    }
    pending_marks_.clear();
  }

  // Re-mesh on the first valid pose, then whenever the robot moved enough (one at a time)
  if (have_robot && !remesh_job_.valid() &&
    (force_remesh_ || !last_meshed_ ||
    std::hypot(x - last_meshed_->first, y - last_meshed_->second) >= hysteresis_ ||
    (surface_ && std::abs(z - last_meshed_z_) >= 1.0)))
  {
    force_remesh_ = false;
    bigmap_maps_manager::ProfScope remesh_scope(prof(), "REMESH (total)");
    prepare_remesh(x, y, z);
    last_meshed_ = {x, y};
    last_meshed_z_ = z;
    if (async_remesh_ && navmap_.navcels.size() > 0) {
      // (the first mesh is made here: there is nothing to show until it exists)
      remesh_job_ = std::async(
        std::launch::async, [this, x, y, z, params = params_]() {
          return make_remesh(x, y, z, params, nullptr);
        });
    } else {
      // built aside and committed together: an exception while building leaves the previous
      // mesh and NavMap
      commit_remesh(make_remesh(x, y, z, params_, prof()));
    }
  }

  // (no map before the first pose: an empty NavMap would only mislead the filters and the planner)
  if (map_changed_ || (!nav_state.has("map.navmap") && last_meshed_)) {
    bigmap_maps_manager::ProfScope s(prof(), "set map.navmap (copy)");
    nav_state.set("map.navmap", navmap_);
    map_changed_ = false;
  }

  process_save(nav_state);

  {
    // view_max_z changed at runtime (ros2 param set): the map RViz shows is published again
    double wanted = view_max_z_;
    const std::string vkey = get_plugin_name() + (
      get_node()->has_parameter(get_plugin_name() + ".output.view_max_z") ?
      ".output.view_max_z" : ".view_max_z");
    get_node()->get_parameter(vkey, wanted);
    if (wanted != view_max_z_ && nav_state.has("map.navmap")) {
      view_max_z_ = wanted;
      std::vector<char> flags(mesh_.triangle_count(), 0);
      for (const auto c : removed_) {
        if (c < flags.size()) {flags[c] = 1;}
      }
      publish_map(flags, nav_state.get<::navmap::NavMap>("map.navmap"));
      RCLCPP_INFO(
        get_node()->get_logger(), "[%s] RViz shows the map up to z = %.2f m",
        get_plugin_name().c_str(), view_max_z_);
    }
  }

  if (!nav_state.has("map.navmap")) {return;}

  // Sensor obstacles BEFORE the filters, so that the inflation (what A* reads) is built on this
  // cycle's obstacles, also right after a re-mesh, when the NavState map has just been replaced
  if (have_robot) {
    if (dynamic_enabled_ || memory_active()) {
      bigmap_maps_manager::ProfScope s(prof(), "DYNAMIC MAP (total)");
      update_dynamic_map(nav_state, x, y, z, q);
    } else {
      release_sensor_obstacles(nav_state);
    }
  }

  // Filters edit the NavMap held by the NavState (obstacles seen by the sensors, inflation)
  // With removal the published map has holes, so per-NavCel layer updates (indexed on the
  // complete mesh) would not match it: the layers travel inside the map message instead
  const std::size_t subscribers =
    removal_enabled_ ? 0 : layer_updates_pub_->get_subscription_count();
  const bool new_subscriber = subscribers > last_subscribers_;
  last_subscribers_ = subscribers;
  std_msgs::msg::Header header;
  header.frame_id = map_frame_;
  header.stamp = get_node()->now();
  for (const auto & filter : filters_) {
    bigmap_maps_manager::ProfScope filter_scope(prof(), "filters");
    filter->set_map_resolution(map_resolution_);
    // Take back our own marks first, so that the filter (and our seeds) only see the sensors',
    // not the ring we blocked in the previous cycle
    if (removal_enabled_ && obstacles_from_filter_ && filter->get_layer_name() == "obstacles") {
      restore_removed(nav_state);
    }
    filter->update(nav_state);
    if (removal_enabled_ && obstacles_from_filter_ && filter->get_layer_name() == "obstacles") {
      apply_obstacle_removal(nav_state);  // before the inflation filter, which builds on it
    }
    if (subscribers == 0 || !filter->is_adding_layer()) {continue;}
    const auto & nm = nav_state.get<::navmap::NavMap>("map.navmap");
    const auto layer = nm.layers.get(filter->get_layer_name());
    if (!layer || (!layer->has_changes() && !new_subscriber)) {continue;}
    layer_updates_pub_->publish(
      navmap_ros::to_layer_update_msg(
        nm, filter->get_layer_name(), header, new_subscriber ? 0.0f : 0.25f));
  }
  const auto & nm = nav_state.get<::navmap::NavMap>("map.navmap");
  if (publish_pending_) {
    publish_pending_ = false;
    publish_map(pending_flags_, nm);
  }
  for (const auto & lname : nm.list_layers()) {
    nm.layers.get(lname)->clear_changes();
  }
  if (profile_enabled_ && profiler_.due()) {
    RCLCPP_INFO(
      get_node()->get_logger(), "[%s] %s", get_plugin_name().c_str(),
      profiler_.report_and_reset("timing of update()").c_str());
  }
}

}  // namespace bigmap
}  // namespace easynav

PLUGINLIB_EXPORT_CLASS(easynav::bigmap::BigMapMapsManager, easynav::MapsManagerBase)
