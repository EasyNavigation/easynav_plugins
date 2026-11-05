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

/// \file
/// \brief Implementation of the AStarPlanner class using A* on ::navmap::NavMap (triangle graph).

#include <array>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdint>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_navmap_planner/AStarPlanner.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navmap_core/Descriptors.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

namespace easynav
{
namespace navmap
{

// How far from the start (m) the inscribed band is still traversable: the robot may be in it.
constexpr double kEscapeDistance = 1.0;
// How far (m) the robot may be from the path it follows for it to be kept (replan hysteresis).
constexpr double kMaxPathOffset = 2.0;
// A point is on a surface if the surface is within this height (m) of it: floors over each other.
constexpr float kSurfaceHeightTolerance = 0.5f;

static double compute_path_length(const nav_msgs::msg::Path & path)
{
  double total_length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;
    total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  return total_length;
}

AStarPlanner::AStarPlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;
      ret << "{ " << rclcpp::Time(path.header.stamp).seconds() << " } Path with " <<
        path.poses.size() << " poses and length "
          << compute_path_length(path) << " m.";
      return ret.str();
    });
}

void AStarPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".cost_factor", 2.0);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".cost_weight", 5.0);
  easynav::declare_parameter_if_absent<bool>(*node, plugin_name + ".continuous_replan", true);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".replan_hysteresis", replan_hysteresis_);
  easynav::declare_parameter_if_absent<double>(
    *node, plugin_name + ".replan_hysteresis_horizon", replan_hysteresis_horizon_);

  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  // Weighted A*: the heuristic is multiplied by this weight (>= 1). 1 keeps A* admissible (optimal
  // paths); a larger weight expands fewer NavCels and may return paths up to that factor longer.
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".heuristic_weight", 1.0);
  node->get_parameter(plugin_name + ".heuristic_weight", heuristic_weight_);
  if (!(heuristic_weight_ >= 1.0)) {heuristic_weight_ = 1.0;}
  node->get_parameter(plugin_name + ".cost_weight", cost_weight_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);
  node->get_parameter(plugin_name + ".replan_hysteresis", replan_hysteresis_);
  if (!(replan_hysteresis_ >= 0.0)) {replan_hysteresis_ = 0.0;}
  node->get_parameter(
    plugin_name + ".replan_hysteresis_horizon", replan_hysteresis_horizon_);
  if (!(replan_hysteresis_horizon_ >= 0.0)) {replan_hysteresis_horizon_ = 0.0;}
  followed_.clear();

  // Terrain (off by default)
  const std::string t = plugin_name + ".terrain.";
  easynav::declare_parameter_if_absent<double>(*node, t + "slope_weight", slope_weight_);
  easynav::declare_parameter_if_absent<double>(*node, t + "max_slope_deg", max_slope_deg_);
  easynav::declare_parameter_if_absent<double>(*node, t + "max_step", max_step_);
  easynav::declare_parameter_if_absent<double>(*node, t + "max_roughness", max_roughness_);
  easynav::declare_parameter_if_absent<double>(*node, t + "min_clearance", min_clearance_);
  node->get_parameter(t + "slope_weight", slope_weight_);
  node->get_parameter(t + "max_slope_deg", max_slope_deg_);
  node->get_parameter(t + "max_step", max_step_);
  node->get_parameter(t + "max_roughness", max_roughness_);
  node->get_parameter(t + "min_clearance", min_clearance_);
  if (!(slope_weight_ >= 0.0)) {slope_weight_ = 0.0;}
  if (!(max_slope_deg_ > 0.0)) {max_slope_deg_ = 90.0;}

  // Links between surfaces: which kinds the robot can take
  std::vector<std::string> link_types{"generic", "ramp", "door"};
  easynav::declare_parameter_if_absent<bool>(*node, plugin_name + ".links.enabled", use_links_);
  easynav::declare_parameter_if_absent(*node, plugin_name + ".links.types", link_types);
  node->get_parameter(plugin_name + ".links.enabled", use_links_);
  node->get_parameter(plugin_name + ".links.types", link_types);
  const std::vector<std::pair<std::string, ::navmap::LinkType>> kinds{
    {"generic", ::navmap::LinkType::GENERIC}, {"ramp", ::navmap::LinkType::RAMP},
    {"stairs", ::navmap::LinkType::STAIRS}, {"elevator", ::navmap::LinkType::ELEVATOR},
    {"door", ::navmap::LinkType::DOOR}};
  link_type_ok_.assign(kinds.size(), 0);
  for (const auto & name : link_types) {
    const auto it = std::find_if(
      kinds.begin(), kinds.end(), [&](const auto & k) {return k.first == name;});
    if (it == kinds.end()) {
      RCLCPP_WARN(
        node->get_logger(), "[%s] unknown link type '%s' (generic, ramp, stairs, elevator, door)",
        plugin_name.c_str(), name.c_str());
      continue;
    }
    link_type_ok_[static_cast<std::size_t>(it->second)] = 1;
  }
  geometry_id_ = 0;  // new limits: the terrain cache is rebuilt on the next plan
  centroids_.clear();

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/path", 10);
}

void AStarPlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map.navmap")) {
    return;
  }

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty() || !nav_state.has("map.navmap")) {
    clear_path(nav_state);
    return;
  }

  const auto & navmap = nav_state.get<::navmap::NavMap>("map.navmap");

  const auto & robot_pose = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  if (goals.header.frame_id != tf_info.map_frame) {
    RCLCPP_WARN(
      get_node()->get_logger(), "Goals frame is not 'map': %s",
      goals.header.frame_id.c_str());
    clear_path(nav_state);
    return;
  }

  auto goals_ts = rclcpp::Time(goals.header.stamp);
  if (!continuous_replan_ &&
    goals_ts < rclcpp::Time(current_path_.header.stamp) &&
    goals.goals.front().pose == current_goal_)
  {
    return;
  }

  current_goal_ = goal;
  auto poses = choose_path(
    navmap, robot_pose.pose.pose, goal, a_star_path(navmap, robot_pose.pose.pose, goal));
  if (!poses.empty()) {
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;
    current_path_.poses.reserve(poses.size());
    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = goals.header.frame_id;
      ps.header.stamp = current_path_.header.stamp;
      ps.pose = pose;
      current_path_.poses.push_back(std::move(ps));
    }

    current_path_ = path_smoother(current_path_, navmap);

    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
      path_published_ = true;
    }
    nav_state.set("path", current_path_);
  } else {
    // No route to the goal.
    clear_path(nav_state);
  }
}

std::vector<geometry_msgs::msg::Pose> AStarPlanner::choose_path(
  const ::navmap::NavMap & map, const geometry_msgs::msg::Pose & robot,
  const geometry_msgs::msg::Pose & goal, std::vector<geometry_msgs::msg::Pose> fresh)
{
  auto remember = [this, &goal](std::vector<geometry_msgs::msg::Pose> p) {
      followed_ = p;
      followed_goal_ = goal;
      return p;
    };
  if (replan_hysteresis_ <= 0.0 || followed_.size() < 2 || !(goal == followed_goal_)) {
    return remember(std::move(fresh));
  }
  auto length = [](const std::vector<geometry_msgs::msg::Pose> & p, std::size_t from) {
      double l = 0.0;
      for (std::size_t k = from + 1; k < p.size(); ++k) {
        l += std::hypot(
          p[k].position.x - p[k - 1].position.x, p[k].position.y - p[k - 1].position.y);
      }
      return l;
    };
  // What is left of the path: the poses after its segment nearest the robot, in 3D (a path may
  // pass right under the robot on floors over each other). Nearest pose could be behind it.
  std::size_t from = 1;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k + 1 < followed_.size(); ++k) {
    const auto & a = followed_[k].position;
    const auto & b = followed_[k + 1].position;
    const double ux = b.x - a.x, uy = b.y - a.y, len2 = ux * ux + uy * uy;
    const double t = len2 > 0.0 ? std::clamp(
      ((robot.position.x - a.x) * ux + (robot.position.y - a.y) * uy) / len2, 0.0, 1.0) : 0.0;
    const double d = std::hypot(
      a.x + t * ux - robot.position.x, a.y + t * uy - robot.position.y,
      a.z + t * (b.z - a.z) - robot.position.z);
    if (d < best) {best = d; from = k + 1;}
  }
  // Still traversable on the current map, up to the horizon: sampled along the path from the robot,
  // below the inscribed cost (kEscapeDistance around the robot: below lethal) and in the terrain
  // limits. The cost is read as a_star_path does (any layer type).
  const std::string cost_layer =
    map.has_layer("inflated_obstacles") ? "inflated_obstacles" : "obstacles";
  const auto cost = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
    map.layers.get(cost_layer));
  auto cost_at = [&](::navmap::NavCelId c) -> std::uint8_t {
      return cost && c < cost->size() ? (*cost)[c] :
             map.layer_get<std::uint8_t>(cost_layer, c, navmap_ros::FREE_SPACE);
    };
  bool valid = best < kMaxPathOffset;
  ::navmap::NavMap::LocateOpts opts;
  const double step = std::clamp(cel_spacing_, 0.05, 0.25);
  double along = 0.0;
  Eigen::Vector3d prev(robot.position.x, robot.position.y, robot.position.z);
  for (std::size_t k = from; k < followed_.size() && valid; ++k) {
    const Eigen::Vector3d next(
      followed_[k].position.x, followed_[k].position.y, followed_[k].position.z);
    const double len = (next - prev).head<2>().norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 1; i <= n && valid; ++i) {
      const Eigen::Vector3d q = prev + (next - prev) * (static_cast<double>(i) / n);
      const double at = along + len * i / n;
      if (replan_hysteresis_horizon_ > 0.0 && at > replan_hysteresis_horizon_) {
        k = followed_.size();   // the rest is checked when the robot gets near
        break;
      }
      std::size_t sidx = 0;
      ::navmap::NavCelId c;
      Eigen::Vector3f bary, hit;
      if (!map.locate_navcel(q.cast<float>(), sidx, c, bary, &hit, opts)) {
        valid = false;
        break;
      }
      opts.hint_cid = c;
      const std::uint8_t v = cost_at(c);
      const bool near = (q - Eigen::Vector3d(robot.position.x, robot.position.y,
        robot.position.z)).head<2>().norm() < kEscapeDistance;
      valid = (near ? v < navmap_ros::LETHAL_OBSTACLE : v <
        navmap_ros::INSCRIBED_INFLATED_OBSTACLE) &&
        (terrain_ok_.size() != map.navcels.size() || terrain_ok_[c]);
    }
    along += len;
    prev = next;
  }
  if (!valid) {return remember(std::move(fresh));}
  // The new path only if it is clearly shorter. If A* found none (fresh is empty) but this one is
  // still valid up to the horizon, keep it; it is checked every cycle and replaced as soon as it
  // is no longer valid.
  const double left = std::hypot(
    followed_[from].position.x - robot.position.x,
    followed_[from].position.y - robot.position.y) + length(followed_, from);
  if (!fresh.empty() && length(fresh, 0) < (1.0 - replan_hysteresis_) * left) {
    return remember(std::move(fresh));
  }
  if (fresh.empty()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "A* found no path: keeping the followed one");
  }
  // It starts at followed_[from], ahead of the robot; the first segment was checked above
  std::vector<geometry_msgs::msg::Pose> kept(followed_.begin() + static_cast<std::ptrdiff_t>(from),
    followed_.end());
  return remember(std::move(kept));
}

void AStarPlanner::clear_path(NavState & nav_state)
{
  current_path_.poses.clear();
  if (path_published_ && path_pub_->get_subscription_count() > 0) {
    current_path_.header.stamp = get_node()->now();
    path_pub_->publish(current_path_);
  }
  path_published_ = false;
  nav_state.set("path", current_path_);
}

nav_msgs::msg::Path
AStarPlanner::path_smoother(
  const nav_msgs::msg::Path & in_path,
  const ::navmap::NavMap & navmap,
  int iterations,
  float alpha,
  float corner_keep_deg)
{
  nav_msgs::msg::Path out = in_path;
  if (out.poses.size() < 3 || iterations <= 0 || alpha <= 0.0f) {
    // Nothing to do
    return out;
  }

  const size_t N = out.poses.size();

  // --- 1) Pre-locate the original NavCel for each point (and keep it fixed) ---
  std::vector<::navmap::NavCelId> cids(N, std::numeric_limits<uint32_t>::max());
  std::vector<size_t> surf_idx(N, std::numeric_limits<size_t>::max());
  std::vector<Eigen::Vector3f> pts(N);

  // Fill pts from input and locate cids
  for (size_t i = 0; i < N; ++i) {
    const auto & p = out.poses[i].pose.position;
    pts[i] = Eigen::Vector3f(
      static_cast<float>(p.x),
      static_cast<float>(p.y),
      static_cast<float>(p.z));
  }

  // Use walking hints to speed up sequential location
  ::navmap::NavMap::LocateOpts opts;
  for (size_t i = 0; i < N; ++i) {
    size_t sidx = 0;
    ::navmap::NavCelId cid{};
    Eigen::Vector3f bary;
    Eigen::Vector3f hit;

    // Try full locate with hint (from previous point). NavMap's search returns the surface nearest
    // in height however far: a point by the edge of a floor, over the gap beside a ramp, would be
    // found on the floor below and moved there. That one is no good: the closest NavCel instead
    bool ok = navmap.locate_navcel(pts[i], sidx, cid, bary, &hit, opts);
    if (ok) {
      const ::navmap::NavCel & tri = navmap.navcels[cid];
      const Eigen::Vector3f on = ::navmap::closest_point_on_triangle(
        pts[i], navmap.positions.at(tri.v[0]), navmap.positions.at(tri.v[1]),
        navmap.positions.at(tri.v[2]));
      ok = std::abs(on.z() - pts[i].z()) < kSurfaceHeightTolerance;
    }
    if (!ok) {
      // Fallback to closest triangle (keeps the query on-surface)
      float sqd = 0.0f;
      Eigen::Vector3f cp;
      ok = navmap.closest_navcel(pts[i], sidx, cid, cp, sqd);
      if (ok) {
        pts[i] = cp;
      }
    }
    if (ok) {
      surf_idx[i] = sidx;
      cids[i] = cid;
      opts.hint_cid = cid;
      opts.hint_surface = sidx;
    } else {
      // If locate fails, keep the original point but mark cid as invalid.
      cids[i] = std::numeric_limits<uint32_t>::max();
      opts.hint_cid.reset();
      opts.hint_surface.reset();
    }
  }

  // Helper to fetch triangle vertices (A,B,C) for a cid
  auto get_triangle_vertices =
    [&](::navmap::NavCelId cid) -> std::array<Eigen::Vector3f, 3> {
      const ::navmap::NavCel & tri = navmap.navcels[cid];
      const auto A = navmap.positions.at(tri.v[0]);
      const auto B = navmap.positions.at(tri.v[1]);
      const auto C = navmap.positions.at(tri.v[2]);
      return {A, B, C};
    };

  // Helper: clamp a 3D point to the triangle of a given cid (closest point)
  auto clamp_to_triangle =
    [&](const Eigen::Vector3f & p, ::navmap::NavCelId cid) -> Eigen::Vector3f {
      const auto V = get_triangle_vertices(cid);
      return ::navmap::closest_point_on_triangle(p, V[0], V[1], V[2]);
    };

  // Optional: precompute anchors for sharp corners
  std::vector<uint8_t> is_anchor(N, 0);
  is_anchor.front() = 1;
  is_anchor.back() = 1;
  if (corner_keep_deg > 0.0f && N >= 3) {
    const float thr_rad = corner_keep_deg * static_cast<float>(M_PI) / 180.0f;
    for (size_t i = 1; i + 1 < N; ++i) {
      const Eigen::Vector2f a(pts[i - 1].x(), pts[i - 1].y());
      const Eigen::Vector2f b(pts[i].x(), pts[i].y());
      const Eigen::Vector2f c(pts[i + 1].x(), pts[i + 1].y());
      const Eigen::Vector2f u = (a - b);
      const Eigen::Vector2f v = (c - b);
      float nu = u.norm(), nv = v.norm();
      if (nu > 1e-6f && nv > 1e-6f) {
        float cosang = u.dot(v) / (nu * nv);
        cosang = std::max(-1.0f, std::min(1.0f, cosang));
        float ang = std::acos(cosang);
        if (ang < thr_rad) {is_anchor[i] = 1;}
      }
    }
  }

  // --- 2) Iterative smoothing with per-point (fixed) triangle constraint ---
  std::vector<Eigen::Vector3f> curr = pts;
  std::vector<Eigen::Vector3f> next = pts;

  for (int it = 0; it < iterations; ++it) {
    for (size_t i = 0; i < N; ++i) {
      // Keep invalid-cid points and anchors untouched
      if (i == 0 || i == N - 1 || is_anchor[i] ||
        cids[i] == std::numeric_limits<uint32_t>::max())
      {
        next[i] = curr[i];
        continue;
      }

      const Eigen::Vector2f prev_xy(curr[i - 1].x(), curr[i - 1].y());
      const Eigen::Vector2f curr_xy(curr[i].x(), curr[i].y());
      const Eigen::Vector2f next_xy(curr[i + 1].x(), curr[i + 1].y());

      // Laplacian target in XY
      const Eigen::Vector2f lap_target = 0.5f * (prev_xy + next_xy);
      Eigen::Vector2f cand_xy = (1.0f - alpha) * curr_xy + alpha * lap_target;

      // Build a 3D candidate with current z as seed; then clamp to triangle
      Eigen::Vector3f cand3(cand_xy.x(), cand_xy.y(), curr[i].z());

      // Clamp to the *original* triangle of this point
      Eigen::Vector3f clamped = clamp_to_triangle(cand3, cids[i]);

      next[i] = clamped;  // already lies on triangle plane, z' consistent
    }
    curr.swap(next);
  }

  // --- 3) Write back to Path (keeping header/frame) ---
  for (size_t i = 1; i + 1 < N; ++i) {  // The endpoints stay exactly as they were
    out.poses[i].pose.position.x = curr[i].x();
    out.poses[i].pose.position.y = curr[i].y();
    out.poses[i].pose.position.z = curr[i].z();
    // Orientation: leave untouched; if needed, you can realign yaw to local tangent later.
  }

  return out;
}

// Helper: detect if a layer exists (optional; if you already have API, adjust accordingly)
static inline bool layer_exists(const ::navmap::NavMap & nm, const std::string & name)
{
  return static_cast<bool>(nm.layers.get(name));
}

void AStarPlanner::build_terrain(const ::navmap::NavMap & map)
{
  using ::navmap::NavCelId;
  const std::size_t N = map.navcels.size();
  terrain_ok_.assign(N, 1);
  slope_cost_.assign(N, 0.0f);
  const float cos_max = static_cast<float>(std::cos(max_slope_deg_ * M_PI / 180.0));
  for (NavCelId c = 0; c < N; ++c) {
    const float nz = std::clamp(std::abs(map.navcels[c].normal.z()), 1e-3f, 1.0f);
    if (max_slope_deg_ < 90.0 && nz < cos_max) {terrain_ok_[c] = 0;}
    slope_cost_[c] = (1.0f - nz * nz) / (nz * nz);  // tan^2 of the slope
  }
  if (max_step_ <= 0.0 && max_roughness_ <= 0.0 && min_clearance_ <= 0.0) {return;}
  // Descriptors on a copy: the map in NavState is not written by the planner
  ::navmap::NavMap tmp = map;
  ::navmap::DescriptorParams dp;
  dp.slope = false;
  dp.step = max_step_ > 0.0;
  dp.roughness = max_roughness_ > 0.0;
  dp.clearance = min_clearance_ > 0.0;
  ::navmap::compute_descriptors(tmp, dp);
  for (NavCelId c = 0; c < N; ++c) {
    if (dp.step && tmp.layer_get<float>("step", c) > max_step_) {terrain_ok_[c] = 0;}
    if (dp.roughness && tmp.layer_get<float>("roughness", c) > max_roughness_) {
      terrain_ok_[c] = 0;
    }
    if (dp.clearance && tmp.layer_get<float>("clearance", c) < min_clearance_) {
      terrain_ok_[c] = 0;
    }
  }
}

void AStarPlanner::ensure_graph_cache(const ::navmap::NavMap & map)
{
  using ::navmap::NavCelId;
  const std::size_t N = map.navcels.size();
  const std::size_t V = map.positions.x.size();

  // Same geometry as the cached one?
  const bool same = centroids_.size() == N && vertex_cels_.size() == V &&
    neighbors_.size() == N && geometry_id_ == map.geometry_id();

  if (!same) {
    geometry_id_ = map.geometry_id();
    build_terrain(map);
    centroids_.resize(N);
    for (NavCelId c = 0; c < static_cast<NavCelId>(N); ++c) {
      const auto cc = map.navcel_centroid(c);
      centroids_[c] = Eigen::Vector3f{cc.x(), cc.y(), cc.z()};
    }

    vertex_cels_.assign(V, {});
    for (NavCelId c = 0; c < static_cast<NavCelId>(N); ++c) {
      for (const auto v : map.navcels[c].v) {
        if (v < V) {vertex_cels_[v].push_back(c);}
      }
    }

    // Neighbors through a shared vertex; two shared vertices are a shared edge
    neighbors_.assign(N, {});
    double spacing_sum = 0.0;
    std::size_t spacing_count = 0;
    for (NavCelId c = 0; c < static_cast<NavCelId>(N); ++c) {
      auto & out = neighbors_[c];
      for (const auto v : map.navcels[c].v) {
        if (v >= V) {continue;}
        for (const auto n : vertex_cels_[v]) {
          if (n == c) {continue;}
          auto it = std::find_if(
            out.begin(), out.end(), [n](const Neighbor & x) {return x.cid == n;});
          if (it == out.end()) {
            out.push_back({n, v});
          } else {
            it->shared_vertex = kNoVertex;
          }
        }
      }
      for (const auto & n : out) {
        if (n.shared_vertex == kNoVertex) {
          spacing_sum += (centroids_[n.cid] - centroids_[c]).norm();
          ++spacing_count;
        }
      }
    }
    cel_spacing_ = spacing_count > 0 ? spacing_sum / static_cast<double>(spacing_count) : 0.1;
  }

  // Sized for this map once; entries are made valid per search by their stamp (see epoch_)
  if (g_.size() != N) {
    g_.assign(N, std::numeric_limits<double>::infinity());
    parent_.assign(N, std::numeric_limits<::navmap::NavCelId>::max());
    g_stamp_.assign(N, 0);
    closed_stamp_.clear();   // epoch_ restarts: old stamps would read as closed (sized when used)
    epoch_ = 0;
  }
  if (++epoch_ == 0) {   // wrapped: clear the stamps once
    std::fill(g_stamp_.begin(), g_stamp_.end(), 0);
    std::fill(closed_stamp_.begin(), closed_stamp_.end(), 0);
    epoch_ = 1;
  }
}

namespace
{

// Whether (x, y) is inside the XY projection of triangle (a, b, c), with a small tolerance.
bool in_triangle_xy(
  const Eigen::Vector3f & p, const Eigen::Vector3f & a, const Eigen::Vector3f & b,
  const Eigen::Vector3f & c)
{
  const float d = (b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y());
  if (std::abs(d) < 1e-12f) {return false;}
  const float l1 = ((b.y() - c.y()) * (p.x() - c.x()) + (c.x() - b.x()) * (p.y() - c.y())) / d;
  const float l2 = ((c.y() - a.y()) * (p.x() - c.x()) + (a.x() - c.x()) * (p.y() - c.y())) / d;
  const float eps = 1e-4f;
  return l1 >= -eps && l2 >= -eps && (1.0f - l1 - l2) >= -eps;
}

// Height of the triangle's plane at (x, y).
float height_at(
  const Eigen::Vector3f & p, const Eigen::Vector3f & a, const Eigen::Vector3f & b,
  const Eigen::Vector3f & c)
{
  const Eigen::Vector3f n = (b - a).cross(c - a);
  if (std::abs(n.z()) < 1e-9f) {return a.z();}
  return a.z() - (n.x() * (p.x() - a.x()) + n.y() * (p.y() - a.y())) / n.z();
}

}  // namespace

std::vector<Eigen::Vector3f> AStarPlanner::shortcut_path(
  const ::navmap::NavMap & nm,
  const std::vector<Eigen::Vector3f> & points,
  const std::vector<::navmap::NavCelId> & cels)
{
  using ::navmap::NavCelId;
  const std::size_t n = points.size();
  if (n < 2) {return points;}

  auto vertices = [&nm](NavCelId c) {
      const auto & t = nm.navcels[c];
      return std::array<Eigen::Vector3f, 3>{
      nm.positions.at(t.v[0]), nm.positions.at(t.v[1]), nm.positions.at(t.v[2])};
    };

  // NavCel under (x, y) of p on the surface `from` is on, searched from `from` and its neighbors
  // first: consecutive samples of a segment are in the same NavCel or a neighbor one. NavMap's
  // search only as a fallback. The height is that of `from`'s plane there, not the segment's
  // (a straight segment over a hill is in the air or under the ground)
  auto locate_near = [&](const Eigen::Vector3f & p_in, NavCelId from, NavCelId & out) -> bool {
      const auto tf = vertices(from);
      const Eigen::Vector3f p(p_in.x(), p_in.y(), height_at(p_in, tf[0], tf[1], tf[2]));
      auto inside = [&](NavCelId c) {
          const auto t = vertices(c);
          return in_triangle_xy(p, t[0], t[1], t[2]) &&
                 std::abs(height_at(p, t[0], t[1], t[2]) - p.z()) < kSurfaceHeightTolerance;
        };
      if (inside(from)) {out = from; return true;}
      for (const auto & nb : neighbors_[from]) {
        if (inside(nb.cid)) {out = nb.cid; return true;}
      }
      ::navmap::NavMap::LocateOpts opts;
      opts.hint_cid = from;
      std::size_t sidx = 0;
      Eigen::Vector3f bary, hit;
      // NavMap's search returns the surface nearest in height, however far: with floors over each
      // other, a segment that crosses (in the plane) the gap beside a ramp would be found "on" the
      // floor below or above and taken, its samples moved to that floor. Only a surface that
      // continues the one of `from`
      if (!nm.locate_navcel(p, sidx, out, bary, &hit, opts)) {return false;}
      return inside(out);
    };

  // A step through a link (not neighbors in the mesh) is kept as is: no shortcut across it
  std::vector<char> jump(n, 0);
  for (std::size_t k = 1; k < n; ++k) {
    const auto & nb = neighbors_[cels[k - 1]];
    jump[k] = cels[k] != cels[k - 1] && std::none_of(
      nb.begin(), nb.end(), [&](const Neighbor & x) {return x.cid == cels[k];});
  }

  // Straight segment i -> j: on traversable NavCels no costlier, and no steeper, than the
  // waypoints it replaces
  auto visible = [&](std::size_t i, std::size_t j) -> bool {
      for (std::size_t k = i + 1; k <= j; ++k) {
        if (jump[k]) {return false;}
      }
      std::uint8_t allowed = 0;
      for (std::size_t k = i + 1; k < j; ++k) {
        allowed = std::max(allowed, occ(cels[k]));
      }
      float allowed_slope = 0.0f;
      for (std::size_t k = i; k <= j; ++k) {
        allowed_slope = std::max(allowed_slope, slope_cost_[cels[k]]);
      }
      allowed_slope = allowed_slope * 1.05f + 1e-3f;
      const Eigen::Vector3f a = points[i], b = points[j];
      const double len = (b - a).head<2>().norm();
      const int steps = std::max(1, static_cast<int>(std::ceil(len / (0.25 * cel_spacing_))));
      NavCelId cur = cels[i];
      for (int s = 1; s < steps; ++s) {
        const Eigen::Vector3f p = a + (b - a) * (static_cast<float>(s) / steps);
        NavCelId c;
        if (!locate_near(p, cur, c)) {return false;}
        if (slope_weight_ > 0.0 && slope_cost_[c] > allowed_slope) {return false;}
        const std::uint8_t v = occ(c);
        if (v >= navmap_ros::INSCRIBED_INFLATED_OBSTACLE || v > allowed || !terrain_ok_[c]) {
          return false;
        }
        cur = c;
      }
      // and ends on the surface it was followed on: not right under (or over) b, on another floor
      NavCelId c;
      if (!locate_near(b, cur, c)) {return false;}
      const auto t = vertices(c);
      return std::abs(height_at(b, t[0], t[1], t[2]) - b.z()) < kSurfaceHeightTolerance;
    };

  // Greedy: from each kept waypoint, the farthest visible one (exponential, then binary search)
  std::vector<std::size_t> kept{0};
  std::size_t i = 0;
  while (i + 1 < n) {
    std::size_t good = i + 1;  // A neighbor NavCel: always reachable
    std::size_t bad = n;
    for (std::size_t k = 2; ; k *= 2) {
      const std::size_t j = std::min(i + k, n - 1);
      if (j <= good) {break;}
      if (visible(i, j)) {
        good = j;
        if (j == n - 1) {break;}
      } else {
        bad = j;
        break;
      }
    }
    while (bad < n && bad - good > 1) {
      const std::size_t mid = (good + bad) / 2;
      (visible(i, mid) ? good : bad) = mid;
    }
    kept.push_back(good);
    i = good;
  }

  // Resampled at the NavCel spacing, on the surface
  std::vector<Eigen::Vector3f> out{points.front()};
  for (std::size_t k = 1; k < kept.size(); ++k) {
    const Eigen::Vector3f a = points[kept[k - 1]], b = points[kept[k]];
    if (kept[k] == kept[k - 1] + 1 && jump[kept[k]]) {  // through a link: its two ends
      out.push_back(b);
      continue;
    }
    const double len = (b - a).head<2>().norm();
    const int steps = std::max(1, static_cast<int>(std::round(len / cel_spacing_)));
    NavCelId cur = cels[kept[k - 1]];
    for (int s = 1; s <= steps; ++s) {
      Eigen::Vector3f p = a + (b - a) * (static_cast<float>(s) / steps);
      NavCelId c;
      if (s < steps && locate_near(p, cur, c)) {
        const auto t = vertices(c);
        p.z() = height_at(p, t[0], t[1], t[2]);
        cur = c;
      }
      out.push_back(p);
    }
  }
  return out;
}

std::vector<geometry_msgs::msg::Pose> AStarPlanner::a_star_path(
  const ::navmap::NavMap & nm,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  using ::navmap::NavCelId;
  using navmap_ros::FREE_SPACE;
  using navmap_ros::INSCRIBED_INFLATED_OBSTACLE;
  using navmap_ros::LETHAL_OBSTACLE;
  using navmap_ros::NO_INFORMATION;

  last_search_stats_ = SearchStats{};
  if (nm.navcels.empty()) {return {};}

  // 1) Locate start and goal NavCels (fallback to closest triangle if necessary)
  std::size_t sidx_s = 0, sidx_g = 0;
  NavCelId cid_start = 0, cid_goal = 0;
  Eigen::Vector3f bary;
  Eigen::Vector3f hit;

  Eigen::Vector3f pS(start.position.x, start.position.y, start.position.z);
  Eigen::Vector3f pG(goal.position.x, goal.position.y, goal.position.z);

  bool okS = nm.locate_navcel(pS, sidx_s, cid_start, bary, &hit);
  if (!okS) {
    Eigen::Vector3f q;
    float d2;
    if (!nm.closest_navcel(pS, sidx_s, cid_start, q, d2)) {return {};}
  }
  bool okG = nm.locate_navcel(pG, sidx_g, cid_goal, bary, &hit);
  if (!okG) {
    Eigen::Vector3f q;
    float d2;
    if (!nm.closest_navcel(pG, sidx_g, cid_goal, q, d2)) {return {};}
  }

  const std::size_t N = nm.navcels.size();

  // 2) Choose cost layer: prefer "inflated_obstacles", fallback to "obstacles"
  const std::string cost_layer =
    layer_exists(nm, "inflated_obstacles") ? "inflated_obstacles" : "obstacles";

  // Ensure cached buffers match the current NavMap.
  ensure_graph_cache(nm);

  // Precomputed centroids in `centroids_` are used for cost, heuristic, and edge lengths.
  auto euclid = [&](NavCelId a, NavCelId b) -> double {
      const auto d = centroids_[a] - centroids_[b];
      return static_cast<double>(d.norm());
    };

  // Per-NavCel uint8_t cost values (0..255) of the selected layer: the layer itself when it is
  // uint8 of the map's size (no copy), else converted once into occ_ (a missing value: FREE_SPACE)
  struct ResetOcc
  {
    const std::uint8_t * & p;
    ~ResetOcc() {p = nullptr;}   // the layer's data is only valid during this plan
  } reset_occ{occ_data_};
  {
    const auto view = std::dynamic_pointer_cast<::navmap::LayerView<std::uint8_t>>(
      nm.layers.get(cost_layer));
    if (view && view->size() == N) {
      occ_data_ = view->data().data();
    } else {
      occ_data_ = nullptr;
      occ_.resize(N);
      for (NavCelId c = 0; c < static_cast<NavCelId>(N); ++c) {
        occ_[c] = nm.layer_get<std::uint8_t>(cost_layer, c, FREE_SPACE);
      }
    }
  }
  // g of the current search (an entry of another search is +inf)
  const double inf_cost = std::numeric_limits<double>::infinity();
  auto g = [&](NavCelId c) -> double {return g_stamp_[c] == epoch_ ? g_[c] : inf_cost;};
  auto set_g = [&](NavCelId c, double v, NavCelId from) {
      g_[c] = v;
      parent_[c] = from;
      g_stamp_[c] = epoch_;
    };

  // Traversability: block lethal, unknown and inscribed (the robot would touch an obstacle),
  // as the costmap planner does.
  auto traversable = [&](NavCelId c) -> bool {
      const std::uint8_t v = occ(c);
      return v < INSCRIBED_INFLATED_OBSTACLE && terrain_ok_[c];
    };
  // The robot may already be within the inscribed band (e.g. stopped by a reflex): it can
  // leave it (within kEscapeDistance), unless it is on an obstacle or an unknown cell.
  auto can_start_at = [&](NavCelId c) -> bool {
      const std::uint8_t v = occ(c);
      return (v != LETHAL_OBSTACLE) && (v != NO_INFORMATION);
    };

  // Normalize a uint8_t cost into [0, 1]; FREE_SPACE=0 → 0.0; INSCRIBED=253 → ~1.0.
  // Returns +inf for non-traversable (lethal/unknown).
  auto normalized_cost = [&](NavCelId c) -> double {
      const std::uint8_t v = occ(c);
      if (v == LETHAL_OBSTACLE || v == NO_INFORMATION) {
        return std::numeric_limits<double>::infinity();
      }
      // Cap at INSCRIBED_INFLATED_OBSTACLE to avoid division by 0 at 254/255.
      const double max_cost = static_cast<double>(INSCRIBED_INFLATED_OBSTACLE);  // 253
      return static_cast<double>(v) / max_cost;  // FREE=0 → 0.0, INSCRIBED=253 → 1.0
    };

  // If the start is on an obstacle/unknown cell, or the goal is not traversable, do not plan.
  if (!can_start_at(cid_start) || !traversable(cid_goal)) {
    return {};
  }

  // Weighted step cost:
  //   base geometric cost (edge length) scaled by (cost_factor_ + cost_weight_ *
  //   norm_cost(target)). This preserves admissibility with heuristic h = Euclidean distance, since
  //   the minimal multiplier ≥ 1.
  auto step_cost = [&](NavCelId from, NavCelId to) -> double {
      const double base = euclid(from, to);
      if (!std::isfinite(base) || base <= 0.0) {
        return std::numeric_limits<double>::infinity();
      }

      const double ncost = normalized_cost(to);
      if (!std::isfinite(ncost)) {
        return std::numeric_limits<double>::infinity();
      }

      // Ensure the multiplier is at least 1.0 so h = euclid remains admissible.
      // If your cost_factor_ is already ≥ 1, this holds. Otherwise we clamp.
      const double cf = std::max(1.0, static_cast<double>(cost_factor_));
      double mult = cf + static_cast<double>(cost_weight_) * ncost;
      if (slope_weight_ > 0.0) {
        // tan^2 of the slope of the NavCel: gentle slopes cost little, steep ones a lot, and
        // across a slope as much as up or down it
        mult += slope_weight_ * static_cast<double>(slope_cost_[to]);
      }

      return base * mult;
    };

  // 4) A* search on the triangle graph, using cost-aware edges.
  struct Node
  {
    NavCelId cid;
    double f;
  };
  struct Cmp
  {
    bool operator()(const Node & a, const Node & b) const {return a.f > b.f;}
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  // Every meter costs at least cost_factor: still admissible, and far fewer expansions
  const double h_scale =
    heuristic_weight_ * std::max(1.0, static_cast<double>(cost_factor_));
  auto h = [&](NavCelId a, NavCelId b) -> double {
      return h_scale * euclid(a, b);
    };

  set_g(cid_start, 0.0, std::numeric_limits<NavCelId>::max());
  open.push(Node{cid_start, h(cid_start, cid_goal)});
  last_search_stats_ = SearchStats{0, 1};

  // Weighted A* (heuristic_weight > 1) without re-opening: each NavCel is expanded once (paths at
  // most heuristic_weight times the optimal cost). With weight 1 the search is the classic one.
  const bool weighted = heuristic_weight_ > 1.0;
  if (weighted && closed_stamp_.size() != N) {closed_stamp_.assign(N, 0);}
  auto closed = [&](NavCelId c) {return closed_stamp_[c] == epoch_;};

  while (!open.empty()) {
    const auto cur = open.top();
    open.pop();
    const NavCelId u = cur.cid;
    if (weighted) {
      if (closed(u)) {continue;}
      closed_stamp_[u] = epoch_;
    }
    ++last_search_stats_.expanded;

    if (u == cid_goal) {break;}

    for (const auto & nb : neighbors_[u]) {
      const NavCelId v = nb.cid;

      // Skip non-traversable neighbors (lethal, unknown or inscribed); inscribed ones only
      // near the start, so a robot within the inscribed band can leave it.
      const bool escaping = can_start_at(v) &&
        (centroids_[v] - centroids_[cid_start]).norm() < kEscapeDistance;
      if (!traversable(v) && !escaping) {continue;}

      // Only a vertex in common: no cutting a corner, every NavCel around it must be passable
      if (nb.shared_vertex != kNoVertex) {
        const auto & around = vertex_cels_[nb.shared_vertex];
        const bool clear = std::all_of(
          around.begin(), around.end(), [&](NavCelId c) {
            return traversable(c) || (escaping && can_start_at(c));
          });
        if (!clear) {continue;}
      }

      if (weighted && closed(v)) {continue;}
      const double sc = step_cost(u, v);
      if (!std::isfinite(sc)) {continue;}

      const double tentative = g(u) + sc;
      if (tentative < g(v)) {
        set_g(v, tentative, u);
        const double f = tentative + h(v, cid_goal);
        open.push(Node{v, f});
        ++last_search_stats_.pushed;
      }
    }

    // Links to other surfaces (stairs, ramps, doors...) of the allowed kinds
    if (!use_links_) {continue;}
    for (const auto li : nm.links_from(u)) {
      const auto & l = nm.links()[li];
      const auto kind = static_cast<std::size_t>(l.type);
      if (kind >= link_type_ok_.size() || !link_type_ok_[kind]) {continue;}
      const NavCelId v = l.from == u ? l.to : l.from;
      if (v >= N || !traversable(v) || (weighted && closed(v))) {continue;}
      const double ncost = normalized_cost(v);
      if (!std::isfinite(ncost)) {continue;}
      // Never shorter than the straight line, so the heuristic stays admissible
      const double len = std::max(static_cast<double>(l.length), euclid(u, v));
      const double cf = std::max(1.0, static_cast<double>(cost_factor_));
      const double tentative = g(u) + len * (cf + static_cast<double>(cost_weight_) * ncost);
      if (tentative < g(v)) {
        set_g(v, tentative, u);
        open.push(Node{v, tentative + h(v, cid_goal)});
        ++last_search_stats_.pushed;
      }
    }
  }

  if (!std::isfinite(g(cid_goal))) {
    return {};
  }

  // 5) Path reconstruction (centroid-based polyline).
  std::vector<NavCelId> cels;
  for (NavCelId c = cid_goal;
    c != std::numeric_limits<NavCelId>::max();
    c = parent_[c])
  {
    cels.push_back(c);
    if (c == cid_start) {break;}
  }
  std::reverse(cels.begin(), cels.end());

  std::vector<Eigen::Vector3f> points;
  points.reserve(cels.size());
  for (const auto c : cels) {
    points.push_back(centroids_[c]);
  }
  // Ends exactly at the goal, not at its NavCel's centroid
  points.back().x() = static_cast<float>(goal.position.x);
  points.back().y() = static_cast<float>(goal.position.y);

  std::vector<geometry_msgs::msg::Pose> path;
  for (const auto & pt : shortcut_path(nm, points, cels)) {
    geometry_msgs::msg::Pose p;
    p.position.x = pt.x();
    p.position.y = pt.y();
    p.position.z = pt.z();
    p.orientation = goal.orientation;
    path.push_back(std::move(p));
  }
  // Exactly the goal (the waypoints are floats)
  path.back().position.x = goal.position.x;
  path.back().position.y = goal.position.y;
  return path;
}

}  // namespace navmap
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::AStarPlanner, easynav::PlannerMethodBase)
