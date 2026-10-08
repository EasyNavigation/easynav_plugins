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
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

namespace easynav
{
namespace navmap
{

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

  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  node->get_parameter(plugin_name + ".cost_weight", cost_weight_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);

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
  auto poses = a_star_path(navmap, robot_pose.pose.pose, goal);
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

    // Try full locate with hint (from previous point)
    bool ok = navmap.locate_navcel(pts[i], sidx, cid, bary, &hit, opts);
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

void AStarPlanner::ensure_graph_cache(const ::navmap::NavMap & map)
{
  using ::navmap::NavCelId;
  const std::size_t N = map.navcels.size();
  const std::size_t V = map.positions.x.size();

  // Same geometry as the cached one? (sizes, and the first and last centroids)
  bool same = centroids_.size() == N && vertex_cels_.size() == V && neighbors_.size() == N;
  if (same && N > 0) {
    const auto c0 = map.navcel_centroid(0);
    const auto cn = map.navcel_centroid(static_cast<NavCelId>(N - 1));
    same = (Eigen::Vector3f{c0.x(), c0.y(), c0.z()} - centroids_.front()).norm() < 1e-6f &&
      (Eigen::Vector3f{cn.x(), cn.y(), cn.z()} - centroids_.back()).norm() < 1e-6f;
  }

  if (!same) {
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

  if (occ_.size() != N) {
    occ_.resize(N);
  }

  const double inf = std::numeric_limits<double>::infinity();

  if (g_.size() != N) {
    g_.assign(N, inf);
  } else {
    std::fill(g_.begin(), g_.end(), inf);
  }

  if (parent_.size() != N) {
    parent_.assign(N, std::numeric_limits<::navmap::NavCelId>::max());
  } else {
    std::fill(
      parent_.begin(), parent_.end(),
      std::numeric_limits<::navmap::NavCelId>::max());
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

  // NavCel under p, searched from `from` and its neighbors first: consecutive samples of a
  // segment are in the same NavCel or a neighbor one. NavMap's search only as a fallback.
  auto locate_near = [&](const Eigen::Vector3f & p, NavCelId from, NavCelId & out) -> bool {
      auto inside = [&](NavCelId c) {
          const auto t = vertices(c);
          return in_triangle_xy(p, t[0], t[1], t[2]) &&
                 std::abs(height_at(p, t[0], t[1], t[2]) - p.z()) < 0.5f;
        };
      if (inside(from)) {out = from; return true;}
      for (const auto & nb : neighbors_[from]) {
        if (inside(nb.cid)) {out = nb.cid; return true;}
      }
      ::navmap::NavMap::LocateOpts opts;
      opts.hint_cid = from;
      std::size_t sidx = 0;
      Eigen::Vector3f bary, hit;
      return nm.locate_navcel(p, sidx, out, bary, &hit, opts);
    };

  // Straight segment i -> j: on traversable NavCels no costlier than the waypoints it replaces
  auto visible = [&](std::size_t i, std::size_t j) -> bool {
      std::uint8_t allowed = 0;
      for (std::size_t k = i + 1; k < j; ++k) {
        allowed = std::max(allowed, occ_[cels[k]]);
      }
      const Eigen::Vector3f a = points[i], b = points[j];
      const double len = (b - a).head<2>().norm();
      const int steps = std::max(1, static_cast<int>(std::ceil(len / (0.25 * cel_spacing_))));
      NavCelId cur = cels[i];
      for (int s = 1; s < steps; ++s) {
        const Eigen::Vector3f p = a + (b - a) * (static_cast<float>(s) / steps);
        NavCelId c;
        if (!locate_near(p, cur, c)) {return false;}
        const std::uint8_t v = occ_[c];
        if (v >= navmap_ros::INSCRIBED_INFLATED_OBSTACLE || v > allowed) {return false;}
        cur = c;
      }
      return true;
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
  using namespace navmap_ros;

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

  // Cache per-NavCel uint8_t cost values (0..255) for the selected layer.
  for (NavCelId c = 0; c < static_cast<NavCelId>(N); ++c) {
    // If the cell has no stored value, assume FREE_SPACE (0).
    occ_[c] = nm.layer_get<std::uint8_t>(cost_layer, c, FREE_SPACE);
  }

  // Traversability: block lethal, unknown and inscribed (the robot would touch an obstacle),
  // as the costmap planner does.
  auto traversable = [&](NavCelId c) -> bool {
      const std::uint8_t v = occ_[c];
      return v < INSCRIBED_INFLATED_OBSTACLE;
    };
  // How far from the start inscribed cells are still traversable (m).
  constexpr double kEscapeDistance = 1.0;
  // The robot may already be within the inscribed band (e.g. stopped by a reflex): it can
  // leave it, unless it is on an obstacle or an unknown cell.
  auto can_start_at = [&](NavCelId c) -> bool {
      const std::uint8_t v = occ_[c];
      return (v != LETHAL_OBSTACLE) && (v != NO_INFORMATION);
    };

  // Normalize a uint8_t cost into [0, 1]; FREE_SPACE=0 → 0.0; INSCRIBED=253 → ~1.0.
  // Returns +inf for non-traversable (lethal/unknown).
  auto normalized_cost = [&](NavCelId c) -> double {
      const std::uint8_t v = occ_[c];
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
  //   base geometric cost (edge length) scaled by (cost_factor_ + cost_weight_ * norm_cost(target)).
  // This preserves admissibility with heuristic h = Euclidean distance, since the minimal multiplier ≥ 1.
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
      const double mult = cf + static_cast<double>(cost_weight_) * ncost;

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
  const double h_scale = std::max(1.0, static_cast<double>(cost_factor_));
  auto h = [&](NavCelId a, NavCelId b) -> double {
      return h_scale * euclid(a, b);
    };

  g_[cid_start] = 0.0;
  open.push(Node{cid_start, h(cid_start, cid_goal)});

  while (!open.empty()) {
    const auto cur = open.top();
    open.pop();
    const NavCelId u = cur.cid;

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

      const double sc = step_cost(u, v);
      if (!std::isfinite(sc)) {continue;}

      const double tentative = g_[u] + sc;
      if (tentative < g_[v]) {
        g_[v] = tentative;
        parent_[v] = u;
        const double f = tentative + h(v, cid_goal);
        open.push(Node{v, f});
      }
    }
  }

  if (!std::isfinite(g_[cid_goal])) {
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
