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

#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdint>

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

  node->declare_parameter<double>(plugin_name + ".cost_factor", 2.0);
  node->declare_parameter<bool>(plugin_name + ".continuous_replan", true);

  node->get_parameter(plugin_name + ".cost_factor", cost_factor_);
  node->get_parameter(plugin_name + ".continuous_replan", continuous_replan_);

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
}

void AStarPlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map.navmap")) {
    return;
  }

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty() || !nav_state.has("map.navmap")) {
    nav_state.set("path", current_path_);
    return;
  }

  const auto & navmap = nav_state.get<::navmap::NavMap>("map.navmap");

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  if (goals.header.frame_id != tf_info.map_frame) {
    RCLCPP_WARN(
      get_node()->get_logger(), "Goals frame is not 'map': %s",
      goals.header.frame_id.c_str());
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
    }
  }
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
  for (size_t i = 0; i < N; ++i) {
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
  const std::size_t N = map.navcels.size();

  // Cache centroids: only recompute when the number of NavCels changes.
  if (centroids_.size() != N) {
    centroids_.resize(N);
    for (::navmap::NavCelId c = 0; c < static_cast<::navmap::NavCelId>(N); ++c) {
      const auto cc = map.navcel_centroid(c);
      centroids_[c] = Eigen::Vector3f{cc.x(), cc.y(), cc.z()};
    }
  }

  // Ensure occupancy buffer has the right size (values are filled per-planning call).
  if (occ_.size() != N) {
    occ_.resize(N);
  }

  // Resize and reset A* buffers.
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

  // Traversability: block lethal and unknown.
  auto traversable = [&](NavCelId c) -> bool {
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

  // If start or goal lands on non-traversable, do not plan.
  if (!traversable(cid_start) || !traversable(cid_goal)) {
    return {};
  }

  // Weighted step cost:
  //   base geometric cost (edge length) scaled by (cost_factor_ + inflation_penalty_ * norm_cost(target)).
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
      const double mult = cf + static_cast<double>(inflation_penalty_) * ncost;

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

  auto h = [&](NavCelId a, NavCelId b) -> double {
      // Heuristic: pure Euclidean distance → admissible (never overestimates).
      return euclid(a, b);
    };

  g_[cid_start] = 0.0;
  open.push(Node{cid_start, h(cid_start, cid_goal)});

  while (!open.empty()) {
    const auto cur = open.top();
    open.pop();
    const NavCelId u = cur.cid;

    if (u == cid_goal) {break;}

    const auto & tri = nm.navcels[u];
    for (int e = 0; e < 3; ++e) {
      NavCelId v = tri.neighbor[e];
      if (v == std::numeric_limits<std::uint32_t>::max()) {
        continue;
      }
      const std::size_t vidx = static_cast<std::size_t>(v);
      if (vidx >= N) {
        continue;
      }

      // Skip non-traversable neighbors (lethal or unknown).
      if (!traversable(v)) {continue;}

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
  std::vector<geometry_msgs::msg::Pose> path;
  for (NavCelId c = cid_goal;
    c != std::numeric_limits<NavCelId>::max();
    c = parent_[c])
  {
    geometry_msgs::msg::Pose p;
    p.position.x = centroids_[c].x();
    p.position.y = centroids_[c].y();
    p.position.z = centroids_[c].z();
    p.orientation = goal.orientation;
    path.push_back(std::move(p));
    if (c == cid_start) {break;}
  }
  std::reverse(path.begin(), path.end());

  if (path.empty()) {path.push_back(goal);}
  return path;
}

}  // namespace navmap
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::AStarPlanner, easynav::PlannerMethodBase)
