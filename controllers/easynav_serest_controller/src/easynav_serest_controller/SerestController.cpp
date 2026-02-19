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
/// \brief Implementation of the SimpleController class.

#include <algorithm>
#include <limits>
#include <cmath>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_serest_controller/SerestController.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace easynav
{


SerestController::SerestController() = default;
SerestController::~SerestController() = default;

void
SerestController::on_initialize()
{
  auto node = get_node();
  const auto & ns = get_plugin_name();

  // Maximums and basic limits
  node->declare_parameter<bool>(ns + ".allow_reverse", allow_reverse_);
  node->declare_parameter<double>(ns + ".v_progress_min", v_progress_min_);
  node->declare_parameter<double>(ns + ".k_s_share_max", k_s_share_max_);

  node->declare_parameter<double>(ns + ".max_linear_speed", max_linear_speed_);
  node->declare_parameter<double>(ns + ".max_angular_speed", max_angular_speed_);
  node->declare_parameter<double>(ns + ".max_linear_acc", max_linear_acc_);
  node->declare_parameter<double>(ns + ".max_angular_acc", max_angular_acc_);

  // Tracking
  node->declare_parameter<double>(ns + ".k_s", k_s_);
  node->declare_parameter<double>(ns + ".k_theta", k_theta_);
  node->declare_parameter<double>(ns + ".k_y", k_y_);
  node->declare_parameter<double>(ns + ".ell", ell_);
  node->declare_parameter<double>(ns + ".v_ref", v_ref_);
  node->declare_parameter<double>(ns + ".eps", eps_);

  // Safety
  node->declare_parameter<double>(ns + ".a_acc", a_acc_);
  node->declare_parameter<double>(ns + ".a_brake", a_brake_);
  node->declare_parameter<double>(ns + ".a_lat_max", a_lat_max_);
  node->declare_parameter<double>(ns + ".d0_margin", d0_margin_);
  node->declare_parameter<double>(ns + ".tau_latency", tau_latency_);
  node->declare_parameter<double>(ns + ".d_hard", d_hard_);
  node->declare_parameter<double>(ns + ".t_emerg", t_emerg_);

  // Blend at vertices
  node->declare_parameter<double>(ns + ".blend_base", blend_base_);
  node->declare_parameter<double>(ns + ".blend_k_per_v", blend_k_per_v_);
  node->declare_parameter<double>(ns + ".kappa_max", kappa_max_);

  // For obstacle detection
  node->declare_parameter<double>(ns + ".dist_search_radius", dist_search_radius_);

  node->declare_parameter<double>(ns + ".goal_pos_tol", goal_pos_tol_);
  node->declare_parameter<double>(ns + ".goal_yaw_tol_deg", goal_yaw_tol_deg_);
  node->declare_parameter<double>(ns + ".slow_radius", slow_radius_);
  node->declare_parameter<double>(ns + ".slow_min_speed", slow_min_speed_);
  node->declare_parameter<double>(ns + ".final_align_k", final_align_k_);
  node->declare_parameter<double>(ns + ".final_align_wmax", final_align_wmax_);

  node->declare_parameter<bool>(ns + ".corner_guard_enable", corner_guard_enable_);
  node->declare_parameter<double>(ns + ".corner_gain_ey", corner_gain_ey_);
  node->declare_parameter<double>(ns + ".corner_gain_eth", corner_gain_eth_);
  node->declare_parameter<double>(ns + ".corner_gain_kappa", corner_gain_kappa_);
  node->declare_parameter<double>(ns + ".corner_min_alpha", corner_min_alpha_);
  node->declare_parameter<double>(ns + ".corner_boost_omega", corner_boost_omega_);
  node->declare_parameter<double>(ns + ".apex_ey_des", apex_ey_des_);
  node->declare_parameter<double>(ns + ".a_lat_soft", a_lat_soft_);


  // Get
  node->get_parameter<bool>(ns + ".allow_reverse", allow_reverse_);
  node->get_parameter<double>(ns + ".v_progress_min", v_progress_min_);
  node->get_parameter<double>(ns + ".k_s_share_max", k_s_share_max_);

  node->get_parameter<double>(ns + ".max_linear_speed", max_linear_speed_);
  node->get_parameter<double>(ns + ".max_angular_speed", max_angular_speed_);
  node->get_parameter<double>(ns + ".max_linear_acc", max_linear_acc_);
  node->get_parameter<double>(ns + ".max_angular_acc", max_angular_acc_);

  node->get_parameter<double>(ns + ".k_s", k_s_);
  node->get_parameter<double>(ns + ".k_theta", k_theta_);
  node->get_parameter<double>(ns + ".k_y", k_y_);
  node->get_parameter<double>(ns + ".ell", ell_);
  node->get_parameter<double>(ns + ".v_ref", v_ref_);
  node->get_parameter<double>(ns + ".eps", eps_);

  node->get_parameter<double>(ns + ".a_acc", a_acc_);
  node->get_parameter<double>(ns + ".a_brake", a_brake_);
  node->get_parameter<double>(ns + ".a_lat_max", a_lat_max_);
  node->get_parameter<double>(ns + ".d0_margin", d0_margin_);
  node->get_parameter<double>(ns + ".tau_latency", tau_latency_);
  node->get_parameter<double>(ns + ".d_hard", d_hard_);
  node->get_parameter<double>(ns + ".t_emerg", t_emerg_);

  node->get_parameter<double>(ns + ".blend_base", blend_base_);
  node->get_parameter<double>(ns + ".blend_k_per_v", blend_k_per_v_);
  node->get_parameter<double>(ns + ".kappa_max", kappa_max_);

  node->get_parameter<double>(ns + ".dist_search_radius", dist_search_radius_);

  node->get_parameter<double>(ns + ".goal_pos_tol", goal_pos_tol_);
  node->get_parameter<double>(ns + ".goal_yaw_tol_deg", goal_yaw_tol_deg_);
  node->get_parameter<double>(ns + ".slow_radius", slow_radius_);
  node->get_parameter<double>(ns + ".slow_min_speed", slow_min_speed_);
  node->get_parameter<double>(ns + ".final_align_k", final_align_k_);
  node->get_parameter<double>(ns + ".final_align_wmax", final_align_wmax_);

  node->get_parameter<bool>(ns + ".corner_guard_enable", corner_guard_enable_);
  node->get_parameter<double>(ns + ".corner_gain_ey", corner_gain_ey_);
  node->get_parameter<double>(ns + ".corner_gain_eth", corner_gain_eth_);
  node->get_parameter<double>(ns + ".corner_gain_kappa", corner_gain_kappa_);
  node->get_parameter<double>(ns + ".corner_min_alpha", corner_min_alpha_);
  node->get_parameter<double>(ns + ".corner_boost_omega", corner_boost_omega_);
  node->get_parameter<double>(ns + ".apex_ey_des", apex_ey_des_);
  node->get_parameter<double>(ns + ".a_lat_soft", a_lat_soft_);

  last_vlin_ = 0.0;
  last_vrot_ = 0.0;
  last_update_ts_ = node->now();
}

SerestController::PathData
SerestController::build_path_data(const nav_msgs::msg::Path & path) const
{
  PathData pd;
  pd.pts.reserve(path.poses.size());
  pd.s_acc.reserve(path.poses.size());

  double s = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & ps = path.poses[i].pose.position;
    pd.pts.push_back(v2(ps.x, ps.y));
    if (i == 0) {
      pd.s_acc.push_back(0.0);
    } else {
      s += norm(pd.pts[i] - pd.pts[i - 1]);
      pd.s_acc.push_back(s);
    }
  }
  return pd;
}

SerestController::Projection
SerestController::project_on_path(const PathData & pd, const Vec2 & p) const
{
  Projection prj;
  if (pd.pts.size() <= 1) {
    prj.seg_idx = 0;
    prj.s_star = 0.0;
    prj.closest = pd.pts.empty() ? p : pd.pts.front();
    prj.t = 0.0;
    return prj;
  }

  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < pd.pts.size(); ++i) {
    Vec2 a = pd.pts[i], b = pd.pts[i + 1];
    Vec2 ab = b - a;
    double ab2 = dot(ab, ab);
    double t = ab2 > 1e-12 ? std::clamp(dot(p - a, ab) / ab2, 0.0, 1.0) : 0.0;
    Vec2 q = a + ab * t;
    double d2 = dot(p - q, p - q);
    if (d2 < best_d2) {
      best_d2 = d2;
      prj.seg_idx = i;
      prj.closest = q;
      prj.t = t;
    }
  }
  // s* = s_i + t * |ab|
  Vec2 a = pd.pts[prj.seg_idx], b = pd.pts[prj.seg_idx + 1];
  double seg_len = norm(b - a);
  prj.s_star = pd.s_acc[prj.seg_idx] + prj.t * seg_len;
  return prj;
}

SerestController::RefKinematics
SerestController::ref_heading_and_curvature(
  const PathData & pd, const Projection & prj, double v_prev) const
{
  RefKinematics rk{};

  auto seg_dir = [&](size_t i)->Vec2 {
      if (i + 1 >= pd.pts.size()) {return v2(1, 0);}
      return normalize(pd.pts[i + 1] - pd.pts[i]);
    };
  auto atan2dir = [](const Vec2 & t){return std::atan2(t.y, t.x);};

  // Relevant segment indices: i-1, i, i + 1
  const size_t i = prj.seg_idx;
  const Vec2 Ti = seg_dir(i);
  double psi_i = atan2dir(Ti);

  // Local blend with look-ahead
  double b = std::max(0.1, blend_base_ + blend_k_per_v_ * std::fabs(v_prev));

  // Determine if we are near the beginning or end of a segment
  double s_i = pd.s_acc[i];
  double s_ip1 = pd.s_acc[i + 1];
  double s = prj.s_star;

  // Default: constant segment heading, kappa = 0
  rk.psi_ref = psi_i;
  rk.kappa_hat = 0.0;

  // Blend near the beginning of segment i (with i-1)
  if (i > 0 && (s - s_i) < b) {
    Vec2 Tim1 = seg_dir(i - 1);
    double psi_im1 = atan2dir(Tim1);
    double w = 1.0 / (1.0 + std::exp(-( (s - s_i) / b ))); // sigmoid in [s_i, s_i+b]
    // Circular interpolation of headings
    double sx = (1.0 - w) * std::cos(psi_im1) + w * std::cos(psi_i);
    double sy = (1.0 - w) * std::sin(psi_im1) + w * std::sin(psi_i);
    rk.psi_ref = std::atan2(sy, sx);

    // kappa surrogate ~ dpsi/ds ≈ (Δψ/b) σ'(·)
    double dpsi = std::atan2(std::sin(psi_i - psi_im1), std::cos(psi_i - psi_im1));
    double sigma_prime = (w * (1 - w)) / b; // derivada de logística escalada
    rk.kappa_hat = std::clamp(dpsi * sigma_prime, -kappa_max_, kappa_max_);
  }

  // Blend near the end of segment i (towards i + 1)
  if (i + 1 < pd.pts.size() - 1 && (s_ip1 - s) < b) {
    Vec2 Tip1 = seg_dir(i + 1);
    double psi_ip1 = atan2dir(Tip1);
    double w = 1.0 / (1.0 + std::exp(-( (s_ip1 - s) / b ))); // symmetric
    // Blend between i and i + 1
    double sx = (1.0 - w) * std::cos(psi_i) + w * std::cos(psi_ip1);
    double sy = (1.0 - w) * std::sin(psi_i) + w * std::sin(psi_ip1);
    rk.psi_ref = std::atan2(sy, sx);

    double dpsi = std::atan2(std::sin(psi_ip1 - psi_i), std::cos(psi_ip1 - psi_i));
    double sigma_prime = (w * (1 - w)) / b;
    double kappa2 = std::clamp(dpsi * sigma_prime, -kappa_max_, kappa_max_);
    // If there are two overlapping blends, combine smoothly (average)
    rk.kappa_hat = 0.5 * (rk.kappa_hat + kappa2);
  }

  return rk;
}

void
SerestController::frenet_errors(
  const Vec2 & robot_xy, double robot_yaw,
  const Projection & prj, double psi_ref,
  double & e_y, double & e_theta) const
{
  // Normal vector to the right of the tangent
  Vec2 T = v2(std::cos(psi_ref), std::sin(psi_ref));
  Vec2 N = v2(-T.y, T.x); // 90º to the left (convention)
  Vec2 err = robot_xy - prj.closest;

  e_y = dot(err, N); // signed lateral distance
  e_theta = std::atan2(std::sin(robot_yaw - psi_ref), std::cos(robot_yaw - psi_ref));
}

double
SerestController::closest_obstacle_distance(
  const NavState & nav_state)
{
  // 1) Prefer direct measurement if it exists
  if (nav_state.has("closest_obstacle_distance")) {
    try {
      return nav_state.get<double>("closest_obstacle_distance");
    } catch (...) {
      // fall through to estimation
    }
  }

  // 2) Analyze distance sensors
  if (!nav_state.has("points")) {return std::numeric_limits<double>::infinity();}

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  auto view = PointPerceptionsOpsView(perceptions);
  view.downsample(0.3)
  .fuse(tf_info.robot_frame)
  .filter({-dist_search_radius_, -dist_search_radius_, NAN},
    {dist_search_radius_, dist_search_radius_, 2.0})
  .collapse({NAN, NAN, 0.1})
  .downsample(0.3);
  const auto & fused = view.as_points();

  auto latest_time = [](const PointPerceptions & perceptions){
      rclcpp::Time latest_stamp;
      bool inited = false;

      for (const auto & perception : perceptions) {
        if (!inited || perception->stamp > latest_stamp) {
          latest_stamp = perception->stamp;
          inited = true;
        }
      }
      return latest_stamp;
    };

  if (last_input_ts_ < latest_time(perceptions)) {
    last_input_ts_ = latest_time(perceptions);
  }

  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto p : fused) {
    double dist = sqrt(p.x * p.x + p.y * p.y);
    min_dist = std::min(min_dist, dist);
  }

  return min_dist;
}

double
SerestController::v_safe_from_distance(double d, double slope_sin) const
{
  // a_eff reduces braking capability on slopes; in 2D slope_sin = 0
  const double a_eff = std::max(0.1, a_brake_ - 9.81 * slope_sin);
  if (d <= d0_margin_) {return 0.0;}

  double vsq = 2.0 * a_eff * (d - d0_margin_);
  double v = vsq > 0.0 ? std::sqrt(vsq) : 0.0;
  v -= tau_latency_ * a_eff;
  return std::max(0.0, v);
}

double
SerestController::v_curvature_limit(double kappa_hat) const
{
  // Three typical limits: v_max, ω_max/|κ| and sqrt(a_lat_max/|κ|)
  double v1 = max_linear_speed_;
  double v2 = std::numeric_limits<double>::infinity();
  double v3 = std::numeric_limits<double>::infinity();

  double ak = std::fabs(kappa_hat);
  if (ak > 1e-6) {
    v2 = max_angular_speed_ / ak;
    v3 = std::sqrt(a_lat_max_ / ak);
  }
  return std::min({v1, v2, v3});
}

double
SerestController::compute_dt_and_update_clock()
{
  auto now = get_node()->now();
  double dt = (now - last_update_ts_).seconds();
  if (dt <= 0.0) {dt = 1.0 / 30.0;}
  last_update_ts_ = now;
  return dt;
}

bool
SerestController::fetch_required_inputs(
  NavState & nav_state,
  nav_msgs::msg::Path & path,
  nav_msgs::msg::Odometry & odom)
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  if (!nav_state.has("path") || !nav_state.has("robot_pose") || !nav_state.has("map.dynamic")) {
    publish_stop(nav_state, tf_info.robot_frame);
    return false;
  }

  path = nav_state.get<nav_msgs::msg::Path>("path");
  odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

  if (rclcpp::Time(path.header.stamp, last_input_ts_.get_clock_type()) > last_input_ts_) {
    last_input_ts_ = path.header.stamp;
  }
  if (rclcpp::Time(odom.header.stamp, last_input_ts_.get_clock_type()) > last_input_ts_) {
    last_input_ts_ = odom.header.stamp;
  }

  if (path.poses.empty()) {
    publish_stop(nav_state, path.header.frame_id);
    return false;
  }

  return true;
}

void
SerestController::robot_state_from_odom(
  const nav_msgs::msg::Odometry & odom,
  Vec2 & robot_xy, double & yaw) const
{
  const auto & rp = odom.pose.pose.position;
  const auto & rq = odom.pose.pose.orientation;
  yaw = tf2::getYaw(rq);
  robot_xy = Vec2{rp.x, rp.y};
}

void
SerestController::compute_goal_zone(
  const nav_msgs::msg::Path & path,
  const Vec2 & robot_xy, double robot_yaw,
  double & dist_xy_goal, double & e_theta_goal,
  double & stop_r, double & slow_r, double & gamma_slow,
  Vec2 & goal_xy, double & yaw_goal) const
{
  const auto & pgoal = path.poses.back().pose.position;
  yaw_goal = tf2::getYaw(path.poses.back().pose.orientation);
  goal_xy = Vec2{pgoal.x, pgoal.y};

  dist_xy_goal = std::hypot(robot_xy.x - goal_xy.x, robot_xy.y - goal_xy.y);
  e_theta_goal = std::atan2(std::sin(robot_yaw - yaw_goal), std::cos(robot_yaw - yaw_goal));

  // Use externally provided goal position tolerance when available (via update_rt),
  // which stores it in goal_pos_tol_. Here we only shape the radial zone using
  // the current value of goal_pos_tol_ as stop radius.
  stop_r = std::max(0.0, goal_pos_tol_);
  slow_r = std::max(slow_radius_, stop_r + 0.05);

  gamma_slow = 1.0;
  if (dist_xy_goal < slow_r) {
    gamma_slow = std::clamp((dist_xy_goal - stop_r) / std::max(1e-3, slow_r - stop_r), 0.0, 1.0);
  }
}

void
SerestController::safety_limits(
  const NavState & nav_state,
  const RefKinematics & rk,
  double & d_closest, double & v_safe, double & v_curv)
{
  d_closest = closest_obstacle_distance(nav_state);
  v_safe = v_safe_from_distance(d_closest, /*slope_sin=*/0.0);

  // Curvature-based limit ("soft" version of the original file)
  const double ak = std::fabs(rk.kappa_hat);
  double v_curv_soft = std::numeric_limits<double>::infinity();
  if (ak > 1e-6) {
    v_curv_soft = std::min(max_angular_speed_ / ak, std::sqrt(a_lat_soft_ / ak));
  }
  v_curv = std::min(max_linear_speed_, v_curv_soft);
}

void
SerestController::apply_corner_guard(
  const RefKinematics & rk, double e_y, double e_theta,
  double & v_prog_ref, double & omega_boost, double & ey_apex_term) const
{
  double alpha_corner = 1.0;
  omega_boost = 1.0;
  ey_apex_term = 0.0;

  if (!corner_guard_enable_) {
    v_prog_ref *= alpha_corner;
    return;
  }

  const double sgn_k = (rk.kappa_hat >= 0.0) ? 1.0 : -1.0;
  const double e_y_out = std::max(0.0, sgn_k * e_y);
  const double eth = std::fabs(e_theta);
  const double kap = std::fabs(rk.kappa_hat);

  const double penal = corner_gain_ey_ * e_y_out +
    corner_gain_eth_ * eth +
    corner_gain_kappa_ * kap;

  alpha_corner = std::clamp(1.0 / (1.0 + penal), corner_min_alpha_, 1.0);
  omega_boost = 1.0 + corner_boost_omega_ * std::min(1.0, e_y_out / std::max(1e-3, ell_));

  // Push towards the "apex" if you are outside and there is curvature
  if (std::fabs(rk.kappa_hat) > 1e-6 && e_y_out > 0.0) {
    const double e_y_des = -sgn_k * std::max(0.0, apex_ey_des_);
    const double e_y_err = e_y - e_y_des;
    ey_apex_term = -k_y_ * std::atan(e_y_err / std::max(ell_, 1e-3));
  }

  v_prog_ref *= alpha_corner;
}

bool
SerestController::should_turn_in_place(
  bool allow_reverse, double e_theta, double e_theta_goal,
  double dist_to_end, [[maybe_unused]] double turn_in_place_thr) const
{
  // Keep compatibility with the signature, but ignore turn_in_place_thr
  // and use two internal thresholds without exposing parameters.
  const double thr_enter = 60.0 * M_PI / 180.0; // enter TiP if |e_theta| > 60°
  // const double thr_exit = 35.0 * PI / 180.0; // exit TiP if |e_theta| < 35°

  // Do not allow reverse "shortcut" in this decision: if reverse is not allowed,
  // the criterion is stricter.
  bool tip = (!allow_reverse) && (std::fabs(e_theta) > thr_enter);

  // Near the end, if the yaw error to the goal is large, also request TiP
  if (dist_to_end < 0.50) {
    if (!allow_reverse && std::fabs(e_theta_goal) > thr_enter) {
      tip = true;
    }
  }

  return tip;
}

bool
SerestController::maybe_final_align_and_publish(
  NavState & nav_state, const nav_msgs::msg::Path & path,
  double dist_xy_goal, double stop_r, double e_theta_goal,
  double gamma_slow, double dt)
{
  const double goal_yaw_tol = goal_yaw_tol_deg_ * M_PI / 180.0;

  if (dist_xy_goal > stop_r) {
    return false;
  }

  // En stop-zone: vlin = 0 siempre
  double vlin = 0.0;
  double vrot = 0.0;
  bool in_final_align = false;
  bool arrived = false;

  if (std::fabs(e_theta_goal) <= goal_yaw_tol) {
    vrot = 0.0;
    arrived = true;
  } else {
    in_final_align = true;
    double w_cmd = -final_align_k_ * e_theta_goal;
    if (w_cmd > final_align_wmax_) {w_cmd = final_align_wmax_;}
    if (w_cmd < -final_align_wmax_) {w_cmd = -final_align_wmax_;}

    const double max_dvrot = max_angular_acc_ * dt;
    w_cmd = std::clamp(w_cmd, last_vrot_ - max_dvrot, last_vrot_ + max_dvrot);
    vrot = w_cmd;
  }

  // Publicación y actualización de estado
  twist_stamped_.header.frame_id = path.header.frame_id;
  twist_stamped_.header.stamp = last_input_ts_;
  twist_stamped_.twist.linear.x = vlin;
  twist_stamped_.twist.angular.z = vrot;

  last_vlin_ = vlin;
  last_vrot_ = vrot;

  nav_state.set("cmd_vel", twist_stamped_);
  nav_state.set("serest.debug.goal.dist_xy", dist_xy_goal);
  nav_state.set("serest.debug.goal.gamma_slow", gamma_slow);
  nav_state.set("serest.debug.goal.in_final_align", static_cast<int>(in_final_align));
  nav_state.set("serest.debug.goal.arrived", static_cast<int>(arrived));
  return true;
}

void
SerestController::rate_limit_and_saturate(double dt, double & vlin, double & vrot)
{
  const double max_dvlin = max_linear_acc_ * dt;
  const double max_dvrot = max_angular_acc_ * dt;

  vlin = std::clamp(vlin, last_vlin_ - max_dvlin, last_vlin_ + max_dvlin);
  vrot = std::clamp(vrot, last_vrot_ - max_dvrot, last_vrot_ + max_dvrot);

  if (allow_reverse_) {
    vlin = std::clamp(vlin, -max_linear_speed_, max_linear_speed_);
  } else {
    vlin = std::clamp(vlin, 0.0, max_linear_speed_);
  }
  vrot = std::clamp(vrot, -max_angular_speed_, max_angular_speed_);
}

void
SerestController::publish_cmd_and_debug(
  NavState & nav_state, const nav_msgs::msg::Path & path,
  double vlin, double vrot,
  double e_y, double e_theta, double kappa_hat,
  double d_closest, double v_safe, double v_curv, double alpha,
  bool allow_reverse, double dist_to_end,
  double dist_xy_goal, double gamma_slow,
  int in_final_align, int arrived)
{
  twist_stamped_.header.frame_id = path.header.frame_id;
  twist_stamped_.header.stamp = last_input_ts_;
  twist_stamped_.twist.linear.x = vlin;
  twist_stamped_.twist.angular.z = vrot;
  nav_state.set("cmd_vel", twist_stamped_);

  nav_state.set("serest.debug.e_y", e_y);
  nav_state.set("serest.debug.e_theta", e_theta);
  nav_state.set("serest.debug.kappa_hat", kappa_hat);
  nav_state.set("serest.debug.d_closest", d_closest);
  nav_state.set("serest.debug.v_safe", v_safe);
  nav_state.set("serest.debug.v_curv", v_curv);
  nav_state.set("serest.debug.alpha", alpha);
  nav_state.set("serest.debug.allow_reverse", static_cast<int>(allow_reverse));
  nav_state.set("serest.debug.dist_to_end", dist_to_end);
  nav_state.set("serest.debug.goal.dist_xy", dist_xy_goal);
  nav_state.set("serest.debug.goal.gamma_slow", gamma_slow);
  nav_state.set("serest.debug.goal.in_final_align", in_final_align);
  nav_state.set("serest.debug.goal.arrived", arrived);
}

void
SerestController::publish_stop(NavState & nav_state, const std::string & frame_id)
{
  auto now = last_input_ts_;
  twist_stamped_.header.frame_id = frame_id;
  twist_stamped_.header.stamp = now;
  twist_stamped_.twist.linear.x = 0.0;
  twist_stamped_.twist.angular.z = 0.0;
  nav_state.set("cmd_vel", twist_stamped_);
}

void
SerestController::update_rt(NavState & nav_state)
{
  // 0) Time step
  const double dt = compute_dt_and_update_clock();

  // 1) Required inputs
  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odom;
  if (!fetch_required_inputs(nav_state, path, odom)) {return;}

  // 1.5) Goal tolerances: prefer shared GoalManager values, fallback to local params
  double goal_pos_tol = goal_pos_tol_;
  double goal_yaw_tol = goal_yaw_tol_deg_ * (M_PI / 180.0);
  if (nav_state.has("goal_tolerance.position")) {
    goal_pos_tol = nav_state.get<double>("goal_tolerance.position");
  }
  if (nav_state.has("goal_tolerance.yaw")) {
    goal_yaw_tol = nav_state.get<double>("goal_tolerance.yaw");
  }

  // 2) Robot state (position + yaw)
  Vec2 robot_xy; double yaw = 0.0;
  robot_state_from_odom(odom, robot_xy, yaw);

  // 3) Path primitives, closest-point projection, and local reference kinematics
  PathData pd = build_path_data(path);
  Projection prj = project_on_path(pd, robot_xy);
  RefKinematics rk = ref_heading_and_curvature(pd, prj, last_vlin_);

  // 4) Frenet errors: lateral offset and heading error w.r.t. reference tangent
  double e_y = 0.0, e_theta = 0.0;
  frenet_errors(robot_xy, yaw, prj, rk.psi_ref, e_y, e_theta);

  // 5) Goal-related terms and slow/stop zone shaping
  double dist_xy_goal = 0.0, e_theta_goal = 0.0, stop_r = 0.0, slow_r = 0.0, gamma_slow = 1.0;
  Vec2 goal_xy; double yaw_goal = 0.0;
  compute_goal_zone(path, robot_xy, yaw, dist_xy_goal, e_theta_goal,
                    stop_r, slow_r, gamma_slow, goal_xy, yaw_goal);

  // 6) Global safety limits derived from sensors and curvature
  double d_closest = 0.0, v_safe = 0.0, v_curv = 0.0;
  safety_limits(nav_state, rk, d_closest, v_safe, v_curv);

  // 6.5) Early turn-in-place with hysteresis (start-of-path aware)
  {
    const double s_total = pd.s_acc.back();
    const double dist_to_end = s_total - prj.s_star;

    // Internal angular thresholds (enter/exit)
    const double thr_enter = 60.0 * M_PI / 180.0;
    const double thr_exit = 35.0 * M_PI / 180.0;
    const double near_start_s = 0.30;  // treat the first 30 cm as the start region

    // Base request from the regular criterion (using the provided threshold)
    bool tip_request = should_turn_in_place(allow_reverse_, e_theta, e_theta_goal, dist_to_end,
      thr_enter);

    // Additional start-of-path gate: enforce TiP if still near s*=0 and yaw misalignment is large
    if (!allow_reverse_ && prj.s_star < near_start_s && std::fabs(e_theta) > thr_enter) {
      tip_request = true;
    }

    // Hysteresis on the TiP state
    if (!tip_active_ && tip_request && std::fabs(e_theta) > thr_enter) {
      tip_active_ = true;
    } else if (tip_active_ &&
      (std::fabs(e_theta) < thr_exit || prj.s_star > (near_start_s - 0.10)))
    {
      tip_active_ = false;
    }

    // TiP execution branch: publish v=0 and a bounded angular command, then return
    if (tip_active_) {
      double w_cmd = -k_theta_ * e_theta -
        k_y_ * std::atan(e_y / std::max(ell_, 1e-3));

      const double gamma_omega = std::max(0.25, gamma_slow);
      w_cmd *= gamma_omega;

      // Angular acceleration limit
      const double max_dvrot = max_angular_acc_ * dt;
      w_cmd = std::clamp(w_cmd, last_vrot_ - max_dvrot, last_vrot_ + max_dvrot);

      // Angular speed saturation
      double vlin = 0.0;
      double vrot = std::clamp(w_cmd, -max_angular_speed_, max_angular_speed_);

      // Persist current command to avoid linear “drag” on the next cycle
      last_vlin_ = vlin;
      last_vrot_ = vrot;

      // Publish early; avoid any later block reopening v > 0
      publish_cmd_and_debug(
        nav_state, path, vlin, vrot,
        e_y, e_theta, rk.kappa_hat,
        d_closest, v_safe, v_curv, /*alpha*/1.0,
        allow_reverse_, dist_to_end,
        dist_xy_goal, gamma_slow,
        /*in_final_align*/0, /*arrived*/0);
      return;
    }
  }

  // 7) SeReST-LGS nominal tracking
  const double cos_et = std::cos(e_theta);
  const double cos_et_pos = std::max(0.0, cos_et);
  const double denom = std::max(1e-3, 1.0 - rk.kappa_hat * e_y);

  // Progress reference limited by curvature, safety and configured v_ref
  double v_prog_ref_free = std::min({max_linear_speed_, v_curv, v_safe, v_ref_});
  double v_prog_ref = v_prog_ref_free * gamma_slow;

  // Maintain a small cruising speed when roughly aligned and outside the stop zone (no reverse)
  const double align_thr = 30.0 * M_PI / 180.0;
  if (!allow_reverse_ && (dist_xy_goal > stop_r) && std::fabs(e_theta) < align_thr) {
    v_prog_ref = std::max(v_prog_ref, std::min(slow_min_speed_, v_prog_ref_free));
  }

  // Corner guard: reduce progress and optionally boost omega near tight corners
  double omega_boost = 1.0, ey_apex_term = 0.0;
  apply_corner_guard(rk, e_y, e_theta, v_prog_ref, omega_boost, ey_apex_term);

  // Nominal progress along the path with bounded lateral “suction”
  double forward_term = ((allow_reverse_ ? std::fabs(cos_et) : cos_et_pos) * v_prog_ref) / denom;
  double lateral_term = k_s_ * e_y;
  double lateral_cap = k_s_share_max_ * std::max(1e-3, forward_term);
  lateral_term = std::clamp(lateral_term, -lateral_cap, lateral_cap);
  double s_dot_nom = forward_term - lateral_term;

  // Nominal angular rate: curvature feedforward + heading/lateral corrective terms
  double omega_nom = rk.kappa_hat * s_dot_nom -
    k_theta_ * e_theta -
    k_y_ * std::atan(e_y / std::max(ell_, 1e-3)) +
    ey_apex_term;

  const double gamma_omega = std::max(0.25, gamma_slow);
  omega_nom *= (omega_boost * gamma_omega);

  // 8) Final alignment inside stop zone (publishes and returns if active)
  if (maybe_final_align_and_publish(
        nav_state, path, dist_xy_goal, stop_r, e_theta_goal, gamma_slow, dt))
  {
    return;
  }

  // 9) Time reparameterization and reconstruction of linear speed
  const double alpha = std::min(1.0, (v_ref_ > 1e-6) ? (v_safe / v_ref_) : 1.0);
  double s_dot = allow_reverse_ ? (alpha * s_dot_nom) : std::max(0.0, alpha * s_dot_nom);
  const double cos_for_v = allow_reverse_ ?
    std::max(1e-3, std::fabs(cos_et)) :
    std::max(1e-3, cos_et_pos);
  double v_track = s_dot * denom / cos_for_v;

  double v_cmd_raw = 0.0;
  if (allow_reverse_) {
    const double v_mag_limit = std::min(v_curv, v_safe);
    v_cmd_raw = std::clamp(v_track, -v_mag_limit, v_mag_limit);
  } else {
    v_cmd_raw = std::min({v_track, v_curv, v_safe, max_linear_speed_});
    v_cmd_raw = std::max(0.0, v_cmd_raw);
  }

  // Optional classic TiP safeguard using the same angular threshold
  {
    const double turn_in_place_thr = (60.0 * M_PI / 180.0);
    const double s_total = pd.s_acc.back();
    const double dist_to_end = s_total - prj.s_star;

    if (should_turn_in_place(allow_reverse_, e_theta, e_theta_goal, dist_to_end,
      turn_in_place_thr))
    {
      v_cmd_raw = 0.0;
      omega_nom = -k_theta_ * e_theta - k_y_ * std::atan(e_y / std::max(ell_, 1e-3));
      omega_nom *= gamma_omega;
    }
  }

  // 10) Emergency override (hard distance or time-to-collision condition)
  const double v_prev = last_vlin_;
  double vlin = v_cmd_raw;
  double vrot = omega_nom + rk.kappa_hat * (s_dot - s_dot_nom);

  const bool emerg = (d_closest <= d_hard_) ||
    (d_closest / std::max(std::fabs(v_prev), 1e-3) <= t_emerg_);
  if (emerg) {
    if (v_prev > 0.0) {
      vlin = std::max(0.0, v_prev - a_brake_ * dt);
    } else {
      vlin = std::min(0.0, v_prev + a_brake_ * dt);
    }
    vrot = 0.0;
  }

  // 11) Rate limiting and saturations
  rate_limit_and_saturate(dt, vlin, vrot);
  last_vlin_ = vlin;
  last_vrot_ = vrot;

  // 12) Publication and debug values
  const double s_total = pd.s_acc.back();
  const double dist_to_end = s_total - prj.s_star;
  publish_cmd_and_debug(
    nav_state, path, vlin, vrot,
    e_y, e_theta, rk.kappa_hat,
    d_closest, v_safe, v_curv, alpha,
    allow_reverse_, dist_to_end,
    dist_xy_goal, gamma_slow,
    /*in_final_align=*/0, /*arrived=*/0);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SerestController, easynav::ControllerMethodBase)
