// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the SimpleController class.

#include <algorithm>
#include <limits>
#include <cmath>

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "easynav_serest_controller/SerestController.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace easynav
{


SerestController::SerestController() = default;
SerestController::~SerestController() = default;

std::expected<void, std::string>
SerestController::on_initialize()
{
  auto node = get_node();
  const auto & ns = get_plugin_name();

  // Máximos y límites básicos
  node->declare_parameter<bool>(ns + ".allow_reverse", allow_reverse_);
  node->declare_parameter<double>(ns + ".v_progress_min", v_progress_min_);
  node->declare_parameter<double>(ns + ".k_s_share_max", k_s_share_max_);

  node->declare_parameter<double>(ns + ".max_linear_speed", max_linear_speed_);
  node->declare_parameter<double>(ns + ".max_angular_speed", max_angular_speed_);
  node->declare_parameter<double>(ns + ".max_linear_acc", max_linear_acc_);
  node->declare_parameter<double>(ns + ".max_angular_acc", max_angular_acc_);

  // Seguimiento
  node->declare_parameter<double>(ns + ".k_s", k_s_);
  node->declare_parameter<double>(ns + ".k_theta", k_theta_);
  node->declare_parameter<double>(ns + ".k_y", k_y_);
  node->declare_parameter<double>(ns + ".ell", ell_);
  node->declare_parameter<double>(ns + ".v_ref", v_ref_);
  node->declare_parameter<double>(ns + ".eps", eps_);

  // Seguridad
  node->declare_parameter<double>(ns + ".a_acc", a_acc_);
  node->declare_parameter<double>(ns + ".a_brake", a_brake_);
  node->declare_parameter<double>(ns + ".a_lat_max", a_lat_max_);
  node->declare_parameter<double>(ns + ".d0_margin", d0_margin_);
  node->declare_parameter<double>(ns + ".tau_latency", tau_latency_);
  node->declare_parameter<double>(ns + ".d_hard", d_hard_);
  node->declare_parameter<double>(ns + ".t_emerg", t_emerg_);

  // Blend en vértices
  node->declare_parameter<double>(ns + ".blend_base", blend_base_);
  node->declare_parameter<double>(ns + ".blend_k_per_v", blend_k_per_v_);
  node->declare_parameter<double>(ns + ".kappa_max", kappa_max_);

  // For obstacle detection
  node->declare_parameter<double>(ns + ".dist_search_radius",  dist_search_radius_);

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

  node->get_parameter<double>(ns + ".dist_search_radius",  dist_search_radius_);

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

  return {};
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

  // Índices de segmentos relevante: i-1, i, i + 1
  const size_t i = prj.seg_idx;
  const Vec2 Ti = seg_dir(i);
  double psi_i = atan2dir(Ti);

  // Mezcla local con previsualización
  double b = std::max(0.1, blend_base_ + blend_k_per_v_ * std::fabs(v_prev));

  // Determina si estamos cerca del comienzo o fin de un segmento
  double s_i = pd.s_acc[i];
  double s_ip1 = pd.s_acc[i + 1];
  double s = prj.s_star;

  // Por defecto: rumbo constante del segmento, kappa = 0
  rk.psi_ref = psi_i;
  rk.kappa_hat = 0.0;

  // Blend cerca del inicio del segmento i (con i-1)
  if (i > 0 && (s - s_i) < b) {
    Vec2 Tim1 = seg_dir(i - 1);
    double psi_im1 = atan2dir(Tim1);
    double w = 1.0 / (1.0 + std::exp(-( (s - s_i) / b ))); // sigmoide en [s_i, s_i+b]
    // Interpolación circular de rumbos
    double sx = (1.0 - w) * std::cos(psi_im1) + w * std::cos(psi_i);
    double sy = (1.0 - w) * std::sin(psi_im1) + w * std::sin(psi_i);
    rk.psi_ref = std::atan2(sy, sx);

    // kappa surrogate ~ dpsi/ds ≈ (Δψ/b) σ'(·)
    double dpsi = std::atan2(std::sin(psi_i - psi_im1), std::cos(psi_i - psi_im1));
    double sigma_prime = (w * (1 - w)) / b; // derivada de logística escalada
    rk.kappa_hat = std::clamp(dpsi * sigma_prime, -kappa_max_, kappa_max_);
  }

  // Blend cerca del final del segmento i (hacia i + 1)
  if (i + 1 < pd.pts.size() - 1 && (s_ip1 - s) < b) {
    Vec2 Tip1 = seg_dir(i + 1);
    double psi_ip1 = atan2dir(Tip1);
    double w = 1.0 / (1.0 + std::exp(-( (s_ip1 - s) / b ))); // simétrico
    // Mezcla entre i y i + 1
    double sx = (1.0 - w) * std::cos(psi_i) + w * std::cos(psi_ip1);
    double sy = (1.0 - w) * std::sin(psi_i) + w * std::sin(psi_ip1);
    rk.psi_ref = std::atan2(sy, sx);

    double dpsi = std::atan2(std::sin(psi_ip1 - psi_i), std::cos(psi_ip1 - psi_i));
    double sigma_prime = (w * (1 - w)) / b;
    double kappa2 = std::clamp(dpsi * sigma_prime, -kappa_max_, kappa_max_);
    // Si hay dos blends solapados, combinamos suavemente (promedio)
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
  // Vector normal a la derecha de la tangente
  Vec2 T = v2(std::cos(psi_ref), std::sin(psi_ref));
  Vec2 N = v2(-T.y, T.x); // 90º a la izquierda (convención)
  Vec2 err = robot_xy - prj.closest;

  e_y = dot(err, N); // distancia lateral con signo
  e_theta = std::atan2(std::sin(robot_yaw - psi_ref), std::cos(robot_yaw - psi_ref));
}

double
SerestController::closest_obstacle_distance(
  const NavState & nav_state) const
{
  // 1) Preferir medición directa si existe
  if (nav_state.has("closest_obstacle_distance")) {
    try {
      return nav_state.get<double>("closest_obstacle_distance");
    } catch (...) {
      // continúa a estimación
    }
  }

  // 2) Analizar los sensores de distancia
  if (!nav_state.has("points")) {return std::numeric_limits<double>::infinity();}

  const auto perceptions = nav_state.get<PointPerceptions>("points");
  auto fused = PointPerceptionsOpsView(perceptions)
    .downsample(0.3)
    .fuse(get_tf_prefix() + "base_link")
    ->filter({-dist_search_radius_, -dist_search_radius_, NAN},
      {dist_search_radius_, dist_search_radius_, 2.0})
    .collapse({NAN, NAN, 0.1})
    ->downsample(0.3)
    .as_points();

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
  // a_eff reduce la capacidad de frenado en pendiente; en 2D slope_sin = 0
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
  // Tres límites típicos: v_max, ω_max/|κ| y sqrt(a_lat_max/|κ|)
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

void
SerestController::update_rt(NavState & nav_state)
{
  auto now = get_node()->now();
  double dt = (now - last_update_ts_).seconds();
  if (dt <= 0.0) {dt = 1.0 / 30.0;}        // robustez si los stamps no avanzan
  last_update_ts_ = now;

  // --- Entradas mínimas requeridas ---
  if (!nav_state.has("path") || !nav_state.has("robot_pose") || !nav_state.has("map.dynamic")) {
    // Sin datos: parar
    twist_stamped_.header.frame_id = "base_link";
    twist_stamped_.header.stamp = now;
    twist_stamped_.twist.linear.x = 0.0;
    twist_stamped_.twist.angular.z = 0.0;
    nav_state.set("cmd_vel", twist_stamped_);
    return;
  }

  const auto path = nav_state.get<nav_msgs::msg::Path>("path");
  const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

  // Parada si no hay trayectoria
  if (path.poses.empty()) {
    twist_stamped_.header.frame_id = path.header.frame_id;
    twist_stamped_.header.stamp = now;
    twist_stamped_.twist.linear.x = 0.0;
    twist_stamped_.twist.angular.z = 0.0;
    nav_state.set("cmd_vel", twist_stamped_);
    return;
  }

  // --- 1) Estado del robot ---
  const auto & rp = odom.pose.pose.position;
  const auto & rq = odom.pose.pose.orientation;
  const double yaw = tf2::getYaw(rq);
  const Vec2 robot_xy{rp.x, rp.y};

  // --- 2) Polilínea y proyección ---
  PathData pd = build_path_data(path);
  Projection prj = project_on_path(pd, robot_xy);

  // --- 3) Rumbo de referencia y curvatura surrogate (LGS con preview) ---
  RefKinematics rk = ref_heading_and_curvature(pd, prj, last_vlin_);

  // --- 4) Errores de Frenet discretos respecto a la ruta ---
  double e_y = 0.0, e_theta = 0.0;
  frenet_errors(robot_xy, yaw, prj, rk.psi_ref, e_y, e_theta);

  // --- 5) Info del goal y “zona lenta” ---
  const double PI = 3.14159265358979323846;
  const double goal_yaw_tol = goal_yaw_tol_deg_ * PI / 180.0;

  const auto & pgoal = path.poses.back().pose.position;
  const double yaw_goal = tf2::getYaw(path.poses.back().pose.orientation);
  const Vec2 goal_xy{pgoal.x, pgoal.y};

  const double dist_xy_goal = std::hypot(robot_xy.x - goal_xy.x, robot_xy.y - goal_xy.y);
  const double e_theta_goal = std::atan2(std::sin(yaw - yaw_goal), std::cos(yaw - yaw_goal));

  // “Stop-zone” y “Slow-zone”
  const double stop_r = std::max(0.0, goal_pos_tol_);                 // donde ya no avanzamos linealmente
  const double slow_r = std::max(slow_radius_, stop_r + 0.05);        // empieza a frenar antes de llegar
  double gamma_slow = 1.0;                                            // factor [0..1] para tapering
  if (dist_xy_goal < slow_r) {
    gamma_slow = std::clamp((dist_xy_goal - stop_r) / std::max(1e-3, slow_r - stop_r), 0.0, 1.0);
  }

  // --- 6) Límites de seguridad globales (antes del nominal) ---
  const double d_closest = closest_obstacle_distance(nav_state);
  const double v_safe = v_safe_from_distance(d_closest, /*slope_sin=*/0.0);

  // Límite por curvatura con aceleración lateral “blanda” (más conservador en giros)
  const double ak = std::fabs(rk.kappa_hat);
  double v_curv_soft = std::numeric_limits<double>::infinity();
  if (ak > 1e-6) {
    v_curv_soft = std::min(max_angular_speed_ / ak, std::sqrt(a_lat_soft_ / ak));
  }
  const double v_curv = std::min(max_linear_speed_, v_curv_soft);

  // --- 7) Control nominal SeReST-LGS (con empuje robusto + slow-zone + corner-guard) ---
  const double cos_et = std::cos(e_theta);
  const double cos_et_pos = std::max(0.0, cos_et);                  // evita “caminar hacia atrás”
  const double denom = std::max(1e-3, 1.0 - rk.kappa_hat * e_y);

  // 7.1 Progreso deseado independiente de last_vlin_, con tapering por slow-zone
  double v_prog_ref_free = std::min({max_linear_speed_, v_curv, v_safe, v_ref_});
  double v_prog_ref = v_prog_ref_free * gamma_slow;

  // Mínimo de crucero si estamos alineados y aún fuera de la stop-zone (forward-only)
  const double align_thr = 30.0 * PI / 180.0; // 30°
  if (!allow_reverse_ && (dist_xy_goal > stop_r) && std::fabs(e_theta) < align_thr) {
    v_prog_ref = std::max(v_prog_ref, std::min(slow_min_speed_, v_prog_ref_free));
  }

  // --- Corner guard: recorte de velocidad y boost angular cuando te abres por fuera de la curva ---
  double alpha_corner = 1.0;
  double omega_boost = 1.0;

  if (corner_guard_enable_) {
    const double sgn_k = (rk.kappa_hat >= 0.0) ? 1.0 : -1.0;
    const double e_y_out = std::max(0.0, sgn_k * e_y);          // solo si estás yéndote hacia afuera
    const double eth = std::fabs(e_theta);
    const double kap = std::fabs(rk.kappa_hat);

    const double penal = corner_gain_ey_ * e_y_out +
      corner_gain_eth_ * eth +
      corner_gain_kappa_ * kap;

    alpha_corner = std::clamp(1.0 / (1.0 + penal), corner_min_alpha_, 1.0);
    omega_boost = 1.0 + corner_boost_omega_ * std::min(1.0, e_y_out / std::max(1e-3, ell_));
  }

  // Aplica corner guard a la referencia de velocidad
  v_prog_ref *= alpha_corner;

  // 7.2 Avance nominal por la ruta y “succión” lateral limitada
  double forward_term = ((allow_reverse_ ? std::fabs(cos_et) : cos_et_pos) * v_prog_ref) / denom;

  double lateral_term = k_s_ * e_y;
  double lateral_cap = k_s_share_max_ * std::max(1e-3, forward_term);
  if (lateral_term > lateral_cap) {lateral_term = lateral_cap;}
  if (lateral_term < -lateral_cap) {lateral_term = -lateral_cap;}

  double s_dot_nom = forward_term - lateral_term;

  // 7.3 Turn-in-place (solo si NO permitimos reverse)
  const double turn_in_place_thr = (80.0 * PI / 180.0);
  bool turn_in_place = (!allow_reverse_) && (std::fabs(e_theta) > turn_in_place_thr);

  // 7.4 Región terminal: si estás muy cerca, prioriza orientación al final
  const double s_total = pd.s_acc.back();
  const double dist_to_end = s_total - prj.s_star;
  if (dist_to_end < 0.50 && !path.poses.empty()) {
    if (!allow_reverse_ && std::fabs(e_theta_goal) > turn_in_place_thr) {turn_in_place = true;}
  }

  // 7.5 Objetivo lateral de “ápice”: empuja hacia dentro si estás por fuera
  double ey_apex_term = 0.0;
  if (corner_guard_enable_ && std::fabs(rk.kappa_hat) > 1e-6) {
    const double sgn_k = (rk.kappa_hat >= 0.0) ? 1.0 : -1.0;
    const double e_y_out = std::max(0.0, sgn_k * e_y);
    if (e_y_out > 0.0) {
      const double e_y_des = -sgn_k * std::max(0.0, apex_ey_des_);  // dentro de la curva
      const double e_y_err = e_y - e_y_des;
      ey_apex_term = -k_y_ * std::atan(e_y_err / std::max(ell_, 1e-3));
    }
  }

  // 7.6 Giro nominal (con boost angular y taper angular cercano al goal)
  double omega_nom = rk.kappa_hat * s_dot_nom -
    k_theta_ * e_theta -
    k_y_ * std::atan(e_y / std::max(ell_, 1e-3)) +
    ey_apex_term;

  const double gamma_omega = std::max(0.25, gamma_slow);  // amortigua giro en slow-zone
  omega_nom *= (omega_boost * gamma_omega);

  // --- 8) Fase de alineación final en el goal (stop-zone) ---
  bool in_final_align = false;
  bool arrived = false;

  if (dist_xy_goal <= stop_r) {
    // No avanzar linealmente en la stop-zone
    double vlin = 0.0;
    double vrot;

    if (std::fabs(e_theta_goal) <= goal_yaw_tol) {
      // ¡Llegado!
      vrot = 0.0;
      arrived = true;

      twist_stamped_.header.frame_id = path.header.frame_id;
      twist_stamped_.header.stamp = now;
      twist_stamped_.twist.linear.x = vlin;
      twist_stamped_.twist.angular.z = vrot;

      last_vlin_ = vlin;
      last_vrot_ = vrot;

      nav_state.set("cmd_vel", twist_stamped_);
      nav_state.set("serest.debug.goal.dist_xy", dist_xy_goal);
      nav_state.set("serest.debug.goal.gamma_slow", gamma_slow);
      nav_state.set("serest.debug.goal.in_final_align", static_cast<int>(in_final_align));
      nav_state.set("serest.debug.goal.arrived", static_cast<int>(arrived));
      return;
    } else {
      // Alinear orientación final
      in_final_align = true;
      double w_cmd = -final_align_k_ * e_theta_goal;
      // Limitar velocidad angular en esta fase
      if (w_cmd > final_align_wmax_) {w_cmd = final_align_wmax_;}
      if (w_cmd < -final_align_wmax_) {w_cmd = -final_align_wmax_;}

      // Rate limiting
      const double max_dvrot = max_angular_acc_ * dt;
      w_cmd = std::clamp(w_cmd, last_vrot_ - max_dvrot, last_vrot_ + max_dvrot);

      // Publica y sale (evitamos el resto del pipeline aquí)
      twist_stamped_.header.frame_id = path.header.frame_id;
      twist_stamped_.header.stamp = now;
      twist_stamped_.twist.linear.x = vlin;
      twist_stamped_.twist.angular.z = w_cmd;

      last_vlin_ = vlin;
      last_vrot_ = w_cmd;

      nav_state.set("cmd_vel", twist_stamped_);
      // Debug
      nav_state.set("serest.debug.goal.dist_xy", dist_xy_goal);
      nav_state.set("serest.debug.goal.gamma_slow", gamma_slow);
      nav_state.set("serest.debug.goal.in_final_align", static_cast<int>(in_final_align));
      nav_state.set("serest.debug.goal.arrived", static_cast<int>(arrived));
      return;
    }
  }

  // --- 9) Reparametrización temporal (α) y reconstrucción de v ---
  const double alpha = std::min(1.0, (v_ref_ > 1e-6) ? (v_safe / v_ref_) : 1.0);
  double s_dot = allow_reverse_ ? (alpha * s_dot_nom) : std::max(0.0, alpha * s_dot_nom);

  const double cos_for_v = allow_reverse_ ? std::max(1e-3, std::fabs(cos_et)) :
    std::max(1e-3, cos_et_pos);

  double v_track = s_dot * denom / cos_for_v;

  // Comando lineal bruto y límites (con tapering ya aplicado en v_prog_ref)
  double v_cmd_raw;
  if (allow_reverse_) {
    const double v_mag_limit = std::min(v_curv, v_safe);
    v_cmd_raw = std::clamp(v_track, -v_mag_limit, v_mag_limit);
  } else {
    v_cmd_raw = std::min({v_track, v_curv, v_safe, max_linear_speed_});
    v_cmd_raw = std::max(0.0, v_cmd_raw);
  }

  // Modo giro en el sitio (solo forward-only)
  if (turn_in_place) {
    v_cmd_raw = 0.0;
    omega_nom = -k_theta_ * e_theta - k_y_ * std::atan(e_y / std::max(ell_, 1e-3));
    omega_nom *= gamma_omega; // amortigua si ya estamos en slow-zone
  }

  // --- 10) Emergencia (parada por colisión inminente) ---
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

  // --- 11) Rate limiting y saturaciones finales ---
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

  last_vlin_ = vlin;
  last_vrot_ = vrot;

  // --- 12) Publicación y depuración ---
  twist_stamped_.header.frame_id = path.header.frame_id;
  twist_stamped_.header.stamp = now;
  twist_stamped_.twist.linear.x = vlin;
  twist_stamped_.twist.angular.z = vrot;
  nav_state.set("cmd_vel", twist_stamped_);

  nav_state.set("serest.debug.e_y", e_y);
  nav_state.set("serest.debug.e_theta", e_theta);
  nav_state.set("serest.debug.kappa_hat", rk.kappa_hat);
  nav_state.set("serest.debug.d_closest", d_closest);
  nav_state.set("serest.debug.v_safe", v_safe);
  nav_state.set("serest.debug.v_curv", v_curv);
  nav_state.set("serest.debug.alpha", alpha);
  nav_state.set("serest.debug.allow_reverse", static_cast<int>(allow_reverse_));
  nav_state.set("serest.debug.dist_to_end", dist_to_end);
  // Debug de goal
  nav_state.set("serest.debug.goal.dist_xy", dist_xy_goal);
  nav_state.set("serest.debug.goal.gamma_slow", gamma_slow);
  nav_state.set("serest.debug.goal.in_final_align", 0);
  nav_state.set("serest.debug.goal.arrived", 0);
}


}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::SerestController, easynav::ControllerMethodBase)
