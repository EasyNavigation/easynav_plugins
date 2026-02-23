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

#ifndef EASYNAV_SEREST_CONTROLLER__SERESTCONTROLLER_HPP_
#define EASYNAV_SEREST_CONTROLLER__SERESTCONTROLLER_HPP_

#include <vector>
#include <string>

#include "rclcpp/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{


/**
 * @brief SeReST (Safe Reparameterized Time) controller for path tracking.
 *
 * This controller follows a discrete polyline path (e.g., A* on grid) using a
 * local geometric substitute (LGS) to obtain a smooth reference heading and a
 * surrogate curvature. Progress along the path is time–reparameterized by a
 * safety governor that depends on the closest obstacle distance. It supports:
 * - Forward-only or reverse-enabled operation
 * - Smooth slowdown and precise arrival at a goal pose (position + yaw)
 * - Real-time reaction to dynamic obstacles with emergency stop
 * - Corner-guard behavior to avoid opening to the outside of tight turns
 *
 * The controller consumes:
 *  - A continuously refreshed path (robot -> goal)
 *  - Robot pose (odometry)
 * and outputs: a Twist (linear, angular velocities).
 */
class SerestController : public ControllerMethodBase
{
public:
  /// @brief Default constructor.
  SerestController();
  /// @brief Destructor.
  ~SerestController() override;

  /**
   * @brief Initialize parameters and internal state.
   * @throws std::runtime_error on initialization error.
   */
  void on_initialize() override;

  /**
   * @brief Real-time control update (called ~20–30 Hz).
   *
   * Consumes @c NavState keys:
   * - "path": @c nav_msgs::msg::Path
   * - "robot_pose": @c nav_msgs::msg::Odometry
   * - (optional) "closest_obstacle_distance": @c double
   *
   * Produces:
   * - "cmd_vel": @c geometry_msgs::msg::TwistStamped
   *
   * @param[in,out] nav_state Blackboard for inputs/outputs and diagnostics.
   */
  void update_rt(NavState & nav_state) override;

  /**
   * @brief Simple 2D vector utility type.
   *
   * Provides basic arithmetic operators and a friend scalar multiplication
   * (scalar * vector).
   */
  struct Vec2
  {
    double x; ///< X component.
    double y; ///< Y component.

    /// @brief Vector addition.
    inline Vec2 operator+(const Vec2 & b) const {return {x + b.x, y + b.y};}
    /// @brief Vector subtraction.
    inline Vec2 operator-(const Vec2 & b) const {return {x - b.x, y - b.y};}
    /// @brief Scalar multiplication (vector * scalar).
    inline Vec2 operator*(double s) const {return {x * s, y * s};}

    /// @brief Scalar multiplication (scalar * vector).
    friend inline Vec2 operator*(double s, const Vec2 & a) {return {a.x * s, a.y * s};}
  };

private:
  /**
   * @brief Polyline path data with arc-length accumulator.
   */
  struct PathData
  {
    std::vector<Vec2> pts;      ///< Path points in world frame (2D).
    std::vector<double> s_acc;  ///< Arc-length cumulative per point index.
  };

  /**
   * @brief Projection of robot onto the active path segment.
   */
  struct Projection
  {
    size_t seg_idx{0};   ///< Current segment index [i, i+1].
    double s_star{0.0};  ///< Arc-length at the closest point.
    Vec2   closest{0.0, 0.0}; ///< Closest point coordinates on the path.
    double t{0.0};       ///< Segment parameter in [0,1].
  };

  /**
   * @brief Reference kinematics at the projection point.
   */
  struct RefKinematics
  {
    double psi_ref{0.0};     ///< Reference heading (rad).
    double kappa_hat{0.0};   ///< Surrogate curvature (1/m).
  };

  /**
   * @brief Build polyline data (points + arc-length accumulator) from a ROS Path.
   * @param path Discrete path as @c nav_msgs::msg::Path .
   * @return PathData Precomputed polyline representation.
   */
  PathData build_path_data(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Project a world point onto the polyline.
   * @param pd Precomputed polyline data.
   * @param p Point to project (robot position).
   * @return Projection Closest point information on the path.
   */
  Projection project_on_path(const PathData & pd, const Vec2 & p) const;

  /**
   * @brief Compute reference heading and surrogate curvature (LGS with local blending).
   * @param pd Precomputed polyline data.
   * @param prj Projection result (segment, arc-length, closest point).
   * @param v_prev Previous commanded linear velocity (for preview-dependent blend).
   * @return RefKinematics Reference heading and curvature surrogate at s*.
   */
  RefKinematics ref_heading_and_curvature(
    const PathData & pd, const Projection & prj, double v_prev) const;

  /**
   * @brief Compute discrete Frenet errors w.r.t. a reference heading at the closest point.
   * @param robot_xy Robot position in world coordinates.
   * @param robot_yaw Robot yaw (rad).
   * @param prj Projection result onto the path.
   * @param psi_ref Reference heading at s* (rad).
   * @param[out] e_y Signed lateral error (m).
   * @param[out] e_theta Heading error (rad), wrapped to [-pi, pi].
   */
  void frenet_errors(
    const Vec2 & robot_xy, double robot_yaw,
    const Projection & prj, double psi_ref,
    double & e_y, double & e_theta) const;

  /**
   * @brief Return the closest obstacle distance.
   *
   * If @c NavState contains key "closest_obstacle_distance", that value is used.
   * Otherwise, it is estimated from the sensors
   *
   * @param nav_state Blackboard accessor.
   * @return double Minimum obstacle distance (m). Infinity if none is found.
   */
  double closest_obstacle_distance(
    const NavState & nav_state);

  /**
   * @brief Compute a safe linear speed bound from obstacle distance and slope.
   * @param d Closest obstacle distance (m).
   * @param slope_sin Sine of slope angle (0 for planar 2D).
   * @return double Safe linear speed (m/s), nonnegative.
   */
  double v_safe_from_distance(double d, double slope_sin = 0.0) const;

  /**
   * @brief Curvature-based speed limit using lateral acceleration / yaw-rate bounds.
   * @param kappa_hat Surrogate curvature (1/m).
   * @return double Maximum allowed linear speed (m/s) from curvature.
   */
  double v_curvature_limit(double kappa_hat) const;

  // Gestiona dt y actualiza last_update_ts_. Devuelve dt robusto.
  double compute_dt_and_update_clock();

  // Comprueba entradas mínimas. Publica stop si faltan y devuelve false.
  // Si todo ok, rellena 'path' y 'odom' y devuelve true.
  bool fetch_required_inputs(
    NavState & nav_state,
    nav_msgs::msg::Path & path,
    nav_msgs::msg::Odometry & odom);

  // Extrae estado del robot (xy, yaw) desde la odometría.
  void robot_state_from_odom(
    const nav_msgs::msg::Odometry & odom,
    Vec2 & robot_xy, double & yaw) const;

  // Calcula información de goal y zonas slow/stop.
  void compute_goal_zone(
    const nav_msgs::msg::Path & path,
    const Vec2 & robot_xy, double robot_yaw,
    double & dist_xy_goal, double & e_theta_goal,
    double & stop_r, double & slow_r, double & gamma_slow,
    Vec2 & goal_xy, double & yaw_goal) const;

  // Calcula límites de seguridad globales (distancia obstáculo, v_safe, v_curv).
  void safety_limits(
    const NavState & nav_state,
    const RefKinematics & rk,
    double & d_closest, double & v_safe, double & v_curv);

  // Aplica corner‑guard: ajusta v_prog_ref y obtiene omega_boost + término ápice.
  void apply_corner_guard(
    const RefKinematics & rk, double e_y, double e_theta,
    double & v_prog_ref, double & omega_boost, double & ey_apex_term) const;

  // Determina si debe hacerse giro en el sitio en esta iteración.
  bool should_turn_in_place(
    bool allow_reverse, double e_theta, double e_theta_goal,
    double dist_to_end, double turn_in_place_thr) const;

  // Fase de alineación final: publica y devuelve true si se ha gestionado aquí.
  bool maybe_final_align_and_publish(
    NavState & nav_state, const nav_msgs::msg::Path & path,
    double dist_xy_goal, double stop_r, double e_theta_goal,
    double gamma_slow, double dt);

  // Aplica rate‑limit y saturaciones finales sobre (vlin, vrot) y actualiza last_*.
  void rate_limit_and_saturate(double dt, double & vlin, double & vrot);

  // Publica cmd_vel y variables de depuración comunes del final de update_rt.
  void publish_cmd_and_debug(
    NavState & nav_state, const nav_msgs::msg::Path & path,
    double vlin, double vrot,
    double e_y, double e_theta, double kappa_hat,
    double d_closest, double v_safe, double v_curv, double alpha,
    bool allow_reverse, double dist_to_end,
    double dist_xy_goal, double gamma_slow,
    int in_final_align, int arrived);

  // Publica parada inmediata con frame y stamp actuales.
  void publish_stop(NavState & nav_state, const std::string & frame_id);

  // --- Parameters ---

  /// @brief Allow reverse motion (if false, forward-only).
  bool   allow_reverse_{false};
  /// @brief Minimal forward progress (m/s) when reasonably aligned.
  double v_progress_min_{0.05};
  /// @brief Max fraction of forward progress that -k_s * e_y can cancel.
  double k_s_share_max_{0.5};

  /// @brief Maximum linear speed (m/s).
  double max_linear_speed_{0.6};
  /// @brief Maximum angular speed (rad/s).
  double max_angular_speed_{1.5};
  /// @brief Maximum linear acceleration (m/s^2).
  double max_linear_acc_{0.8};
  /// @brief Maximum angular acceleration (rad/s^2).
  double max_angular_acc_{2.0};

  /// @brief Longitudinal/lateral tracking gains and lateral scale.
  double k_s_{0.8}, k_theta_{2.0}, k_y_{1.2}, ell_{0.3};
  /// @brief Nominal reference speed (m/s) used by alpha-scaling.
  double v_ref_{0.6};
  /// @brief Small epsilon for numerical guards.
  double eps_{1e-3};

  /// @brief Comfort acceleration (redundant with max_linear_acc_ by default).
  double a_acc_{0.8};
  /// @brief Comfortable emergency braking deceleration (m/s^2).
  double a_brake_{1.2};
  /// @brief Max admissible lateral acceleration (m/s^2).
  double a_lat_max_{1.5};
  /// @brief Geometric safety margin added to obstacle distance (m).
  double d0_margin_{0.30};
  /// @brief Perception + control latency (s).
  double tau_latency_{0.10};
  /// @brief Hard stop distance threshold (m).
  double d_hard_{0.20};
  /// @brief Time-to-collision emergency threshold (s).
  double t_emerg_{0.25};

  /// @brief Base blend length for heading smoothing at vertices (m).
  double blend_base_{0.6};
  /// @brief Additional blend per |v| (s * m/s -> m).
  double blend_k_per_v_{0.6};
  /// @brief Surrogate curvature saturation (1/m).
  double kappa_max_{2.5};

  /// @brief Search radius (m) around the robot when estimating closest obstacle from the map.
  double dist_search_radius_{2.0};

  /// @brief Position tolerance (m) to consider the goal position reached.
  double goal_pos_tol_{0.05};
  /// @brief Orientation tolerance (degrees) to consider the goal yaw reached.
  double goal_yaw_tol_deg_{5.0};
  /// @brief Slowdown radius (m) to start tapering linear speed towards the goal.
  double slow_radius_{0.60};
  /// @brief Minimal linear speed (m/s) inside the slow zone (still efficient).
  double slow_min_speed_{0.03};

  /// @brief Proportional gain used during final orientation alignment.
  double final_align_k_{2.0};
  /// @brief Max angular speed (rad/s) during final alignment.
  double final_align_wmax_{0.6};

  /// @brief Enable corner-guard behavior in tight turns.
  bool  corner_guard_enable_{true};
  /// @brief Corner-guard weight for outside lateral error (e_y_out) in speed reduction.
  double corner_gain_ey_{1.5};
  /// @brief Corner-guard weight for |e_theta| in speed reduction.
  double corner_gain_eth_{0.7};
  /// @brief Corner-guard weight for |kappa_hat| in speed reduction.
  double corner_gain_kappa_{0.4};
  /// @brief Minimum alpha after corner-guard (lower bound on speed scaling).
  double corner_min_alpha_{0.35};
  /// @brief Angular boost factor based on e_y_out to increase yaw authority.
  double corner_boost_omega_{0.8};
  /// @brief Desired inner lateral bias (m) at apex to avoid opening outside.
  double apex_ey_des_{0.05};
  /// @brief Soft lateral acceleration limit (m/s^2) for curvature-based speed in corners.
  double a_lat_soft_{1.1};

  // --- Internal state ---

  bool startup_anchor_active_{true};
  geometry_msgs::msg::Pose goal_pose_last_{};
  // Activa/desactiva giro en el sitio con histéresis
  bool tip_active_{false};

  /// @brief Last commanded linear speed (m/s).
  double last_vlin_{0.0};
  /// @brief Last commanded angular speed (rad/s).
  double last_vrot_{0.0};
  /// @brief Last update timestamp.
  rclcpp::Time last_update_ts_;
  /// @brief Last input timestamp (max of path and odom).
  rclcpp::Time last_input_ts_;
  /// @brief Output TwistStamped buffer.
  geometry_msgs::msg::TwistStamped twist_stamped_;
};

/**
 * @brief Construct a 2D vector.
 * @param x X component.
 * @param y Y component.
 * @return SerestController::Vec2 The resulting vector.
 */
inline SerestController::Vec2 v2(double x, double y) {return {x, y};}

/**
 * @brief Dot product between two 2D vectors.
 * @param a First vector.
 * @param b Second vector.
 * @return double a·b
 */
inline double dot(const SerestController::Vec2 & a, const SerestController::Vec2 & b)
{
  return a.x * b.x + a.y * b.y;
}

/**
 * @brief 2D cross product Z component (a.x*b.y - a.y*b.x).
 * @param a First vector.
 * @param b Second vector.
 * @return double Scalar Z component of a × b.
 */
inline double crossz(const SerestController::Vec2 & a, const SerestController::Vec2 & b)
{
  return a.x * b.y - a.y * b.x;
}

/**
 * @brief Euclidean norm (length) of a 2D vector.
 * @param a Vector.
 * @return double ||a||
 */
inline double norm(const SerestController::Vec2 & a) {return std::hypot(a.x, a.y);}

/**
 * @brief Normalize a 2D vector, with a safe fallback for very small norms.
 * @param a Vector to normalize.
 * @return SerestController::Vec2 Unit vector (or {1,0} if norm is tiny).
 */
inline SerestController::Vec2 normalize(const SerestController::Vec2 & a)
{
  double n = norm(a);
  return n > 1e-9 ? SerestController::Vec2{a.x / n, a.y / n} : SerestController::Vec2{1.0, 0.0};
}

}  // namespace easynav

#endif  // EASYNAV_SEREST_CONTROLLER__SERESTCONTROLLER_HPP_
