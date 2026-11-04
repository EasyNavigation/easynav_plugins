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

#include "easynav_mpc_controller/MPCOptimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "Eigen/SparseCore"

namespace easynav
{
namespace mpc
{

namespace
{

using Triplets = std::vector<Eigen::Triplet<double>>;

/// Rows of l <= A x <= u, built one at a time.
struct Constraints
{
  Triplets a;
  std::vector<double> lower, upper;

  int add(double lo, double hi)
  {
    lower.push_back(lo);
    upper.push_back(hi);
    return static_cast<int>(lower.size()) - 1;
  }
  void coef(int row, int col, double value) {a.emplace_back(row, col, value);}
};

/// Bounds of a change \p nominal_delta could not leave: [lo, hi] widened to include it, so the
/// nominal trajectory (deviation 0) always satisfies them and the QP is never infeasible because
/// of the linearization point.
void widen(double nominal, double & lo, double & hi)
{
  lo = std::min(lo, nominal);
  hi = std::max(hi, nominal);
}

}  // namespace

void MPCOptimizer::configure(const MPCParams & params)
{
  params_ = params;
  layout_ = ModelLayout(params.model);
  if (static_cast<int>(params_.axes.size()) != layout_.nv) {
    params_.axes.assign(layout_.nv, AxisLimits{});
  }
  previous_.clear();
}

void MPCOptimizer::set_axes(const std::vector<AxisLimits> & axes, double min_turning_radius)
{
  if (static_cast<int>(axes.size()) == layout_.nv) {
    params_.axes = axes;
  }
  params_.min_turning_radius = min_turning_radius;
}

MPCResult MPCOptimizer::compute(const MPCInput & in)
{
  const auto start = std::chrono::steady_clock::now();
  MPCResult result;
  const int N = params_.horizon_steps;
  const int np = layout_.np, nv = layout_.nv;
  const double dt = params_.dt;
  if (!in.reference || static_cast<int>(in.reference->velocities.size()) != N ||
    in.pose.size() != np || in.velocity.size() != nv || N <= 0)
  {
    return result;
  }
  const auto & ref = *in.reference;
  const auto & axes = params_.axes;
  const bool turning_radius = params_.min_turning_radius > 0.0 && !layout_.holonomic();
  const double lag = params_.time_constant > 0.0 ? std::exp(-dt / params_.time_constant) : 0.0;

  auto clamp_axes = [&](Eigen::VectorXd u) {
      for (int j = 0; j < nv; ++j) {
        u(j) = std::clamp(u(j), axes[j].min, axes[j].max);
      }
      if (turning_radius) {
        u(0) = std::max(u(0), 0.0);
        const double w = u(0) / params_.min_turning_radius;
        u(layout_.wz) = std::clamp(u(layout_.wz), -w, w);
      }
      return u;
    };

  // Nominal commands: the last solution one step on, or the reference. Not a last solution that
  // stays well behind the reference: linearized at a standstill, the model cannot see that moving
  // lets it turn (an Ackermann robot), and the solver would keep it there.
  double previous_speed = 0.0, reference_speed = 0.0;
  for (int k = 0; k < N && static_cast<int>(previous_.size()) == N; ++k) {
    previous_speed += std::abs(previous_[k](0)) / N;
    reference_speed += std::abs(ref.velocities[k](0)) / N;
  }
  const bool warm = static_cast<int>(previous_.size()) == N &&
    previous_speed >= 0.5 * reference_speed;
  std::vector<Eigen::VectorXd> u_nom(N);
  for (int k = 0; k < N; ++k) {
    u_nom[k] = clamp_axes(warm ? previous_[std::min(k + 1, N - 1)] : ref.velocities[k]);
  }
  // Nominal trajectory: poses p[0..N], velocities v[0..N] (v[0] the robot's now)
  std::vector<Eigen::VectorXd> p_nom(N + 1), v_nom(N + 1);
  p_nom[0] = in.pose;
  v_nom[0] = in.velocity;
  for (int k = 0; k < N; ++k) {
    v_nom[k + 1] = lag * v_nom[k] + (1.0 - lag) * u_nom[k];
    p_nom[k + 1] = step(layout_, p_nom[k], v_nom[k + 1], dt);
  }

  // Variables: dp_1..N, dv_1..N, du_0..N-1
  const int nP = N * np, nV = N * nv, n = nP + nV + N * nv;
  auto iP = [&](int k) {return (k - 1) * np;};       // k = 1..N
  auto iV = [&](int k) {return nP + (k - 1) * nv;};  // k = 1..N
  auto iU = [&](int k) {return nP + nV + k * nv;};   // k = 0..N-1

  // Cost: 0.5 x'Px + q'x
  Triplets pt;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
  auto quad = [&](int i, int j, double w) {pt.emplace_back(i, j, w);};
  for (int k = 1; k <= N; ++k) {
    const double ramp = params_.near_weight + (1.0 - params_.near_weight) * (k - 1) /
      std::max(1, N - 1);
    const double f = (k == N ? params_.terminal_factor : 1.0) * ramp;
    Eigen::VectorXd e = p_nom[k] - ref.poses[k];
    e(layout_.yaw) = angle_diff(p_nom[k](layout_.yaw), ref.poses[k](layout_.yaw));
    for (int j = 0; j < np; ++j) {
      double w = j == layout_.yaw ? params_.weight_yaw : params_.weight_xy;
      if (layout_.aerial() && j == 2) {w = params_.weight_z;}
      w *= f;
      quad(iP(k) + j, iP(k) + j, 2.0 * w);
      q(iP(k) + j) += 2.0 * w * e(j);
    }
    const Eigen::VectorXd ev = v_nom[k] - ref.velocities[k - 1];
    for (int j = 0; j < nv; ++j) {
      const double w = j == layout_.wz ? params_.weight_angular_vel : params_.weight_linear_vel;
      quad(iV(k) + j, iV(k) + j, 2.0 * w);
      q(iV(k) + j) += 2.0 * w * ev(j);
    }
  }
  const Eigen::VectorXd last = in.last_cmd.size() == nv ? in.last_cmd : in.velocity;
  for (int k = 0; k < N; ++k) {
    const Eigen::VectorXd ed = u_nom[k] - (k == 0 ? last : u_nom[k - 1]);
    for (int j = 0; j < nv; ++j) {
      const double w = j == layout_.wz ? params_.weight_angular_rate :
        params_.weight_linear_rate;
      quad(iU(k) + j, iU(k) + j, 2.0 * w);
      q(iU(k) + j) += 2.0 * w * ed(j);
      if (k > 0) {
        quad(iU(k - 1) + j, iU(k - 1) + j, 2.0 * w);
        quad(iU(k) + j, iU(k - 1) + j, -2.0 * w);
        quad(iU(k - 1) + j, iU(k) + j, -2.0 * w);
        q(iU(k - 1) + j) -= 2.0 * w * ed(j);
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    quad(i, i, 1e-6);  // strictly convex
  }

  Constraints c;
  // Dynamics: dv_k+1 = lag dv_k + (1 - lag) du_k;  dp_k+1 = A dp_k + B dv_k+1
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j < nv; ++j) {
      const int r = c.add(0.0, 0.0);
      c.coef(r, iV(k + 1) + j, 1.0);
      c.coef(r, iU(k) + j, -(1.0 - lag));
      if (k > 0 && lag > 0.0) {
        c.coef(r, iV(k) + j, -lag);
      }
    }
    Eigen::MatrixXd A, B;
    linearize(layout_, p_nom[k], v_nom[k + 1], dt, A, B);
    for (int i = 0; i < np; ++i) {
      const int r = c.add(0.0, 0.0);
      c.coef(r, iP(k + 1) + i, 1.0);
      for (int j = 0; j < np && k > 0; ++j) {
        if (A(i, j) != 0.0) {c.coef(r, iP(k) + j, -A(i, j));}
      }
      for (int j = 0; j < nv; ++j) {
        if (B(i, j) != 0.0) {c.coef(r, iV(k + 1) + j, -B(i, j));}
      }
    }
  }
  // Commands within the velocity limits
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j < nv; ++j) {
      double lo = axes[j].min, hi = axes[j].max;
      if (turning_radius && j == 0) {lo = std::max(lo, 0.0);}
      widen(u_nom[k](j), lo, hi);
      const int r = c.add(lo - u_nom[k](j), hi - u_nom[k](j));
      c.coef(r, iU(k) + j, 1.0);
    }
  }
  // Velocity changes within the accelerations (decelerating towards zero, accelerating away)
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j < nv; ++j) {
      const double v = v_nom[k](j);
      double lo = -(v >= 0.0 ? axes[j].decel : axes[j].acc) * dt;
      double hi = (v >= 0.0 ? axes[j].acc : axes[j].decel) * dt;
      const double nominal = v_nom[k + 1](j) - v;
      widen(nominal, lo, hi);
      const int r = c.add(lo - nominal, hi - nominal);
      c.coef(r, iV(k + 1) + j, 1.0);
      if (k > 0) {c.coef(r, iV(k) + j, -1.0);}
    }
  }
  // Turning radius: -vx / R <= wz <= vx / R
  if (turning_radius) {
    const double inv_r = 1.0 / params_.min_turning_radius;
    for (int k = 1; k <= N; ++k) {
      const double vx = v_nom[k](0), wz = v_nom[k](layout_.wz);
      double lo = -std::numeric_limits<double>::infinity(), hi = 0.0;
      widen(wz - vx * inv_r, lo, hi);
      int r = c.add(lo - (wz - vx * inv_r), hi - (wz - vx * inv_r));
      c.coef(r, iV(k) + layout_.wz, 1.0);
      c.coef(r, iV(k), -inv_r);
      lo = 0.0;
      hi = std::numeric_limits<double>::infinity();
      widen(wz + vx * inv_r, lo, hi);
      r = c.add(lo - (wz + vx * inv_r), hi - (wz + vx * inv_r));
      c.coef(r, iV(k) + layout_.wz, 1.0);
      c.coef(r, iV(k), inv_r);
    }
  }
  // Obstacles: at each pose, half-planes (half-spaces for aerial robots) towards the nearest
  // points, at the robot's extent in that direction plus the margin
  if (in.obstacles && in.shape && !in.obstacles->empty()) {
    const bool use_z = in.obstacles->use_z();
    for (int k = 1; k <= N; ++k) {
      Eigen::Vector3d pos(p_nom[k](0), p_nom[k](1), layout_.aerial() ? p_nom[k](2) : 0.0);
      const double yaw = p_nom[k](layout_.yaw);
      std::vector<Eigen::Vector3d> normals;
      for (int i : in.obstacles->near(pos, params_.obstacle_range)) {
        if (static_cast<int>(normals.size()) >= params_.obstacles_per_step) {
          break;
        }
        Eigen::Vector3d away = pos - in.obstacles->point(i);
        if (!use_z) {away.z() = 0.0;}
        const double dist = away.norm();
        if (dist < 1e-6) {
          continue;
        }
        away /= dist;
        // One half-plane per direction: points behind the first ones add nothing
        auto similar = [&away](const Eigen::Vector3d & m) {
            return m.dot(away) > std::cos(M_PI / 4.0);
          };
        if (std::any_of(normals.begin(), normals.end(), similar)) {
          continue;
        }
        normals.push_back(away);
        double reach = in.shape->radius;
        if (!use_z) {
          const Eigen::Vector2d toward(-away.x(), -away.y());
          const double cy = std::cos(yaw), sy = std::sin(yaw);
          reach = in.shape->extent(
            Eigen::Vector2d(cy * toward.x() + sy * toward.y(), -sy * toward.x() + cy * toward.y()));
        }
        // n' (p - o) >= reach + margin;  never closer than the nominal if it is already closer
        const double lo = std::min(reach + params_.safety_margin - dist, 0.0);
        const int r = c.add(lo, std::numeric_limits<double>::infinity());
        c.coef(r, iP(k), away.x());
        c.coef(r, iP(k) + 1, away.y());
        if (use_z && layout_.aerial()) {c.coef(r, iP(k) + 2, away.z());}
      }
    }
  }

  QPProblem qp;
  qp.P.resize(n, n);
  qp.P.setFromTriplets(pt.begin(), pt.end());
  qp.q = q;
  const int m = static_cast<int>(c.lower.size());
  qp.A.resize(m, n);
  qp.A.setFromTriplets(c.a.begin(), c.a.end());
  qp.l = Eigen::Map<const Eigen::VectorXd>(c.lower.data(), m);
  qp.u = Eigen::Map<const Eigen::VectorXd>(c.upper.data(), m);
  result.variables = n;
  result.constraints = m;

  Eigen::VectorXd x;
  result.status = solve_qp(qp, params_.qp, x);
  if (result.status == QPStatus::Solved || result.status == QPStatus::Inaccurate) {
    result.commands.resize(N);
    for (int k = 0; k < N; ++k) {
      result.commands[k] = clamp_axes(u_nom[k] + x.segment(iU(k), nv));
    }
    result.cmd = result.commands[0];
    result.predicted.resize(N + 1);
    result.predicted[0] = p_nom[0];
    for (int k = 1; k <= N; ++k) {
      result.predicted[k] = p_nom[k] + x.segment(iP(k), np);
    }
    previous_ = result.commands;
  } else {
    previous_.clear();
  }
  result.solve_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace mpc
}  // namespace easynav
