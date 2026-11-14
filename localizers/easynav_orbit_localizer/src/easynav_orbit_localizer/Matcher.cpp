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

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "Eigen/Cholesky"
#include "easynav_orbit_localizer/Matcher.hpp"

namespace easynav
{
namespace orbit
{

Matcher::Matcher(const DistanceField & field, const MatchOptions & options)
: field_(field), options_(options) {}

std::vector<int> Matcher::freeDofs() const
{
  if (options_.mode == Mode::Free) {return {0, 1, 2, 3, 4, 5};}
  return {0, 1, 5};
}

double Matcher::score(const std::vector<Eigen::Vector3d> & body_points, const Pose & X) const
{
  if (body_points.empty()) {return 0.0;}
  std::size_t inliers = 0;
  for (const auto & p : body_points) {
    const double d = field_.distance(X * p, nullptr);
    if (d <= options_.inlier_distance) {++inliers;}
  }
  return static_cast<double>(inliers) / static_cast<double>(body_points.size());
}

MatchResult Matcher::match(
  const std::vector<Eigen::Vector3d> & body_points, const Pose & X0, const Mat6 * prior_info,
  const std::vector<PositionMeasurement> & positions,
  const std::vector<DepthMeasurement> & depths,
  const std::vector<AttitudeMeasurement> & attitudes) const
{
  MatchResult out;
  out.pose = X0;
  out.n_points = body_points.size();
  const std::vector<int> dofs = freeDofs();
  const int n = static_cast<int>(dofs.size());
  const double w_point = 1.0 / (options_.sigma_point * options_.sigma_point);

  // The Cauchy scale starts as wide as the prior's doubt (three deviations of the position),
  // so a start that far off is not an outlier, and shrinks to its value in a few iterations
  // (graduated non-convexity). Without a prior it starts at four times its value.
  double c_start = 4.0 * options_.cauchy_c;
  if (prior_info) {
    const Mat6 P = (*prior_info + 1e-12 * Mat6::Identity()).inverse();
    const double s = std::sqrt(std::max({P(0, 0), P(1, 1), P(2, 2), 0.0}));
    c_start = std::min(std::max(3.0 * s, options_.cauchy_c), 4.0 * options_.cauchy_c);
  }

  Pose X = X0;
  Mat6 H_full = Mat6::Zero();
  for (int it = 0; it < options_.max_iterations; ++it) {
    H_full.setZero();
    Vec6 b = Vec6::Zero();
    const double c = std::max(options_.cauchy_c, c_start / (1 << it));
    const double c2 = c * c;

    // Points: r = d(X p), J = grad^T R [I | -[p]x]
    for (const auto & p : body_points) {
      Eigen::Vector3d g;
      const double d = field_.distance(X * p, &g);
      if (!std::isfinite(d) || g.squaredNorm() < 1e-12) {continue;}
      const double w = w_point / (1.0 + d * d / c2);   // Cauchy
      const Eigen::RowVector3d gR = g.transpose() * X.linear();
      Vec6 J;
      J.head<3>() = gR.transpose();
      J.tail<3>() = (-gR * hat(p)).transpose();
      H_full.noalias() += w * J * J.transpose();
      b.noalias() += w * J * d;
    }

    // Prior on the perturbation of X0: r = Log(X0^-1 X) + delta
    if (prior_info) {
      const Vec6 eps = logSE3(X0.inverse() * X);
      H_full += *prior_info;
      b += *prior_info * eps;
    }

    // Positions: r = X lever - p_map, J = R [I | -[lever]x]
    for (const auto & m : positions) {
      const Eigen::Vector3d r = X * m.lever - m.p_map;
      Eigen::Matrix<double, 3, 6> J;
      J.block<3, 3>(0, 0) = X.linear();
      J.block<3, 3>(0, 3) = -X.linear() * hat(m.lever);
      const int rows = m.use_z ? 3 : 2;
      for (int k = 0; k < rows; ++k) {
        const double w = 1.0 / (m.sigma[k] * m.sigma[k]);
        H_full.noalias() += w * J.row(k).transpose() * J.row(k);
        b.noalias() += w * J.row(k).transpose() * r[k];
      }
    }

    // Depths: r = z - z_meas, J = R.row(2) on rho
    for (const auto & m : depths) {
      Vec6 J = Vec6::Zero();
      J.head<3>() = X.linear().row(2).transpose();
      const double w = 1.0 / (m.sigma * m.sigma);
      H_full.noalias() += w * J * J.transpose();
      b.noalias() += w * J * (X.translation().z() - m.z);
    }

    // Attitudes: r = Log(R_meas^T R) + phi, body-frame rotation error
    for (const auto & m : attitudes) {
      const Eigen::Vector3d r = logSO3(m.R_map_body.transpose() * X.linear());
      const int rows = m.use_yaw ? 3 : 2;
      for (int k = 0; k < rows; ++k) {
        const double s = k < 2 ? m.sigma_roll_pitch : m.sigma_yaw;
        const double w = 1.0 / (s * s);
        H_full(3 + k, 3 + k) += w;
        b(3 + k) += w * r[k];
      }
    }

    // Reduced system on the free degrees of freedom, damped
    Eigen::MatrixXd H(n, n);
    Eigen::VectorXd g(n);
    for (int a = 0; a < n; ++a) {
      g(a) = b(dofs[a]);
      for (int c = 0; c < n; ++c) {
        H(a, c) = H_full(dofs[a], dofs[c]);
      }
    }
    for (int a = 0; a < n; ++a) {
      H(a, a) += options_.damping * std::max(H(a, a), 1e-9) + 1e-9;
    }
    const Eigen::VectorXd step = H.ldlt().solve(-g);
    if (!step.allFinite()) {break;}
    Vec6 delta = Vec6::Zero();
    for (int a = 0; a < n; ++a) {
      delta(dofs[a]) = step(a);
    }
    X = X * expSE3(delta);
    out.iterations = it + 1;
    if (delta.norm() < options_.step_tolerance) {
      out.converged = true;
      break;
    }
  }

  // Covariance at the solution (the information of the last system, without damping)
  Eigen::MatrixXd H(n, n);
  for (int a = 0; a < n; ++a) {
    for (int c = 0; c < n; ++c) {
      H(a, c) = H_full(dofs[a], dofs[c]);
    }
    H(a, a) += 1e-9;
  }
  const Eigen::MatrixXd C = H.inverse();
  out.cov = Mat6::Zero();
  out.info = Mat6::Zero();
  for (int k = 0; k < 6; ++k) {
    out.cov(k, k) = options_.fixed_variance;
    out.info(k, k) = 1.0 / options_.fixed_variance;
  }
  for (int a = 0; a < n; ++a) {
    for (int c = 0; c < n; ++c) {
      out.cov(dofs[a], dofs[c]) = C(a, c);
      out.info(dofs[a], dofs[c]) = H(a, c);
    }
  }

  out.pose = X;
  out.correction = logSE3(X0.inverse() * X);
  if (!body_points.empty()) {
    std::size_t inliers = 0;
    double sum = 0.0;
    for (const auto & p : body_points) {
      double d = field_.distance(X * p, nullptr);
      if (!std::isfinite(d)) {d = options_.cauchy_c * 10.0;}
      sum += d;
      if (d <= options_.inlier_distance) {++inliers;}
    }
    out.score = static_cast<double>(inliers) / static_cast<double>(body_points.size());
    out.mean_distance = sum / static_cast<double>(body_points.size());
  }
  return out;
}

}  // namespace orbit
}  // namespace easynav
