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

/// \file
/// \brief SE(3) as a Lie group: exponential, logarithm and adjoint, with the tangent vector
/// ordered [rho (translation), phi (rotation)] and right (body-frame) perturbations X * Exp(d).

#ifndef EASYNAV_ORBIT_LOCALIZER__LIE_HPP_
#define EASYNAV_ORBIT_LOCALIZER__LIE_HPP_

#include <algorithm>
#include <cmath>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace easynav
{
namespace orbit
{

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Pose = Eigen::Isometry3d;

/// @brief Skew-symmetric matrix of v: hat(v) * w = v x w.
inline Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

/// @brief Rotation of the rotation vector phi (Rodrigues).
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d & phi)
{
  const double a = phi.norm();
  const Eigen::Matrix3d K = hat(phi);
  if (a < 1e-9) {
    return Eigen::Matrix3d::Identity() + K + 0.5 * K * K;
  }
  return Eigen::Matrix3d::Identity() + (std::sin(a) / a) * K +
         ((1.0 - std::cos(a)) / (a * a)) * K * K;
}

/// @brief Rotation vector of R.
inline Eigen::Vector3d logSO3(const Eigen::Matrix3d & R)
{
  const double c = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) * 0.5));
  const double a = std::acos(c);
  Eigen::Vector3d v(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  if (a < 1e-9) {
    return 0.5 * v;
  }
  if (M_PI - a < 1e-6) {
    // Near a half turn the axis comes from the symmetric part of R
    const Eigen::Matrix3d S = 0.5 * (R + Eigen::Matrix3d::Identity());
    Eigen::Vector3d axis(std::sqrt(std::max(S(0, 0), 0.0)), std::sqrt(std::max(S(1, 1), 0.0)),
      std::sqrt(std::max(S(2, 2), 0.0)));
    // Signs from the off-diagonal products
    if (S(0, 1) < 0.0) {axis.y() = -axis.y();}
    if (S(0, 2) < 0.0) {axis.z() = -axis.z();}
    if (axis.norm() < 1e-9) {return Eigen::Vector3d::Zero();}
    return a * axis.normalized();
  }
  return (a / (2.0 * std::sin(a))) * v;
}

/// @brief Left Jacobian of SO(3): Exp(phi + d) ~ Exp(J_l(phi) d) Exp(phi).
inline Eigen::Matrix3d leftJacobianSO3(const Eigen::Vector3d & phi)
{
  const double a = phi.norm();
  const Eigen::Matrix3d K = hat(phi);
  if (a < 1e-9) {
    return Eigen::Matrix3d::Identity() + 0.5 * K;
  }
  return Eigen::Matrix3d::Identity() + ((1.0 - std::cos(a)) / (a * a)) * K +
         ((a - std::sin(a)) / (a * a * a)) * K * K;
}

/// @brief Inverse of the left Jacobian of SO(3).
inline Eigen::Matrix3d leftJacobianInvSO3(const Eigen::Vector3d & phi)
{
  const double a = phi.norm();
  const Eigen::Matrix3d K = hat(phi);
  if (a < 1e-9) {
    return Eigen::Matrix3d::Identity() - 0.5 * K;
  }
  return Eigen::Matrix3d::Identity() - 0.5 * K +
         ((1.0 / (a * a)) - (1.0 + std::cos(a)) / (2.0 * a * std::sin(a))) * K * K;
}

/// @brief Exp of the twist xi = [rho, phi].
inline Pose expSE3(const Vec6 & xi)
{
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d phi = xi.tail<3>();
  Pose T = Pose::Identity();
  T.linear() = expSO3(phi);
  T.translation() = leftJacobianSO3(phi) * rho;
  return T;
}

/// @brief Log of T, the twist [rho, phi] with T = Exp.
inline Vec6 logSE3(const Pose & T)
{
  Vec6 xi;
  const Eigen::Vector3d phi = logSO3(T.linear());
  xi.tail<3>() = phi;
  xi.head<3>() = leftJacobianInvSO3(phi) * T.translation();
  return xi;
}

/// @brief Adjoint of T: T Exp(d) T^-1 = Exp(Ad(T) d).
inline Mat6 adjoint(const Pose & T)
{
  Mat6 ad = Mat6::Zero();
  const Eigen::Matrix3d & R = T.linear();
  ad.block<3, 3>(0, 0) = R;
  ad.block<3, 3>(0, 3) = hat(T.translation()) * R;
  ad.block<3, 3>(3, 3) = R;
  return ad;
}

/// @brief Yaw (rad) of the pose: the heading of its x axis on the horizontal plane.
inline double yawOf(const Pose & T)
{
  return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
}

/// @brief Roll, pitch and yaw (rad, ZYX) of the rotation.
inline Eigen::Vector3d rpyOf(const Eigen::Matrix3d & R)
{
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  if (std::abs(std::cos(pitch)) < 1e-9) {
    return Eigen::Vector3d(0.0, pitch, std::atan2(-R(0, 1), R(1, 1)));
  }
  return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)), pitch, std::atan2(R(1, 0), R(0, 0)));
}

/// @brief Rotation of (roll, pitch, yaw) in ZYX order.
inline Eigen::Matrix3d fromRPY(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// @brief Planar pose at (x, y, z) heading yaw.
inline Pose fromXYZYaw(double x, double y, double z, double yaw)
{
  Pose T = Pose::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

/// @brief Angle wrapped to (-pi, pi].
inline double wrapAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

}  // namespace orbit
}  // namespace easynav

#endif  // EASYNAV_ORBIT_LOCALIZER__LIE_HPP_
