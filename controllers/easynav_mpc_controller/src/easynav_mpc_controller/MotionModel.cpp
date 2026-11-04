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

#include "easynav_mpc_controller/MotionModel.hpp"

#include <cmath>

namespace easynav
{
namespace mpc
{

bool parse_motion_model(const std::string & name, MotionModel & model)
{
  if (name == "diff") {
    model = MotionModel::Diff;
  } else if (name == "omni") {
    model = MotionModel::Omni;
  } else if (name == "ackermann") {
    model = MotionModel::Ackermann;
  } else if (name == "aerial") {
    model = MotionModel::Aerial;
  } else {
    return false;
  }
  return true;
}

ModelLayout::ModelLayout(MotionModel m)
: model(m)
{
  switch (m) {
    case MotionModel::Diff:
    case MotionModel::Ackermann:
      np = 3; nv = 2; yaw = 2; vy = -1; vz = -1; wz = 1;
      break;
    case MotionModel::Omni:
      np = 3; nv = 3; yaw = 2; vy = 1; vz = -1; wz = 2;
      break;
    case MotionModel::Aerial:
      np = 4; nv = 4; yaw = 3; vy = 1; vz = 2; wz = 3;
      break;
  }
}

namespace
{

double component(const Eigen::VectorXd & v, int index)
{
  return index >= 0 ? v(index) : 0.0;
}

}  // namespace

Eigen::VectorXd step(
  const ModelLayout & l, const Eigen::VectorXd & p, const Eigen::VectorXd & v, double dt)
{
  const double vx = v(0), vy = component(v, l.vy), wz = v(l.wz);
  // Heading at the middle of the step: exact for constant curvature up to O(dt^3)
  const double yaw = p(l.yaw) + 0.5 * wz * dt;
  const double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::VectorXd next = p;
  next(0) += (vx * c - vy * s) * dt;
  next(1) += (vx * s + vy * c) * dt;
  if (l.aerial()) {
    next(2) += v(l.vz) * dt;
  }
  next(l.yaw) += wz * dt;
  return next;
}

void linearize(
  const ModelLayout & l, const Eigen::VectorXd & p, const Eigen::VectorXd & v, double dt,
  Eigen::MatrixXd & A, Eigen::MatrixXd & B)
{
  const double vx = v(0), vy = component(v, l.vy), wz = v(l.wz);
  const double yaw = p(l.yaw) + 0.5 * wz * dt;
  const double c = std::cos(yaw), s = std::sin(yaw);
  // d(x, y)/d(yaw) of the displacement
  const double dx_dyaw = (-vx * s - vy * c) * dt;
  const double dy_dyaw = (vx * c - vy * s) * dt;

  A = Eigen::MatrixXd::Identity(l.np, l.np);
  A(0, l.yaw) = dx_dyaw;
  A(1, l.yaw) = dy_dyaw;

  B = Eigen::MatrixXd::Zero(l.np, l.nv);
  B(0, 0) = c * dt;
  B(1, 0) = s * dt;
  if (l.holonomic()) {
    B(0, l.vy) = -s * dt;
    B(1, l.vy) = c * dt;
  }
  if (l.aerial()) {
    B(2, l.vz) = dt;
  }
  B(0, l.wz) = 0.5 * dt * dx_dyaw;
  B(1, l.wz) = 0.5 * dt * dy_dyaw;
  B(l.yaw, l.wz) = dt;
}

}  // namespace mpc
}  // namespace easynav
