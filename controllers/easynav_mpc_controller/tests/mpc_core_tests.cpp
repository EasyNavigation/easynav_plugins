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
/// \brief The MPC's parts: motion models and their linearization, the QP solver, the robot's
/// shape against obstacles, and the reference along the path.

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "easynav_mpc_controller/MotionModel.hpp"
#include "easynav_mpc_controller/Obstacles.hpp"
#include "easynav_mpc_controller/QPSolver.hpp"
#include "easynav_mpc_controller/Reference.hpp"

using easynav::mpc::MotionModel;
using easynav::mpc::ModelLayout;

namespace
{

nav_msgs::msg::Path polyline(const std::vector<Eigen::Vector3d> & pts, double res = 0.05)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const int n = std::max(1, static_cast<int>((pts[i] - pts[i - 1]).norm() / res));
    for (int k = (i == 1 ? 0 : 1); k <= n; ++k) {
      geometry_msgs::msg::PoseStamped p;
      const Eigen::Vector3d q = pts[i - 1] + (pts[i] - pts[i - 1]) * k / n;
      p.pose.position.x = q.x();
      p.pose.position.y = q.y();
      p.pose.position.z = q.z();
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
  }
  return path;
}

Eigen::VectorXd vec(std::initializer_list<double> v)
{
  Eigen::VectorXd out(v.size());
  int i = 0;
  for (double x : v) {
    out(i++) = x;
  }
  return out;
}

}  // namespace

// ---------------------------------------------------------------- motion models

TEST(MpcMotionModel, NamesAndLayouts)
{
  MotionModel m;
  EXPECT_TRUE(easynav::mpc::parse_motion_model("ackermann", m));
  EXPECT_EQ(m, MotionModel::Ackermann);
  EXPECT_FALSE(easynav::mpc::parse_motion_model("Diff", m)) << "case matters";
  EXPECT_FALSE(easynav::mpc::parse_motion_model("", m));

  const ModelLayout diff(MotionModel::Diff), omni(MotionModel::Omni), air(MotionModel::Aerial);
  EXPECT_EQ(diff.np, 3);
  EXPECT_EQ(diff.nv, 2);
  EXPECT_FALSE(diff.holonomic());
  EXPECT_TRUE(omni.holonomic());
  EXPECT_EQ(air.np, 4);
  EXPECT_EQ(air.nv, 4);
  EXPECT_TRUE(air.aerial());
  EXPECT_FALSE(ModelLayout(MotionModel::Ackermann).turns_in_place());
  EXPECT_TRUE(diff.turns_in_place());
}

TEST(MpcMotionModel, StepsInTheBodyFrame)
{
  const ModelLayout diff(MotionModel::Diff);
  auto p = easynav::mpc::step(diff, vec({1.0, 2.0, M_PI / 2}), vec({1.0, 0.0}), 0.5);
  EXPECT_NEAR(p(0), 1.0, 1e-12);
  EXPECT_NEAR(p(1), 2.5, 1e-12) << "facing +y";

  const ModelLayout omni(MotionModel::Omni);
  p = easynav::mpc::step(omni, vec({0.0, 0.0, M_PI / 2}), vec({0.0, 1.0, 0.0}), 1.0);
  EXPECT_NEAR(p(0), -1.0, 1e-12) << "sideways to the left of +y is -x";

  const ModelLayout air(MotionModel::Aerial);
  p = easynav::mpc::step(air, vec({0.0, 0.0, 5.0, 0.0}), vec({0.0, 0.0, -1.0, 0.0}), 2.0);
  EXPECT_NEAR(p(2), 3.0, 1e-12);
}

TEST(MpcMotionModel, ArcsAreAccurate)
{
  // A full circle of radius 2 in 100 steps closes within millimeters (midpoint heading)
  const ModelLayout diff(MotionModel::Diff);
  Eigen::VectorXd p = vec({0.0, 0.0, 0.0});
  const double v = 1.0, w = 0.5, dt = 4.0 * M_PI / 100.0;
  for (int i = 0; i < 100; ++i) {
    p = easynav::mpc::step(diff, p, vec({v, w}), dt);
  }
  EXPECT_NEAR(p(0), 0.0, 5e-3);
  EXPECT_NEAR(p(1), 0.0, 5e-3);
}

TEST(MpcMotionModel, LinearizationMatchesFiniteDifferences)
{
  for (auto model : {MotionModel::Diff, MotionModel::Omni, MotionModel::Ackermann,
      MotionModel::Aerial})
  {
    const ModelLayout l(model);
    Eigen::VectorXd p = Eigen::VectorXd::Zero(l.np), v = Eigen::VectorXd::Zero(l.nv);
    for (int i = 0; i < l.np; ++i) {
      p(i) = 0.3 * (i + 1);
    }
    for (int i = 0; i < l.nv; ++i) {
      v(i) = 0.7 - 0.4 * i;
    }
    const double dt = 0.1, h = 1e-6;
    Eigen::MatrixXd A, B;
    easynav::mpc::linearize(l, p, v, dt, A, B);
    for (int j = 0; j < l.np; ++j) {
      Eigen::VectorXd d = Eigen::VectorXd::Zero(l.np);
      d(j) = h;
      const Eigen::VectorXd fd = (easynav::mpc::step(l, p + d, v, dt) -
        easynav::mpc::step(l, p - d, v, dt)) / (2 * h);
      EXPECT_LT((A.col(j) - fd).norm(), 1e-6) << "A col " << j;
    }
    for (int j = 0; j < l.nv; ++j) {
      Eigen::VectorXd d = Eigen::VectorXd::Zero(l.nv);
      d(j) = h;
      const Eigen::VectorXd fd = (easynav::mpc::step(l, p, v + d, dt) -
        easynav::mpc::step(l, p, v - d, dt)) / (2 * h);
      EXPECT_LT((B.col(j) - fd).norm(), 1e-6) << "B col " << j;
    }
  }
}

// ---------------------------------------------------------------- QP

namespace
{

easynav::mpc::QPProblem box_qp(double lo, double hi)
{
  // min (x0 - 1)^2 + (x1 + 2)^2  s.t.  lo <= x0 + x1 <= hi,  -3 <= x0 <= 3
  easynav::mpc::QPProblem qp;
  qp.P.resize(2, 2);
  qp.P.insert(0, 0) = 2.0;
  qp.P.insert(1, 1) = 2.0;
  qp.q = vec({-2.0, 4.0});
  qp.A.resize(2, 2);
  qp.A.insert(0, 0) = 1.0;
  qp.A.insert(0, 1) = 1.0;
  qp.A.insert(1, 0) = 1.0;
  qp.l = vec({lo, -3.0});
  qp.u = vec({hi, 3.0});
  return qp;
}

}  // namespace

TEST(MpcQP, UnconstrainedOptimum)
{
  Eigen::VectorXd x;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  auto qp = box_qp(-kInf, kInf);
  ASSERT_EQ(easynav::mpc::solve_qp(qp, {}, x), easynav::mpc::QPStatus::Solved);
  EXPECT_NEAR(x(0), 1.0, 1e-3);
  EXPECT_NEAR(x(1), -2.0, 1e-3);
}

TEST(MpcQP, ActiveConstraint)
{
  // x0 + x1 >= 1: the optimum moves to (2, -1)
  Eigen::VectorXd x;
  auto qp = box_qp(1.0, std::numeric_limits<double>::infinity());
  easynav::mpc::QPSettings s;
  s.eps_abs = s.eps_rel = 1e-6;
  ASSERT_EQ(easynav::mpc::solve_qp(qp, s, x), easynav::mpc::QPStatus::Solved);
  EXPECT_NEAR(x(0), 2.0, 1e-3);
  EXPECT_NEAR(x(1), -1.0, 1e-3);
  EXPECT_GE(x(0) + x(1), 1.0 - 1e-3);
}

TEST(MpcQP, InfeasibleAndBadInput)
{
  // x0 + x1 >= 7 and x0 + x1 <= 5 at once
  Eigen::VectorXd x;
  auto qp = box_qp(7.0, 8.0);
  qp.A.resize(3, 2);
  qp.A.insert(0, 0) = 1.0;
  qp.A.insert(0, 1) = 1.0;
  qp.A.insert(1, 0) = 1.0;
  qp.A.insert(2, 0) = 1.0;
  qp.A.insert(2, 1) = 1.0;
  qp.l = vec({7.0, -3.0, -std::numeric_limits<double>::infinity()});
  qp.u = vec({8.0, 3.0, 5.0});
  EXPECT_EQ(easynav::mpc::solve_qp(qp, {}, x), easynav::mpc::QPStatus::Infeasible);

  auto bad = box_qp(0.0, 1.0);
  bad.q(0) = std::nan("");
  EXPECT_EQ(easynav::mpc::solve_qp(bad, {}, x), easynav::mpc::QPStatus::Failed);
  auto wrong = box_qp(0.0, 1.0);
  wrong.l = vec({0.0});
  EXPECT_EQ(easynav::mpc::solve_qp(wrong, {}, x), easynav::mpc::QPStatus::Failed);
}

TEST(MpcQP, FewIterationsGiveAnInaccurateButUsableSolution)
{
  Eigen::VectorXd x;
  auto qp = box_qp(1.0, std::numeric_limits<double>::infinity());
  easynav::mpc::QPSettings s;
  s.max_iterations = 1;
  s.eps_abs = s.eps_rel = 1e-9;
  const auto status = easynav::mpc::solve_qp(qp, s, x);
  EXPECT_TRUE(
    status == easynav::mpc::QPStatus::Inaccurate || status == easynav::mpc::QPStatus::Solved);
  EXPECT_EQ(x.size(), 2);
}

// ---------------------------------------------------------------- shape and obstacles

TEST(MpcObstacles, ExtentOfACircleAndOfAFootprint)
{
  easynav::mpc::RobotShape circle;
  circle.radius = 0.4;
  EXPECT_DOUBLE_EQ(circle.extent({1.0, 0.0}), 0.4);
  EXPECT_DOUBLE_EQ(circle.extent({0.0, -1.0}), 0.4);

  // A car with its frame at the rear axle: 3.7 m ahead, 0.9 m behind, 1 m to each side
  easynav::mpc::RobotShape car;
  car.footprint = {{3.7, 1.0}, {-0.9, 1.0}, {-0.9, -1.0}, {3.7, -1.0}};
  EXPECT_NEAR(car.extent({1.0, 0.0}), 3.7, 1e-9);
  EXPECT_NEAR(car.extent({-1.0, 0.0}), 0.9, 1e-9);
  EXPECT_NEAR(car.extent({0.0, 1.0}), 1.0, 1e-9);
  EXPECT_NEAR(car.extent(Eigen::Vector2d(1.0, 1.0).normalized()), std::sqrt(2.0), 1e-9);

  EXPECT_TRUE(car.contains({3.0, 0.5}, 0.0));
  EXPECT_FALSE(car.contains({3.8, 0.0}, 0.0));
  EXPECT_TRUE(car.contains({3.8, 0.0}, 0.15)) << "within the margin";
  EXPECT_FALSE(car.contains({0.0, 1.3}, 0.2));
}

TEST(MpcObstacles, NearestFirstAndCollisions)
{
  easynav::mpc::Obstacles obstacles;
  obstacles.set({{2.0, 0.0, 0.3}, {1.0, 0.0, 0.3}, {5.0, 5.0, 0.3}}, false);
  const auto near = obstacles.near({0.0, 0.0, 10.0}, 3.0);
  ASSERT_EQ(near.size(), 2u) << "2D: the height does not count";
  EXPECT_EQ(near[0], 1);
  EXPECT_EQ(near[1], 0);

  easynav::mpc::RobotShape car;
  car.footprint = {{1.5, 0.5}, {-0.5, 0.5}, {-0.5, -0.5}, {1.5, -0.5}};
  EXPECT_TRUE(obstacles.collides(car, {0.0, 0.0, 0.0}, 0.0, 0.0)) << "the nose reaches 1.0";
  EXPECT_FALSE(obstacles.collides(car, {0.0, 0.0, 0.0}, M_PI, 0.0)) << "turned around";
  EXPECT_FALSE(obstacles.collides(car, {0.0, -2.0, 0.0}, 0.0, 0.0));

  easynav::mpc::Obstacles air;
  air.set({{0.0, 0.0, 3.0}}, true);
  easynav::mpc::RobotShape ball;
  ball.radius = 0.5;
  EXPECT_FALSE(air.collides(ball, {0.0, 0.0, 0.0}, 0.0, 0.1)) << "3D: far below it";
  EXPECT_TRUE(air.collides(ball, {0.0, 0.0, 2.6}, 0.0, 0.0));
}

// ---------------------------------------------------------------- reference

TEST(MpcReference, ProgressOnlyMovesForwardAlongAUTurn)
{
  // Out along y = 0 and back along y = 1: the way back passes 1 m from the way out
  easynav::mpc::Reference ref(MotionModel::Diff);
  ref.set_path(polyline({{0, 0, 0}, {4, 0, 0}, {4, 1, 0}, {0, 1, 0}}));
  EXPECT_NEAR(ref.project({0.5, 0.2, 0.0}), 0.5, 1e-9);
  EXPECT_NEAR(ref.project({3.0, 0.4, 0.0}), 3.0, 1e-9);
  // Past the turn, near the start of the way back
  EXPECT_NEAR(ref.project({3.5, 0.9, 0.0}), 5.5, 1e-9);
  // A robot that seems to be near the start again is still on the way back
  EXPECT_GT(ref.project({0.6, 0.6, 0.0}), 5.0);
}

TEST(MpcReference, ANewPathIsSearchedFromItsBeginning)
{
  easynav::mpc::Reference ref(MotionModel::Diff);
  ref.set_path(polyline({{0, 0, 0}, {20, 0, 0}}));
  ref.project({15.0, 0.0, 0.0});
  ref.project({15.5, 0.0, 0.0});
  // The same path again: progress is kept; a different one (replanned): it starts over
  ref.set_path(polyline({{0, 0, 0}, {20, 0, 0}}));
  EXPECT_NEAR(ref.project({16.0, 0.0, 0.0}), 16.0, 1e-9);
  ref.set_path(polyline({{15, 0, 0}, {15, 10, 0}}));
  EXPECT_NEAR(ref.project({15.0, 0.5, 0.0}), 0.5, 1e-9);
}

TEST(MpcReference, SpeedIsLimitedByTheLimitsTheCurvatureAndTheGoal)
{
  easynav::mpc::Reference ref(MotionModel::Diff);
  ref.set_path(polyline({{0, 0, 0}, {30, 0, 0}}));
  easynav::mpc::SpeedLimits lim;
  lim.max_vel = 2.0;
  lim.acc = 1.0;
  lim.decel = 1.0;
  easynav::mpc::RobotShape shape;
  const auto r = ref.build({0, 0, 0}, 0.0, 0.0, 40, 0.1, lim, nullptr, shape, 0.0);
  ASSERT_EQ(r.poses.size(), 41u);
  ASSERT_EQ(r.velocities.size(), 40u);
  // From rest at 1 m/s^2: 0.1 m/s per step, up to 2 m/s
  EXPECT_NEAR(r.velocities[0](0), 0.05, 1e-6);
  EXPECT_NEAR(r.velocities[39](0), 2.0, 0.06);
  for (const auto & v : r.velocities) {
    EXPECT_LE(v(0), 2.0 + 1e-9);
    EXPECT_NEAR(v(1), 0.0, 1e-9) << "straight";
  }

  // Close to the goal: braking to it (2 m left: sqrt(2 * 1 * 2) = 2 m/s at most, decreasing)
  ref.set_path(polyline({{0, 0, 0}, {2, 0, 0}}));
  const auto g = ref.build({0, 0, 0}, 0.0, 2.0, 40, 0.1, lim, nullptr, shape, 0.0);
  EXPECT_LT(g.velocities.back()(0), 0.2);
  EXPECT_LE((g.poses.back().head<2>() - Eigen::Vector2d(2.0, 0.0)).norm(), 0.05);

  // A circle of radius 2: v^2 / 2 <= max_lateral_acc 0.5 -> 1 m/s; w = v / 2
  std::vector<Eigen::Vector3d> circle;
  for (int i = 0; i <= 200; ++i) {
    const double a = i * M_PI / 100.0;
    circle.emplace_back(2.0 * std::sin(a), 2.0 - 2.0 * std::cos(a), 0.0);
  }
  ref.set_path(polyline(circle));
  lim.max_lateral_acc = 0.5;
  const auto c = ref.build({0, 0, 0}, 0.0, 1.0, 20, 0.1, lim, nullptr, shape, 0.0);
  // (the polyline's curvature is a few % below the circle's; at the start of the path the
  // heading's window is one-sided)
  for (std::size_t k = 3; k < c.velocities.size(); ++k) {
    const auto & v = c.velocities[k];
    EXPECT_LE(v(0), 1.05) << k;
    EXPECT_NEAR(v(1), v(0) / 2.0, 0.05) << k;
  }
}

TEST(MpcReference, AnObstacleOnThePathStopsTheReferenceBeforeIt)
{
  easynav::mpc::Reference ref(MotionModel::Diff);
  ref.set_path(polyline({{0, 0, 0}, {30, 0, 0}}));
  easynav::mpc::SpeedLimits lim;
  lim.max_vel = 2.0;
  lim.acc = 1.0;
  lim.decel = 1.0;
  easynav::mpc::RobotShape shape;
  shape.radius = 0.5;
  easynav::mpc::Obstacles obstacles;
  obstacles.set({{6.0, 0.1, 0.3}}, false);
  const auto r = ref.build({0, 0, 0}, 0.0, 2.0, 60, 0.1, lim, &obstacles, shape, 0.1);
  EXPECT_NEAR(r.free_distance, 5.25, 0.3) << "6 - radius - margin";
  EXPECT_LT(r.poses.back()(0) + shape.radius + 0.1, 6.0) << "stops before touching it";
  EXPECT_LT(r.velocities.back()(0), 0.05);

  // Beside the path (2 m away): nothing changes
  obstacles.set({{6.0, 2.0, 0.3}}, false);
  const auto free = ref.build({0, 0, 0}, 0.0, 2.0, 60, 0.1, lim, &obstacles, shape, 0.1);
  EXPECT_TRUE(std::isinf(free.free_distance));
}

TEST(MpcReference, HeadingContinuesFromTheRobotAndAerialPathsClimb)
{
  // The robot's yaw is 2 pi + a bit: the reference's is near it, not near 0
  easynav::mpc::Reference ref(MotionModel::Diff);
  ref.set_path(polyline({{0, 0, 0}, {10, 0, 0}}));
  easynav::mpc::SpeedLimits lim;
  easynav::mpc::RobotShape shape;
  const auto r = ref.build({0, 0, 0}, 2.0 * M_PI + 0.1, 0.0, 5, 0.1, lim, nullptr, shape, 0.0);
  EXPECT_NEAR(r.poses[0](2), 2.0 * M_PI, 1e-6);

  // Aerial: 10 m forward and 5 m up; vertical speed limited to 0.5 m/s
  easynav::mpc::Reference air(MotionModel::Aerial);
  air.set_path(polyline({{0, 0, 2}, {10, 0, 7}}));
  lim.max_vel = 2.0;
  lim.max_vertical_vel = 0.5;
  lim.acc = 10.0;
  const auto a = air.build({0, 0, 2}, 0.0, 1.0, 30, 0.1, lim, nullptr, shape, 0.0);
  ASSERT_EQ(a.poses[0].size(), 4);
  EXPECT_GT(a.poses.back()(2), 2.0) << "it climbs";
  for (const auto & v : a.velocities) {
    EXPECT_LE(v(2), 0.5 + 1e-6) << "vz";
    EXPECT_NEAR(v(2), 0.5 * v(0), 1e-6) << "along the 1:2 slope";
  }
}
