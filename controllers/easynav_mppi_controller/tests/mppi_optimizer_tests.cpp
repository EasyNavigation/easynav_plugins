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
/// \brief The MPPI optimizer: what it does with invalid input, at the goal, when blocked, within
/// its limits, and its nearest-seed grid. Driving along paths: mppi_scenarios_tests.

#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "easynav_mppi_controller/NearestGrid.hpp"
#include "mppi_sim.hpp"

using easynav::HeadingMode;
using easynav::MPPIInput;
using easynav::MPPIStatus;
using easynav::NearestGrid;
using easynav::Pose2D;
using easynav::Velocity;
using mppi_test::Sim;
using mppi_test::diff_params;
using mppi_test::omni_params;
using mppi_test::polyline;

namespace
{

MPPIInput input_for(const nav_msgs::msg::Path & path, Pose2D pose, Velocity velocity = {})
{
  MPPIInput in;
  in.pose = pose;
  in.velocity = velocity;
  in.path = &path;
  in.xy_goal_tolerance = 0.1;
  in.yaw_goal_tolerance = 0.1;
  return in;
}

bool is_zero(const Velocity & v) {return v.vx == 0.0 && v.vy == 0.0 && v.wz == 0.0;}

}  // namespace

// Invalid input.

TEST(MppiOptimizer, WithoutAValidPathOrPoseItStops)
{
  easynav::MPPIOptimizer optimizer;
  optimizer.configure(diff_params());
  const auto path = polyline({{0.0, 0.0}, {2.0, 0.0}}, 0.0);

  MPPIInput in = input_for(path, {});
  in.path = nullptr;
  EXPECT_EQ(optimizer.compute(in).status, MPPIStatus::NoPath);

  nav_msgs::msg::Path empty;
  in = input_for(empty, {});
  EXPECT_EQ(optimizer.compute(in).status, MPPIStatus::NoPath);

  auto nan_path = path;
  nan_path.poses[3].pose.position.y = std::nan("");
  in = input_for(nan_path, {});
  const auto r = optimizer.compute(in);
  EXPECT_EQ(r.status, MPPIStatus::NoPath);
  EXPECT_TRUE(is_zero(r.cmd));

  auto inf_orientation = path;
  inf_orientation.poses.back().pose.orientation.w = std::numeric_limits<double>::infinity();
  in = input_for(inf_orientation, {});
  EXPECT_EQ(optimizer.compute(in).status, MPPIStatus::NoPath);

  in = input_for(path, {std::nan(""), 0.0, 0.0});
  EXPECT_EQ(optimizer.compute(in).status, MPPIStatus::NoPath);
  in = input_for(path, {0.0, 0.0, std::numeric_limits<double>::infinity()});
  EXPECT_EQ(optimizer.compute(in).status, MPPIStatus::NoPath);
  EXPECT_EQ(optimizer.rollouts(), 0);

  // Valid again: it follows.
  in = input_for(path, {});
  EXPECT_EQ(optimizer.compute(in).status, MPPIStatus::Following);
  EXPECT_GT(optimizer.rollouts(), 0);
}

// At the goal.

TEST(MppiOptimizer, AtTheGoalItStops)
{
  for (const auto & params : {diff_params(), omni_params()}) {
    easynav::MPPIOptimizer optimizer;
    optimizer.configure(params);
    const auto path = polyline({{0.0, 0.0}, {2.0, 0.0}}, 0.5);
    const auto r = optimizer.compute(input_for(path, {1.95, 0.02, 0.45}));
    EXPECT_EQ(r.status, MPPIStatus::Arrived);
    EXPECT_TRUE(is_zero(r.cmd));
    EXPECT_EQ(optimizer.rollouts(), 0);
  }
}

TEST(MppiOptimizer, AtTheGoalPositionItTurnsInPlaceToTheGoalHeading)
{
  for (const auto & params : {diff_params(), omni_params()}) {
    for (double goal_yaw : {1.5, -1.5, 3.0}) {
      Sim sim(params, {2.0, 0.0, 0.0});
      sim.path = polyline({{0.0, 0.0}, {2.0, 0.0}}, goal_yaw);
      ASSERT_TRUE(sim.run(10.0)) << "goal yaw " << goal_yaw;
      for (size_t i = 0; i + 1 < sim.cmds.size(); ++i) {
        EXPECT_EQ(sim.statuses[i], MPPIStatus::Aligning);
        EXPECT_EQ(sim.cmds[i].vx, 0.0);
        EXPECT_EQ(sim.cmds[i].vy, 0.0);
        EXPECT_GT(sim.cmds[i].wz * goal_yaw, 0.0) << "the shortest way";
      }
      EXPECT_LT(std::abs(mppi_test::normalize(sim.pose.yaw - goal_yaw)), 0.1);
      EXPECT_LT(std::hypot(sim.pose.x - 2.0, sim.pose.y), 1e-9) << "without moving";
      EXPECT_TRUE(sim.limits_ok);
    }
  }
}

TEST(MppiOptimizer, WithinTheToleranceItKeepsDrivingUnlessStopped)
{
  easynav::MPPIOptimizer optimizer;
  optimizer.configure(diff_params());
  const auto path = polyline({{0.0, 0.0}, {2.0, 0.0}}, 1.0);
  // 8 cm from the goal (more than half the 10 cm tolerance), moving: it keeps driving.
  EXPECT_EQ(
    optimizer.compute(input_for(path, {1.92, 0.0, 0.0}, {0.3, 0.0, 0.0})).status,
    MPPIStatus::Following);
  // Stopped there: no need to creep to the very goal.
  EXPECT_EQ(optimizer.compute(input_for(path, {1.92, 0.0, 0.0})).status, MPPIStatus::Aligning);
  // Out of the tolerance it drives again.
  EXPECT_EQ(optimizer.compute(input_for(path, {1.85, 0.0, 0.0})).status, MPPIStatus::Following);
}

// Obstacles.

TEST(MppiOptimizer, WhenEveryRolloutCollidesItStops)
{
  for (const auto & params : {diff_params(), omni_params()}) {
    easynav::MPPIOptimizer optimizer;
    optimizer.configure(params);
    const auto path = polyline({{0.0, 0.0}, {3.0, 0.0}}, 0.0);
    // At full speed, 0.36 m from a wall: it cannot brake (or turn) before 0.3 m.
    pcl::PointCloud<pcl::PointXYZ> wall;
    mppi_test::add_wall(wall, 0.36, -2.0, 0.36, 2.0);
    auto in = input_for(path, {}, {0.5, 0.0, 0.0});
    in.obstacles = &wall;
    const auto r = optimizer.compute(in);
    EXPECT_EQ(r.status, MPPIStatus::Blocked);
    EXPECT_TRUE(is_zero(r.cmd));
  }
}

TEST(MppiOptimizer, StartingTooCloseToAnObstacleItMovesAway)
{
  for (const auto & params : {diff_params(), omni_params()}) {
    Sim sim(params);
    sim.path = polyline({{0.0, 0.0}, {3.0, 0.0}}, 0.0);
    mppi_test::add_disc(sim.obstacles, -0.1, -0.25, 0.0);  // 0.27 m: within the radius
    ASSERT_TRUE(sim.run(20.0));
    for (const auto & s : sim.statuses) {
      EXPECT_NE(s, MPPIStatus::Blocked);
    }
  }
}

// Limits.

TEST(MppiOptimizer, ADifferentialRobotNeverMovesSidewaysAndReversesOnlyIfAllowed)
{
  auto params = diff_params();
  params.min_vx = 0.0;
  Sim sim(params);
  sim.path = polyline({{0.0, 0.0}, {-2.0, 0.0}}, M_PI);
  ASSERT_TRUE(sim.run(30.0));
  for (const auto & c : sim.cmds) {
    EXPECT_EQ(c.vy, 0.0);
    EXPECT_GE(c.vx, 0.0);
  }
  EXPECT_TRUE(sim.limits_ok);
  EXPECT_TRUE(sim.acceleration_ok);
}

TEST(MppiOptimizer, ADifferentialRobotIgnoresTheHeadingMode)
{
  // It faces along the path whatever it is told: same commands as with "path".
  std::vector<Velocity> commands[2];
  int i = 0;
  for (auto mode : {HeadingMode::Path, HeadingMode::Fixed}) {
    Sim sim(diff_params());
    sim.path = polyline({{0.0, 0.0}, {2.0, 1.0}}, 0.0);
    sim.heading_mode = mode;
    sim.heading_yaw = 2.0;
    sim.run(3.0);
    commands[i++] = sim.cmds;
  }
  ASSERT_EQ(commands[0].size(), commands[1].size());
  for (size_t k = 0; k < commands[0].size(); ++k) {
    EXPECT_EQ(commands[0][k].vx, commands[1][k].vx);
    EXPECT_EQ(commands[0][k].wz, commands[1][k].wz);
  }
}

TEST(MppiOptimizer, AnOmniRobotNeverExceedsItsSpeedInAnyDirection)
{
  // Diagonals included: (vx, vy) stays within the ellipse of the limits (checked by the sim).
  auto params = omni_params();
  params.max_vy = 0.3;
  for (const auto & corners : std::vector<std::vector<std::pair<double, double>>>{
    {{0.0, 0.0}, {3.0, 3.0}}, {{0.0, 0.0}, {-3.0, 2.0}}, {{0.0, 0.0}, {0.0, -3.0}}})
  {
    Sim sim(params);
    sim.path = polyline(corners, 0.0);
    sim.heading_mode = HeadingMode::Fixed;
    ASSERT_TRUE(sim.run(40.0));
    EXPECT_TRUE(sim.limits_ok);
    EXPECT_TRUE(sim.acceleration_ok);
    EXPECT_LE(sim.max_abs_vy(), 0.3 + 1e-9);
  }
}

TEST(MppiOptimizer, AnOmniRobotWithoutLateralSpeedMovesLikeADifferentialOne)
{
  auto params = omni_params();
  params.max_vy = 0.0;
  Sim sim(params);
  sim.path = polyline({{0.0, 0.0}, {0.0, 2.0}}, M_PI / 2.0);
  ASSERT_TRUE(sim.run(20.0));
  EXPECT_EQ(sim.max_abs_vy(), 0.0);
}

TEST(MppiOptimizer, NewLimitsApplyAtOnce)
{
  // E.g. a safely limited speed.
  Sim sim(diff_params());
  sim.path = polyline({{0.0, 0.0}, {6.0, 0.0}}, 0.0);
  sim.run(4.0);
  auto limits = sim.optimizer.params();
  limits.max_vx = 0.2;
  sim.optimizer.set_limits(limits);
  const size_t from = sim.cmds.size() + 30;  // 1 s to brake
  ASSERT_TRUE(sim.run(60.0));
  for (size_t i = from; i < sim.cmds.size(); ++i) {
    EXPECT_LE(sim.cmds[i].vx, 0.2 + 1e-9);
  }
}

// Changes.

TEST(MppiOptimizer, ANewPathIsFollowedFromWhereTheRobotIs)
{
  for (const auto & params : {diff_params(), omni_params()}) {
    Sim sim(params);
    sim.path = polyline({{0.0, 0.0}, {4.0, 0.0}}, 0.0);
    sim.run(3.0);
    sim.path = polyline({{sim.pose.x, sim.pose.y}, {sim.pose.x, sim.pose.y + 2.0}}, M_PI / 2.0);
    const double x = sim.pose.x;
    ASSERT_TRUE(sim.run(30.0));
    EXPECT_NEAR(sim.pose.x, x, 0.1);
    EXPECT_GT(sim.pose.y, 1.9);
  }
}

TEST(MppiOptimizer, AfterResetItStartsFromZero)
{
  easynav::MPPIOptimizer optimizer;
  optimizer.configure(diff_params());
  const auto path = polyline({{0.0, 0.0}, {4.0, 0.0}}, 0.0);
  for (int i = 0; i < 20; ++i) {
    optimizer.compute(input_for(path, {}, {0.5, 0.0, 0.0}));
  }
  EXPECT_GT(optimizer.control_sequence().front().vx, 0.3);
  optimizer.reset();
  for (const auto & v : optimizer.control_sequence()) {
    EXPECT_TRUE(is_zero(v));
  }
}

TEST(MppiOptimizer, ItIsRepeatable)
{
  std::vector<Velocity> commands[2];
  for (auto & c : commands) {
    Sim sim(omni_params());
    sim.path = polyline({{0.0, 0.0}, {2.0, 1.0}, {3.0, -1.0}}, 0.0);
    mppi_test::add_disc(sim.obstacles, 2.0, 0.0, 0.1);
    sim.run(5.0);
    c = sim.cmds;
  }
  ASSERT_EQ(commands[0].size(), commands[1].size());
  for (size_t k = 0; k < commands[0].size(); ++k) {
    EXPECT_EQ(commands[0][k].vx, commands[1][k].vx);
    EXPECT_EQ(commands[0][k].vy, commands[1][k].vy);
    EXPECT_EQ(commands[0][k].wz, commands[1][k].wz);
  }
}

TEST(MppiOptimizer, ACycleTakesFewMilliseconds)
{
#ifndef NDEBUG
  GTEST_SKIP() << "timing only in optimized builds";
#endif
  for (const auto & params : {diff_params(), omni_params()}) {
    easynav::MPPIOptimizer optimizer;
    optimizer.configure(params);
    const auto path = polyline({{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}}, 0.0);
    pcl::PointCloud<pcl::PointXYZ> obstacles;
    for (int i = 0; i < 2000; ++i) {
      const double a = i * 2.0 * M_PI / 2000;
      obstacles.push_back(pcl::PointXYZ(3.0 * std::cos(a), 3.0 * std::sin(a), 0.2));
    }
    auto in = input_for(path, {}, {0.3, 0.0, 0.0});
    in.obstacles = &obstacles;
    in.elapsed = 1.0 / 30.0;
    optimizer.compute(in);
    const auto start = std::chrono::steady_clock::now();
    constexpr int kCycles = 50;
    for (int i = 0; i < kCycles; ++i) {
      optimizer.compute(in);
    }
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count() / kCycles;
    // 500 rollouts of 40 steps fit in a 30 Hz cycle (about 10 ms here, twice that in CI).
    EXPECT_LT(ms, 1000.0 / 30.0);
    RecordProperty("cycle_ms", std::to_string(ms));
  }
}

// NearestGrid.

TEST(NearestGrid, FindsTheNearestSeedLikeBruteForce)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> coord(-3.0, 3.0);
  NearestGrid grid;
  grid.reserve(200);
  grid.reset(0.0, 0.0, 4.0, 0.05);
  std::vector<std::pair<double, double>> seeds;
  for (int i = 0; i < 300; ++i) {
    seeds.emplace_back(coord(rng), coord(rng));
    grid.add_seed(i, seeds.back().first, seeds.back().second);
  }
  grid.propagate();
  for (int q = 0; q < 2000; ++q) {
    const double x = coord(rng);
    const double y = coord(rng);
    double best = std::numeric_limits<double>::infinity();
    for (const auto & [sx, sy] : seeds) {
      best = std::min(best, std::hypot(x - sx, y - sy));
    }
    // Within a cell of the exact answer.
    EXPECT_LE(grid.distance(x, y, -1.0), best + 0.05 * std::sqrt(2.0));
    EXPECT_GE(grid.distance(x, y, -1.0), best - 1e-9);
  }
}

TEST(NearestGrid, WithoutSeedsThereIsNoNearest)
{
  NearestGrid grid;
  grid.reset(0.0, 0.0, 1.0, 0.1);
  grid.propagate();
  EXPECT_EQ(grid.nearest(0.0, 0.0), -1);
  EXPECT_EQ(grid.distance(0.0, 0.0, 42.0), 42.0);
}

TEST(NearestGrid, SeedsOutsideAreIgnoredAndQueriesOutsideUseTheBorder)
{
  NearestGrid grid;
  grid.reset(0.0, 0.0, 1.0, 0.1);
  grid.add_seed(7, 5.0, 0.0);              // outside
  grid.add_seed(8, std::nan(""), 0.0);     // invalid
  grid.add_seed(9, 0.5, 0.5);
  grid.propagate();
  EXPECT_EQ(grid.seeds(), 1);
  const int nearest = grid.nearest(10.0, 10.0);
  ASSERT_GE(nearest, 0);
  EXPECT_EQ(grid.seed_id(nearest), 9);
  EXPECT_NEAR(grid.distance(10.0, 10.0, -1.0), std::hypot(9.5, 9.5), 1e-9) << "exact distance";
  EXPECT_EQ(grid.nearest(std::nan(""), 0.0), -1);
}

TEST(NearestGrid, ALargerAreaThanReservedIsCoarser)
{
  NearestGrid grid;
  grid.reserve(100);
  grid.reset(0.0, 0.0, 10.0, 0.05);  // would be 400 cells per side
  EXPECT_NEAR(grid.resolution(), 0.2, 1e-12);
  grid.add_seed(0, 9.9, -9.9);
  grid.propagate();
  EXPECT_NEAR(grid.distance(-9.9, 9.9, -1.0), std::hypot(19.8, 19.8), 1e-9);
}
