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
/// \brief End to end: every motion model and heading mode, on every kind of path, drives the
/// robot to the goal within its limits, near the path, clear of obstacles and with the heading
/// of the mode.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "mppi_sim.hpp"

using easynav::HeadingMode;
using easynav::MotionModel;
using easynav::MPPIStatus;
using easynav::Pose2D;
using mppi_test::Sim;
using mppi_test::normalize;

namespace
{

using Corners = std::vector<std::pair<double, double>>;

struct Mode
{
  const char * name;
  MotionModel model;
  HeadingMode heading;
};

const Mode kModes[] = {
  {"DiffPath", MotionModel::Diff, HeadingMode::Path},
  {"OmniPath", MotionModel::Omni, HeadingMode::Path},
  {"OmniFree", MotionModel::Omni, HeadingMode::Free},
  {"OmniGoal", MotionModel::Omni, HeadingMode::Goal},
  {"OmniHold", MotionModel::Omni, HeadingMode::Hold},
  {"OmniFixed", MotionModel::Omni, HeadingMode::Fixed},
};

/// Heading of mode "fixed" in every scenario: none of the paths runs along it.
constexpr double kFixedYaw = -M_PI / 3.0;

enum class Obstacles {None, Disc, Corridor, LCorridor, Wall};

struct Scenario
{
  const char * name;
  Pose2D start;
  Corners corners;
  double goal_yaw;
  Obstacles obstacles {Obstacles::None};
  double max_deviation {0.15};  ///< From the path (m), corners and detours aside.
  bool reachable {true};
};

const Scenario kScenarios[] = {
  {"Straight", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {4.0, 0.0}}, 0.0},
  {"LeftTurn", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}}, M_PI / 2.0,
    Obstacles::None, 0.35},
  {"RightTurn", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {3.0, 0.0}, {3.0, -3.0}}, -M_PI / 2.0,
    Obstacles::None, 0.35},
  {"UTurn", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {3.0, 0.0}, {3.0, 1.5}, {0.0, 1.5}}, M_PI,
    Obstacles::None, 0.35},
  {"Behind", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {-3.0, 0.0}}, M_PI},
  {"Sideways", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {0.0, 3.0}}, M_PI / 2.0},
  {"Diagonal", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {3.0, 3.0}}, M_PI / 4.0},
  {"SCurve", {0.0, 0.0, 0.0},
    {{0.0, 0.0}, {1.0, 0.0}, {1.7, 0.3}, {2.3, 1.0}, {3.0, 1.3}, {4.0, 1.3}}, 0.0,
    Obstacles::None, 0.25},
  {"GoalOnly", {0.0, 0.0, 0.0}, {{1.5, 0.5}}, -M_PI / 2.0, Obstacles::None, 0.6},
  {"TurnInPlace", {0.0, 0.0, 0.0}, {{0.0, 0.0}}, 2.5},
  {"ObstacleOnPath", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {5.0, 0.0}}, 0.0, Obstacles::Disc, 1.0},
  {"NarrowCorridor", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {5.0, 0.0}}, 0.0, Obstacles::Corridor},
  {"LCorridor", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}}, M_PI / 2.0,
    Obstacles::LCorridor, 0.3},
  {"Wall", {0.0, 0.0, 0.0}, {{0.0, 0.0}, {5.0, 0.0}}, 0.0, Obstacles::Wall, 1.0, false},
  {"LongCurvy", {0.0, 0.0, 0.0},
    {{0.0, 0.0}, {4.0, 0.0}, {6.0, 2.0}, {6.0, 5.0}, {3.0, 6.0}, {0.0, 6.0}}, M_PI,
    Obstacles::None, 0.35},
  {"LoopNearStart", {0.0, 0.0, 0.0},
    {{0.0, 0.0}, {2.5, 0.0}, {2.5, 2.0}, {0.0, 2.0}, {0.0, 0.8}}, -M_PI / 2.0,
    Obstacles::None, 0.35},
};

double path_length(const Corners & c)
{
  double l = 0.0;
  for (size_t i = 0; i + 1 < c.size(); ++i) {
    l += std::hypot(c[i + 1].first - c[i].first, c[i + 1].second - c[i].second);
  }
  return l;
}

void add_obstacles(Sim & sim, Obstacles kind)
{
  switch (kind) {
    case Obstacles::None:
      break;
    case Obstacles::Disc:
      mppi_test::add_disc(sim.obstacles, 2.5, 0.0, 0.15);
      break;
    case Obstacles::Corridor:
      // 1.1 m wide: 0.25 m to spare on each side of a 0.6 m robot.
      mppi_test::add_wall(sim.obstacles, -1.0, 0.55, 6.0, 0.55);
      mppi_test::add_wall(sim.obstacles, -1.0, -0.55, 6.0, -0.55);
      break;
    case Obstacles::LCorridor:
      mppi_test::add_wall(sim.obstacles, -1.0, 0.6, 2.4, 0.6);
      mppi_test::add_wall(sim.obstacles, 2.4, 0.6, 2.4, 4.0);
      mppi_test::add_wall(sim.obstacles, -1.0, -0.6, 3.6, -0.6);
      mppi_test::add_wall(sim.obstacles, 3.6, -0.6, 3.6, 4.0);
      break;
    case Obstacles::Wall:
      mppi_test::add_wall(sim.obstacles, 2.0, -3.0, 2.0, 3.0);
      break;
  }
}

/// Direction from (x, y) to the point of \p c \p ahead meters along it past the nearest
/// one (what heading_mode "path" faces), and the distance to its nearest inner corner.
std::pair<double, double> path_heading_at(const Corners & c, double x, double y, double ahead)
{
  // Nearest point: segment and arc length.
  double best = std::numeric_limits<double>::infinity();
  double s_best = 0.0;
  double s = 0.0;
  for (size_t i = 0; i + 1 < c.size(); ++i) {
    const double dx = c[i + 1].first - c[i].first;
    const double dy = c[i + 1].second - c[i].second;
    const double len = std::hypot(dx, dy);
    const double f = len > 0.0 ?
      std::clamp(((x - c[i].first) * dx + (y - c[i].second) * dy) / (len * len), 0.0, 1.0) : 0.0;
    const double d = std::hypot(x - (c[i].first + f * dx), y - (c[i].second + f * dy));
    if (d < best) {
      best = d;
      s_best = s + f * len;
    }
    s += len;
  }
  // The point ahead.
  double target = s_best + ahead;
  double tx = c.back().first;
  double ty = c.back().second;
  s = 0.0;
  for (size_t i = 0; i + 1 < c.size(); ++i) {
    const double len = std::hypot(c[i + 1].first - c[i].first, c[i + 1].second - c[i].second);
    if (s + len >= target) {
      const double f = len > 0.0 ? (target - s) / len : 0.0;
      tx = c[i].first + f * (c[i + 1].first - c[i].first);
      ty = c[i].second + f * (c[i + 1].second - c[i].second);
      break;
    }
    s += len;
  }
  double corner = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i + 1 < c.size(); ++i) {
    corner = std::min(corner, std::hypot(c[i].first - x, c[i].second - y));
  }
  return {std::atan2(ty - y, tx - x), corner};
}

}  // namespace

class MppiScenarioTest : public ::testing::TestWithParam<std::tuple<Mode, Scenario>> {};

TEST_P(MppiScenarioTest, ReachesTheGoalAsTheModeSays)
{
  const auto & [mode, scenario] = GetParam();
  auto params = mode.model == MotionModel::Diff ? mppi_test::diff_params() :
    mppi_test::omni_params();
  Sim sim(params, scenario.start);
  sim.heading_mode = mode.heading == HeadingMode::Hold ? HeadingMode::Fixed : mode.heading;
  sim.heading_yaw = mode.heading == HeadingMode::Hold ? scenario.start.yaw : kFixedYaw;
  sim.xy_tolerance = 0.1;
  sim.yaw_tolerance = 0.1;
  sim.path = mppi_test::polyline(scenario.corners, scenario.goal_yaw);
  add_obstacles(sim, scenario.obstacles);

  const auto & goal = scenario.corners.back();
  const double length = path_length(scenario.corners) +
    std::hypot(goal.first - scenario.start.x, goal.second - scenario.start.y);
  const double max_time = 15.0 + 3.0 * length / params.max_vx;
  const bool arrived = sim.run(max_time);
  const std::string where = "t=" + std::to_string(sim.time) + " pose=(" +
    std::to_string(sim.pose.x) + ", " + std::to_string(sim.pose.y) + ", " +
    std::to_string(sim.pose.yaw) + ")";

  // Always: within the limits, clear of the obstacles.
  EXPECT_TRUE(sim.limits_ok);
  EXPECT_TRUE(sim.acceleration_ok);
  EXPECT_GT(sim.min_clearance, 0.0) << where;
  if (mode.model == MotionModel::Diff) {
    EXPECT_DOUBLE_EQ(sim.max_abs_vy(), 0.0);
  }

  if (!scenario.reachable) {
    EXPECT_FALSE(arrived);
    EXPECT_LT(std::hypot(sim.velocity.vx, sim.velocity.vy), 0.05) << "stopped: " << where;
    return;
  }

  ASSERT_TRUE(arrived) << where;
  EXPECT_LE(std::hypot(sim.pose.x - goal.first, sim.pose.y - goal.second), 0.1 + 1e-3) << where;
  EXPECT_LE(std::abs(normalize(sim.pose.yaw - scenario.goal_yaw)), 0.1 + 1e-3) << where;

  // Near the path.
  if (scenario.corners.size() > 1) {
    double deviation = 0.0;
    for (const auto & p : sim.trace) {
      deviation = std::max(deviation, mppi_test::distance_to(scenario.corners, p.x, p.y));
    }
    // A differential robot turning around at the start arcs a little (it is its steering).
    const double allowed = mode.model == MotionModel::Diff ?
      std::max(scenario.max_deviation, 0.2) : scenario.max_deviation;
    EXPECT_LT(deviation, allowed);
  }

  // The heading of the mode: once reached (turning takes a while), kept while moving, away
  // from the goal (where it turns to the goal's) and from the corners of the path (where
  // "path" turns to the next leg).
  const double goal_zone = params.goal_heading_distance + 0.2;
  bool settled = false;
  int checked = 0;
  double worst = 0.0;
  for (const auto & p : sim.trace) {
    if (std::hypot(p.x - goal.first, p.y - goal.second) < goal_zone) {
      continue;
    }
    double target;
    switch (mode.heading) {
      case HeadingMode::Fixed: target = kFixedYaw; break;
      case HeadingMode::Hold: target = scenario.start.yaw; break;
      case HeadingMode::Goal: target = scenario.goal_yaw; break;
      case HeadingMode::Path: {
          // A differential robot's heading is its course: checked as the path deviation.
          if (mode.model == MotionModel::Diff) {
            continue;
          }
          const auto [yaw, corner] = path_heading_at(
            scenario.corners, p.x, p.y, params.heading_lookahead);
          if (scenario.corners.size() < 2 || corner < 1.0) {
            continue;
          }
          target = yaw;
          break;
        }
      default: continue;
    }
    ++checked;
    const double error = std::abs(normalize(p.yaw - target));
    settled = settled || error < 0.1;
    if (settled) {
      worst = std::max(worst, error);
    }
  }
  if (scenario.obstacles == Obstacles::None) {
    // Long enough to turn around (a short path may end before).
    if (checked * sim.period > 4.0) {
      EXPECT_TRUE(settled) << "never took the heading of the mode";
    }
    EXPECT_LT(worst, 0.2) << "heading of the mode, once taken";
  }
}

INSTANTIATE_TEST_SUITE_P(
  Matrix, MppiScenarioTest,
  ::testing::Combine(::testing::ValuesIn(kModes), ::testing::ValuesIn(kScenarios)),
  [](const ::testing::TestParamInfo<MppiScenarioTest::ParamType> & info) {
    return std::string(std::get<0>(info.param).name) + "_" + std::get<1>(info.param).name;
  });

// Non-round robots: the footprint, turned with the robot, against the obstacles.

namespace
{

Sim rectangle_sim(MotionModel model, double length, double width, Pose2D start)
{
  auto params = model == MotionModel::Diff ? mppi_test::diff_params() : mppi_test::omni_params();
  mppi_test::set_rectangle(params, length, width);
  return Sim(params, start);
}

}  // namespace

TEST(MppiFootprint, ANarrowCorridorFitsTheRectangleButNotItsCircle)
{
  // 0.6 m wide: a 0.8 x 0.4 m robot fits lengthwise; its circle (0.89 m across) does not.
  struct Case {MotionModel model; HeadingMode heading;};
  for (const auto & c : {Case{MotionModel::Diff, HeadingMode::Path},
      Case{MotionModel::Omni, HeadingMode::Path}, Case{MotionModel::Omni, HeadingMode::Fixed}})
  {
    Sim sim = rectangle_sim(c.model, 0.8, 0.4, {0.0, 0.0, 0.0});
    sim.heading_mode = c.heading;
    sim.heading_yaw = 0.0;
    sim.path = mppi_test::polyline({{0.0, 0.0}, {5.0, 0.0}}, 0.0);
    mppi_test::add_wall(sim.obstacles, -1.0, 0.3, 6.0, 0.3);
    mppi_test::add_wall(sim.obstacles, -1.0, -0.3, 6.0, -0.3);
    EXPECT_TRUE(sim.run(60.0)) << "model " << static_cast<int>(c.model);
    EXPECT_GT(sim.min_clearance, 0.0);
  }

  // The same robot as a circle: it cannot move without touching the walls.
  auto params = mppi_test::diff_params();
  params.robot_radius = std::hypot(0.4, 0.2);
  params.inscribed_radius = params.robot_radius;
  params.safety_radius = params.robot_radius + 0.1;
  Sim circle(params);
  circle.path = mppi_test::polyline({{0.0, 0.0}, {5.0, 0.0}}, 0.0);
  mppi_test::add_wall(circle.obstacles, -1.0, 0.3, 6.0, 0.3);
  mppi_test::add_wall(circle.obstacles, -1.0, -0.3, 6.0, -0.3);
  EXPECT_FALSE(circle.run(10.0));
  EXPECT_LT(std::hypot(circle.pose.x, circle.pose.y), 0.05);
}

TEST(MppiFootprint, ADoorIsCrossedByTurningTheRobot)
{
  // A 0.7 m door; the 1.0 x 0.5 m robot arrives across it and must turn to pass.
  auto door = [](Sim & sim) {
      mppi_test::add_wall(sim.obstacles, 2.0, -3.0, 2.0, -0.35);
      mppi_test::add_wall(sim.obstacles, 2.0, 0.35, 2.0, 3.0);
    };
  for (auto model : {MotionModel::Diff, MotionModel::Omni}) {
    Sim sim = rectangle_sim(model, 1.0, 0.5, {0.0, 0.0, M_PI / 2.0});
    sim.heading_mode = HeadingMode::Free;
    sim.path = mppi_test::polyline({{0.0, 0.0}, {4.0, 0.0}}, 0.0);
    door(sim);
    EXPECT_TRUE(sim.run(60.0)) << "model " << static_cast<int>(model);
    EXPECT_GT(sim.min_clearance, 0.0);
  }

  // Told to keep facing across it: it does not fit, and it does not try.
  Sim across = rectangle_sim(MotionModel::Omni, 1.0, 0.5, {0.0, 0.0, M_PI / 2.0});
  across.heading_mode = HeadingMode::Fixed;
  across.heading_yaw = M_PI / 2.0;
  across.path = mppi_test::polyline({{0.0, 0.0}, {4.0, 0.0}}, M_PI / 2.0);
  door(across);
  EXPECT_FALSE(across.run(30.0));
  EXPECT_GT(across.min_clearance, 0.0);
}

TEST(MppiFootprint, ALongRobotDoesNotSweepAWallTurningInPlace)
{
  // 1.0 x 0.4 m beside a wall 0.45 m away: turning in place, its corners would hit it.
  Sim sim = rectangle_sim(MotionModel::Diff, 1.0, 0.4, {0.0, 0.0, 0.0});
  sim.path = mppi_test::polyline({{0.0, 0.0}, {-3.0, 0.0}}, M_PI);
  mppi_test::add_wall(sim.obstacles, -5.0, 0.45, 5.0, 0.45);
  EXPECT_TRUE(sim.run(60.0));
  EXPECT_GT(sim.min_clearance, 0.0);
}
