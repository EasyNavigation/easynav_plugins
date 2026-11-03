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
/// \brief The 3D planner's core: the clearance grid and A* with its clearance.

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "easynav_bonxai_planner/Planner3D.hpp"

using easynav::bonxai_planner::ClearanceGrid;
using easynav::bonxai_planner::PlannerParams;
using Eigen::Vector3d;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A wall at x = 0 over y, z in [-10, 10], with a square window of half side \p hole (0: none).
std::vector<Vector3d> wall(double hole)
{
  std::vector<Vector3d> pts;
  for (double y = -10.0; y <= 10.0; y += 0.25) {
    for (double z = -10.0; z <= 10.0; z += 0.25) {
      if (std::abs(y) < hole && std::abs(z) < hole) {continue;}
      pts.emplace_back(0.0, y, z);
    }
  }
  return pts;
}

double length(const std::vector<Vector3d> & path)
{
  double l = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    l += (path[i] - path[i - 1]).norm();
  }
  return l;
}

/// The least clearance along the path, sampled every 10 cm.
double least_clearance(const ClearanceGrid & grid, const std::vector<Vector3d> & path)
{
  double least = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const int n = std::max(1, static_cast<int>((path[i] - path[i - 1]).norm() / 0.1));
    for (int k = 0; k <= n; ++k) {
      least = std::min(least, grid.distance_at(path[i - 1] + (path[i] - path[i - 1]) * k / n));
    }
  }
  return least;
}

}  // namespace

TEST(ClearanceGrid, DistanceToTheNearestObstacle)
{
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build({{0.0, 0.0, 0.0}}, 0.5, 5.0, kNaN, kNaN));
  EXPECT_NEAR(grid.distance_at({0.0, 0.0, 0.0}), 0.0, 1e-6);
  EXPECT_NEAR(grid.distance_at({3.1, 0.1, 0.1}), 3.0, 0.5);
  EXPECT_NEAR(grid.distance_at({2.1, 2.1, 2.1}), std::sqrt(12.0), 0.5);
  EXPECT_DOUBLE_EQ(grid.distance_at({50.0, 0.0, 0.0}), 0.0) << "outside: no clearance";
}

TEST(ClearanceGrid, AltitudeLimitsAndSize)
{
  ClearanceGrid grid;
  EXPECT_FALSE(grid.build({}, 0.5, 5.0, kNaN, kNaN)) << "no obstacles";
  EXPECT_FALSE(grid.build({{0, 0, 0}}, 0.0, 5.0, kNaN, kNaN)) << "no resolution";
  EXPECT_FALSE(grid.build({{0, 0, 0}}, 0.5, 5.0, 3.0, 2.0)) << "empty altitude band";
  const std::vector<Eigen::Vector3d> origin = {{0, 0, 0}};
  EXPECT_FALSE(
    grid.build(origin, 0.01, 100.0, kNaN, kNaN, easynav::bonxai_planner::FillBelow::None, 1000000))
    << "too many cells";
  ASSERT_TRUE(grid.build({{0, 0, 0}}, 1.0, 5.0, -1.0, 20.0));
  EXPECT_NEAR(grid.origin().z(), -1.0, 1e-9);
  EXPECT_EQ(grid.nz(), 21);
}

TEST(Planner3D, StraightThroughFreeSpace)
{
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build({{0, 0, -20}, {0, 0, 20}}, 0.5, 15.0, kNaN, kNaN));
  const auto path = easynav::bonxai_planner::plan(grid, {-10, 0, 0}, {10, 3, 2}, {});
  ASSERT_EQ(path.size(), 2u) << "one straight segment";
  EXPECT_TRUE(path.front().isApprox(Vector3d(-10, 0, 0)));
  EXPECT_TRUE(path.back().isApprox(Vector3d(10, 3, 2)));
}

TEST(Planner3D, AlreadyAtTheGoalIsAPathToIt)
{
  // At the goal (one cell): a path from the robot to the goal, so the controller can turn to
  // the goal's heading, instead of none
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build({{0, 0, -20}, {0, 0, 20}}, 1.0, 15.0, kNaN, kNaN));
  const auto path = easynav::bonxai_planner::plan(grid, {5.1, 0.2, 0.3}, {5.3, 0.1, 0.4}, {});
  ASSERT_EQ(path.size(), 2u);
  EXPECT_TRUE(path.front().isApprox(Vector3d(5.1, 0.2, 0.3)));
  EXPECT_TRUE(path.back().isApprox(Vector3d(5.3, 0.1, 0.4)));
}

TEST(Planner3D, ThroughAWindowWideEnough)
{
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build(wall(2.5), 0.25, 6.0, kNaN, kNaN));
  PlannerParams p;
  p.min_clearance = 1.0;
  p.preferred_clearance = 0.0;
  const auto path = easynav::bonxai_planner::plan(grid, {-8, 0, 0}, {8, 0, 0}, p);
  ASSERT_GE(path.size(), 2u);
  EXPECT_LT(length(path), 17.0) << "straight through the window";
  EXPECT_GE(least_clearance(grid, path), 1.0 - 0.25);
}

TEST(Planner3D, AroundTheWallWhenTheWindowIsTooNarrow)
{
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build(wall(2.0), 0.25, 6.0, kNaN, kNaN));
  PlannerParams p;
  p.min_clearance = 2.5;  // more than the window's half side
  p.preferred_clearance = 0.0;
  const auto path = easynav::bonxai_planner::plan(grid, {-8, 0, 0}, {8, 0, 0}, p);
  ASSERT_GE(path.size(), 3u);
  EXPECT_GT(length(path), 25.0) << "around the wall's edge";
  EXPECT_GE(least_clearance(grid, path), 2.5 - 0.25);
}

TEST(Planner3D, PrefersToKeepItsDistance)
{
  // With a preferred clearance, the path through a wide window stays near its centre
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build(wall(4.0), 0.25, 6.0, kNaN, kNaN));
  PlannerParams p;
  p.min_clearance = 1.0;
  p.preferred_clearance = 4.0;
  p.clearance_weight = 5.0;
  const auto path = easynav::bonxai_planner::plan(grid, {-8, 3, 3}, {8, 3, 3}, p);
  ASSERT_GE(path.size(), 2u);
  double at_wall = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if ((path[i - 1].x() < 0.0) != (path[i].x() < 0.0)) {
      const double t = -path[i - 1].x() / (path[i].x() - path[i - 1].x());
      at_wall = grid.distance_at(path[i - 1] + t * (path[i] - path[i - 1]));
    }
  }
  EXPECT_GT(at_wall, 2.0) << "crosses the window farther from its edges than it must";
}

TEST(Planner3D, NoPathToAnEnclosedGoal)
{
  std::vector<Vector3d> box;
  for (double a = -3.0; a <= 3.0; a += 0.25) {
    for (double b = -3.0; b <= 3.0; b += 0.25) {
      for (double s : {-3.0, 3.0}) {
        box.emplace_back(s, a, b);
        box.emplace_back(a, s, b);
        box.emplace_back(a, b, s);
      }
    }
  }
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build(box, 0.25, 6.0, kNaN, kNaN));
  PlannerParams p;
  p.min_clearance = 1.0;
  p.snap_radius = 1.0;
  EXPECT_TRUE(easynav::bonxai_planner::plan(grid, {-7, 0, 0}, {0, 0, 0}, p).empty());
}

TEST(Planner3D, AStartTooCloseToAnObstacleMovesAwayFirst)
{
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build(wall(0.0), 0.25, 6.0, kNaN, kNaN));
  PlannerParams p;
  p.min_clearance = 1.0;
  // 0.4 m from the wall: the first cell with 1 m of clearance, then on
  const auto path = easynav::bonxai_planner::plan(grid, {-0.4, 0, 0}, {-5, 2, 0}, p);
  ASSERT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x(), -1.1, 0.3);
  EXPECT_TRUE(path.back().isApprox(Vector3d(-5, 2, 0)));
}

TEST(Planner3D, TheAltitudeBandStopsItFromFlyingOver)
{
  // A wall from the floor up to 10 m, 40 m long: over it is shorter, unless the band is lower
  std::vector<Vector3d> pts;
  for (double y = -20.0; y <= 20.0; y += 0.5) {
    for (double z = 0.0; z <= 10.0; z += 0.5) {
      pts.emplace_back(0.0, y, z);
    }
  }
  for (double x = -10.0; x <= 10.0; x += 0.5) {
    for (double y = -25.0; y <= 25.0; y += 0.5) {
      pts.emplace_back(x, y, 0.0);
    }                                                                           // the floor
  }
  PlannerParams p;
  p.min_clearance = 1.0;
  p.preferred_clearance = 0.0;
  ClearanceGrid high, low;
  ASSERT_TRUE(high.build(pts, 0.5, 5.0, 0.0, 20.0));
  ASSERT_TRUE(low.build(pts, 0.5, 5.0, 0.0, 8.0));
  const auto over = easynav::bonxai_planner::plan(high, {-5, 0, 3}, {5, 0, 3}, p);
  const auto around = easynav::bonxai_planner::plan(low, {-5, 0, 3}, {5, 0, 3}, p);
  ASSERT_FALSE(over.empty());
  ASSERT_FALSE(around.empty());
  double top = 0.0;
  for (const auto & q : over) {
    top = std::max(top, q.z());
  }
  EXPECT_GT(top, 10.5) << "over the wall";
  for (const auto & q : around) {
    EXPECT_LE(q.z(), 8.0);
  }
  EXPECT_GT(length(around), length(over));
}

TEST(Planner3D, FillBelowKeepsItOutOfTheGroundAndOfBuildings)
{
  using easynav::bonxai_planner::FillBelow;
  // A floor at z = 0 with a 6 m gap, and a building scanned only from outside: its walls and
  // roof, nothing inside (a hollow shell for the sensors)
  std::vector<Vector3d> pts;
  for (double x = -20.0; x <= 20.0; x += 0.5) {
    for (double y = -20.0; y <= 20.0; y += 0.5) {
      if (std::abs(x + 10.0) < 3.0 && std::abs(y) < 3.0) {continue;}  // the gap
      pts.emplace_back(x, y, 0.0);
    }
  }
  for (double a = -4.0; a <= 4.0; a += 0.5) {
    for (double z = 0.0; z <= 12.0; z += 0.5) {
      for (double s : {-4.0, 4.0}) {
        pts.emplace_back(5.0 + s, a, z);
        pts.emplace_back(5.0 + a, s, z);
      }
    }
    for (double b = -4.0; b <= 4.0; b += 0.5) {
      pts.emplace_back(5.0 + a, b, 12.0);
    }
  }
  PlannerParams p;
  p.min_clearance = 1.5;
  p.preferred_clearance = 0.0;
  p.snap_radius = 0.5;

  ClearanceGrid none, lowest, highest;
  ASSERT_TRUE(none.build(pts, 0.5, 6.0, -6.0, 20.0, FillBelow::None));
  ASSERT_TRUE(lowest.build(pts, 0.5, 6.0, -6.0, 20.0, FillBelow::Lowest));
  ASSERT_TRUE(highest.build(pts, 0.5, 6.0, -6.0, 20.0, FillBelow::Highest));

  // Under the floor, through its gap: only without fill
  EXPECT_FALSE(easynav::bonxai_planner::plan(none, {-10, 0, 5}, {-15, 10, -4}, p).empty());
  EXPECT_TRUE(easynav::bonxai_planner::plan(lowest, {-10, 0, 5}, {-15, 10, -4}, p).empty())
    << "the gap is filled from its neighbours: the ground is solid";

  // Inside the building: reachable through nothing, but its inside is free space unless it
  // is filled up to the roof
  EXPECT_GT(none.distance_at({5, 0, 6}), 2.0);
  EXPECT_DOUBLE_EQ(highest.distance_at({5, 0, 6}), 0.0) << "a solid block";
  EXPECT_GT(lowest.distance_at({5, 0, 6}), 2.0) << "lowest only fills the ground";

  // Over it with highest: the path keeps its distance from the roof
  const auto over = easynav::bonxai_planner::plan(highest, {-5, 0, 4}, {15, 0, 4}, p);
  ASSERT_FALSE(over.empty());
  EXPECT_GE(least_clearance(highest, over), 1.5 - 0.5);
}

TEST(Planner3D, FillBelowNames)
{
  using easynav::bonxai_planner::FillBelow;
  FillBelow f;
  EXPECT_TRUE(easynav::bonxai_planner::parse_fill_below("highest", f));
  EXPECT_EQ(f, FillBelow::Highest);
  EXPECT_TRUE(easynav::bonxai_planner::parse_fill_below("none", f));
  EXPECT_EQ(f, FillBelow::None);
  EXPECT_FALSE(easynav::bonxai_planner::parse_fill_below("top", f));
}

namespace
{

/// The largest turn (rad) between consecutive segments of a path.
double sharpest_turn(const std::vector<Vector3d> & path)
{
  double worst = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const Vector3d a = (path[i] - path[i - 1]).normalized();
    const Vector3d b = (path[i + 1] - path[i]).normalized();
    worst = std::max(worst, std::acos(std::clamp(a.dot(b), -1.0, 1.0)));
  }
  return worst;
}

}  // namespace

TEST(Planner3D, TheSmootherRoundsCornersAndKeepsTheEnds)
{
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build({{0, 0, -30}, {0, 0, 30}}, 0.5, 30.0, kNaN, kNaN));
  // A 90 deg corner in the horizontal plane and then a climb
  const std::vector<Vector3d> corner = {{10, 0, 0}, {20, 0, 0}, {20, 10, 0}, {20, 10, 8}};
  const auto smooth = easynav::bonxai_planner::smooth(grid, corner, 0.5, 1.0, {});
  ASSERT_GT(smooth.size(), 50u);
  EXPECT_TRUE(smooth.front().isApprox(corner.front()));
  EXPECT_TRUE(smooth.back().isApprox(corner.back()));
  EXPECT_LT(sharpest_turn(smooth), 0.35) << "no corner left (it was 90 deg)";
  for (std::size_t i = 1; i < smooth.size(); ++i) {
    EXPECT_LT((smooth[i] - smooth[i - 1]).norm(), 0.6);
  }
  // No smoothing: the corners stay
  easynav::bonxai_planner::SmootherParams none;
  none.iterations = 0;
  const auto kept = easynav::bonxai_planner::smooth(grid, corner, 0.5, 1.0, none);
  EXPECT_NEAR(sharpest_turn(kept), M_PI / 2.0, 1e-6);
}

TEST(Planner3D, TheSmootherNeverComesCloserToAnObstacle)
{
  // A corner around the edge of a wall: rounding it would cut through the wall's clearance
  ClearanceGrid grid;
  ASSERT_TRUE(grid.build(wall(0.0), 0.25, 8.0, kNaN, kNaN));
  PlannerParams p;
  p.min_clearance = 1.5;
  p.preferred_clearance = 0.0;
  const auto path = easynav::bonxai_planner::plan(grid, {-5, 0, 0}, {5, 0, 0}, p);
  ASSERT_GE(path.size(), 3u) << "around the wall";
  const auto smooth = easynav::bonxai_planner::smooth(grid, path, 0.25, p.min_clearance, {});
  EXPECT_GE(least_clearance(grid, smooth), p.min_clearance - 0.25);
  EXPECT_LE(sharpest_turn(smooth), sharpest_turn(path) + 1e-9);
}
