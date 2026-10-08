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

/// \file
/// \brief Tests for the NavMap A* planner: a failed plan leaves an empty path, not the previous
/// one.

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_planner/AStarPlanner.hpp"

class NavMapPlannerTest : public ::testing::Test
{
protected:
  static constexpr int kCells = 40;
  static constexpr double kResolution = 0.1;

  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // One node per test: initialize() declares parameters.
    static int test_count = 0;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "navmap_planner_test_" + std::to_string(test_count++));
    planner_ = std::make_shared<easynav::navmap::AStarPlanner>();
    planner_->initialize(node_, "planner");

    // Free 4 x 4 m grid, robot near a corner.
    grid_.header.frame_id = "map";
    grid_.info.resolution = kResolution;
    grid_.info.width = kCells;
    grid_.info.height = kCells;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(kCells * kCells, 0);
    update_map();

    nav_msgs::msg::Odometry robot;
    robot.header.frame_id = "map";
    robot.pose.pose.position.x = 0.55;
    robot.pose.pose.position.y = 0.55;
    robot.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", robot);
  }

  // Rebuilds the NavMap from grid_, with the "obstacles" layer the planner reads.
  void update_map()
  {
    auto navmap = navmap_ros::from_occupancy_grid(grid_);
    for (std::size_t c = 0; c < navmap.navcels.size(); ++c) {
      const auto cid = static_cast<::navmap::NavCelId>(c);
      navmap.layer_set<std::uint8_t>(
        "obstacles", cid, navmap.layer_get<std::uint8_t>("occupancy", cid, 0));
    }
    nav_state_.set("map.navmap", navmap);
  }

  void occupy_cell(int x, int y)
  {
    grid_.data[y * kCells + x] = 100;
  }

  int to_cell(double m) const {return static_cast<int>(m / kResolution);}

  // Occupied square ring of half side r cells around (x, y), in meters.
  void wall_off(double x, double y, int r = 2)
  {
    const int cx = to_cell(x), cy = to_cell(y);
    for (int d = -r; d <= r; ++d) {
      occupy_cell(cx + d, cy - r);
      occupy_cell(cx + d, cy + r);
      occupy_cell(cx - r, cy + d);
      occupy_cell(cx + r, cy + d);
    }
    update_map();
  }

  // Vertical wall at x across the map.
  void vertical_wall(double x)
  {
    for (int y = 0; y < kCells; ++y) {
      occupy_cell(to_cell(x), y);
    }
    update_map();
  }

  void set_goal(double x, double y, const std::string & frame = "map")
  {
    nav_msgs::msg::Goals goals;
    goals.header.frame_id = frame;
    goals.header.stamp = node_->now();
    geometry_msgs::msg::PoseStamped goal;
    goal.header = goals.header;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    goals.goals.push_back(goal);
    nav_state_.set("goals", goals);
  }

  // Marks as inscribed (cost 253) the "inflated_obstacles" layer of the cells whose centroid
  // satisfies \p inside; every other cell is free in that layer.
  template<typename Pred>
  void inscribe(Pred inside)
  {
    inflate([&](float x, float y) -> std::uint8_t {return inside(x, y) ? 253 : 0;});
  }

  // Sets the "inflated_obstacles" layer of every cell to \p cost of its centroid.
  template<typename Cost>
  void inflate(Cost cost)
  {
    auto navmap = nav_state_.get<::navmap::NavMap>("map.navmap");
    for (std::size_t c = 0; c < navmap.navcels.size(); ++c) {
      const auto cid = static_cast<::navmap::NavCelId>(c);
      const auto p = navmap.navcel_centroid(cid);
      navmap.layer_set<std::uint8_t>("inflated_obstacles", cid, cost(p.x(), p.y()));
    }
    nav_state_.set("map.navmap", navmap);
  }

  void set_robot(double x, double y)
  {
    nav_msgs::msg::Odometry robot;
    robot.header.frame_id = "map";
    robot.pose.pose.position.x = x;
    robot.pose.pose.position.y = y;
    robot.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", robot);
  }

  // Path length inside the box [x0, x1] x [y0, y1].
  double length_inside(double x0, double x1, double y0, double y1) const
  {
    const auto p = path();
    double len = 0.0;
    for (std::size_t i = 1; i < p.poses.size(); ++i) {
      const auto & a = p.poses[i - 1].pose.position;
      const auto & b = p.poses[i].pose.position;
      const double mx = (a.x + b.x) / 2.0, my = (a.y + b.y) / 2.0;
      if (mx > x0 && mx < x1 && my > y0 && my < y1) {
        len += std::hypot(b.x - a.x, b.y - a.y);
      }
    }
    return len;
  }

  nav_msgs::msg::Path path() const
  {
    return nav_state_.get<nav_msgs::msg::Path>("path");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::navmap::AStarPlanner> planner_;
  nav_msgs::msg::OccupancyGrid grid_;
  easynav::NavState nav_state_;
};

TEST_F(NavMapPlannerTest, ReachableGoalProducesAPathEndingNearTheGoal)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);

  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_NEAR(p.poses.back().pose.position.x, 3.05, 2 * kResolution);
  EXPECT_NEAR(p.poses.back().pose.position.y, 3.05, 2 * kResolution);
}

TEST_F(NavMapPlannerTest, PathEndsExactlyAtTheGoalNotAtItsCellCentroid)
{
  // Goals away from any NavCel centroid, including one in the robot's own cell.
  for (const auto & [x, y] : std::vector<std::pair<double, double>>{
    {3.07, 3.02}, {2.012, 0.987}, {0.57, 0.52}})
  {
    set_goal(x, y);
    planner_->update(nav_state_);
    const auto p = path();
    ASSERT_FALSE(p.poses.empty()) << x << ", " << y;
    // The smoother works in float.
    EXPECT_NEAR(p.poses.back().pose.position.x, x, 1e-5);
    EXPECT_NEAR(p.poses.back().pose.position.y, y, 1e-5);
    EXPECT_DOUBLE_EQ(p.poses.back().pose.orientation.w, 1.0);
  }
}

TEST_F(NavMapPlannerTest, InscribedCellsAreNotCrossed)
{
  // A wall of inscribed cells (the robot would touch an obstacle) across the map, no lethal one.
  inscribe([](float x, float) {return x > 1.8f && x < 2.2f;});
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "crossed an inscribed wall";

  // With a free opening in it, the path goes through the opening.
  inscribe([](float x, float y) {return x > 1.8f && x < 2.2f && (y < 1.5f || y > 2.5f);});
  set_goal(3.06, 3.04);
  planner_->update(nav_state_);
  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  for (const auto & ps : p.poses) {
    const auto & pt = ps.pose.position;
    if (pt.x > 1.8 && pt.x < 2.2) {
      EXPECT_GT(pt.y, 1.4) << "through the inscribed part at x " << pt.x;
      EXPECT_LT(pt.y, 2.6) << "through the inscribed part at x " << pt.x;
    }
  }
}

TEST_F(NavMapPlannerTest, ARobotWithinTheInscribedBandCanLeaveIt)
{
  // Inscribed around the robot (0.55, 0.55), as after a stop next to an obstacle.
  inscribe([](float x, float y) {return std::hypot(x - 0.55f, y - 0.55f) < 0.4f;});
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  EXPECT_FALSE(path().poses.empty());
}

// Cost 200 in the box [1.5, 2.5] x [1.2, 2.8].
std::uint8_t costly_box(float x, float y)
{
  return (x > 1.5f && x < 2.5f && y > 1.2f && y < 2.8f) ? 200 : 0;
}

// Cost growing up to 250 towards y = 2, for x in [1, 3].
std::uint8_t cost_gradient(float x, float y)
{
  const float d = std::abs(y - 2.0f);
  return (x > 1.0f && x < 3.0f && d < 1.0f) ? static_cast<std::uint8_t>(250 * (1.0f - d)) : 0;
}

TEST_F(NavMapPlannerTest, CostWeightIsAParameterWithADefault)
{
  EXPECT_DOUBLE_EQ(node_->get_parameter("planner.cost_weight").as_double(), 5.0);
}

// A costly (not inscribed) band on the straight line: the path goes around it.
TEST_F(NavMapPlannerTest, InflationCostKeepsThePathAwayFromObstacles)
{
  set_robot(0.55, 2.0);
  inflate(costly_box);
  set_goal(3.45, 2.0);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());
  EXPECT_LT(length_inside(1.5, 2.5, 1.2, 2.8), 0.3);
}

TEST_F(NavMapPlannerTest, ZeroCostWeightIgnoresTheCost)
{
  node_->set_parameter(rclcpp::Parameter("planner.cost_weight", 0.0));
  planner_ = std::make_shared<easynav::navmap::AStarPlanner>();
  planner_->initialize(node_, "planner");
  set_robot(0.55, 2.0);
  inflate(costly_box);
  set_goal(3.45, 2.0);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());
  EXPECT_GT(length_inside(1.5, 2.5, 1.2, 2.8), 0.8) << "the cost should not matter";
}

// Higher costs push the path farther: the cost grows towards the obstacle at y = 2.
TEST_F(NavMapPlannerTest, PathPrefersLowerCostsInAGradient)
{
  set_robot(0.55, 0.55);
  inflate(cost_gradient);
  set_goal(3.45, 0.55);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());
  for (const auto & ps : path().poses) {
    EXPECT_LT(ps.pose.position.y, 1.3) << "towards the costly side at x " << ps.pose.position.x;
  }
}

TEST_F(NavMapPlannerTest, AnInscribedGoalIsNotPlanned)
{
  inscribe([](float x, float y) {return std::hypot(x - 3.05f, y - 3.05f) < 0.3f;});
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(NavMapPlannerTest, UnreachableGoalsGiveAnEmptyPath)
{
  // Walled off.
  wall_off(1.55, 3.05);
  set_goal(1.55, 3.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "walled off";

  // On an occupied cell.
  occupy_cell(to_cell(3.05), to_cell(1.05));
  update_map();
  set_goal(3.05, 1.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "occupied goal";

  // Behind a wall that splits the map.
  vertical_wall(2.55);
  set_goal(3.55, 0.55);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "split map";
}

TEST_F(NavMapPlannerTest, UnreachableGoalDoesNotKeepThePreviousPath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  wall_off(1.55, 3.05);
  set_goal(1.55, 3.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(NavMapPlannerTest, PlansAgainOnceTheGoalIsReachable)
{
  // reachable -> unreachable -> reachable
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  wall_off(1.55, 3.05);
  set_goal(1.55, 3.05);
  planner_->update(nav_state_);
  ASSERT_TRUE(path().poses.empty());

  set_goal(3.05, 0.55);
  planner_->update(nav_state_);
  EXPECT_FALSE(path().poses.empty());
}

TEST_F(NavMapPlannerTest, NoGoalClearsThePath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  nav_state_.set("goals", nav_msgs::msg::Goals());
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());

  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  EXPECT_FALSE(path().poses.empty());
}

TEST_F(NavMapPlannerTest, GoalInAnotherFrameClearsThePath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  set_goal(3.05, 3.05, "odom");
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());

  // Back to the map frame: plans again.
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  EXPECT_FALSE(path().poses.empty());
}

// ─── Path shape ─────────────────────────────────────────────────────────────────────────────

namespace
{

struct Shape
{
  double length {0.0};      // Path length
  double deviation {0.0};   // Farthest point from the straight line
};

Shape shape_of(const nav_msgs::msg::Path & p, double sx, double sy, double gx, double gy)
{
  Shape s;
  const double L = std::hypot(gx - sx, gy - sy);
  for (std::size_t i = 0; i < p.poses.size(); ++i) {
    const auto & b = p.poses[i].pose.position;
    s.deviation = std::max(
      s.deviation, std::abs((gx - sx) * (sy - b.y) - (sx - b.x) * (gy - sy)) / L);
    if (i > 0) {
      const auto & a = p.poses[i - 1].pose.position;
      s.length += std::hypot(b.x - a.x, b.y - a.y);
    }
  }
  return s;
}

}  // namespace

TEST_F(NavMapPlannerTest, InFreeSpaceThePathIsStraightInEveryDirection)
{
  // Before, with only edge neighbors, one diagonal was an L: 39 % longer, 1.66 m off the line
  struct Case {double sx, sy, gx, gy;};
  for (const auto & c : std::vector<Case>{
    {0.55, 0.55, 3.45, 0.55}, {3.45, 0.55, 0.55, 0.55}, {0.55, 0.55, 0.55, 3.45},
    {0.55, 3.45, 0.55, 0.55}, {0.55, 0.55, 3.45, 3.45}, {3.45, 3.45, 0.55, 0.55},
    {3.45, 0.55, 0.55, 3.45}, {0.55, 3.45, 3.45, 0.55}, {0.55, 0.55, 3.45, 1.55},
    {3.45, 0.55, 0.55, 1.55}, {0.55, 0.55, 1.55, 3.45}, {1.55, 0.55, 0.55, 3.45}})
  {
    set_robot(c.sx, c.sy);
    set_goal(c.gx, c.gy);
    planner_->update(nav_state_);
    const auto p = path();
    ASSERT_FALSE(p.poses.empty());
    const auto s = shape_of(p, c.sx, c.sy, c.gx, c.gy);
    const double straight = std::hypot(c.gx - c.sx, c.gy - c.sy);
    EXPECT_LT(s.length, straight + 0.1) <<
      "(" << c.sx << ", " << c.sy << ") -> (" << c.gx << ", " << c.gy << ")";
    EXPECT_LT(s.deviation, 0.1) <<
      "(" << c.sx << ", " << c.sy << ") -> (" << c.gx << ", " << c.gy << ")";
  }
}

TEST_F(NavMapPlannerTest, ThePathIsDenseAndEndsAtTheGoal)
{
  set_goal(3.45, 1.55);
  planner_->update(nav_state_);
  const auto p = path();
  ASSERT_GE(p.poses.size(), 2u);
  for (std::size_t i = 1; i < p.poses.size(); ++i) {
    const auto & a = p.poses[i - 1].pose.position;
    const auto & b = p.poses[i].pose.position;
    EXPECT_LT(std::hypot(b.x - a.x, b.y - a.y), 2 * kResolution) << i;
  }
  EXPECT_DOUBLE_EQ(p.poses.back().pose.position.x, 3.45);
  EXPECT_DOUBLE_EQ(p.poses.back().pose.position.y, 1.55);
}

TEST_F(NavMapPlannerTest, AroundAWallThePathOnlyBendsAtItsEnd)
{
  // Wall at x = 2 from y = 0 to 2.5: straight to near its end, then straight to the goal
  for (int y = 0; y < 25; ++y) {
    occupy_cell(20, y);
  }
  update_map();
  set_robot(0.55, 0.55);
  set_goal(3.45, 0.55);
  planner_->update(nav_state_);
  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  // Shortest around the end (2.0, 2.5): 2 * hypot(1.45, 1.95) = 4.86 m
  EXPECT_LT(shape_of(p, 0.55, 0.55, 3.45, 0.55).length, 4.86 * 1.05);
  for (const auto & ps : p.poses) {
    const auto & q = ps.pose.position;
    EXPECT_FALSE(q.x >= 2.0 && q.x < 2.1 && q.y < 2.5) << "through the wall at " << q.x <<
      ", " << q.y;
  }
}

TEST_F(NavMapPlannerTest, TheShortcutDoesNotCrossCostlierCells)
{
  // A costly (not inscribed) square between start and goal: the A* goes around it, and the
  // straight shortcut must not cut through it
  inflate([](float x, float y) -> std::uint8_t {
      return (x > 1.5f && x < 2.5f && y > 0.0f && y < 1.6f) ? 200 : 0;
    });
  set_robot(0.55, 0.55);
  set_goal(3.45, 0.55);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());
  EXPECT_NEAR(length_inside(1.55, 2.45, 0.0, 1.55), 0.0, 1e-9);
}

TEST_F(NavMapPlannerTest, CellsTouchingAtACornerAreAWall)
{
  // A diagonal line of occupied cells, each touching the next only at a corner, across the map:
  // no gap to slip through
  for (int i = 0; i < kCells; ++i) {
    occupy_cell(i, i);
  }
  update_map();
  set_robot(2.55, 0.55);
  set_goal(0.55, 2.55);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());

  // The same wall the other way (against the mesh's split diagonal)
  grid_.data.assign(kCells * kCells, 0);
  for (int i = 0; i < kCells; ++i) {
    occupy_cell(i, kCells - 1 - i);
  }
  update_map();
  set_robot(0.55, 0.55);
  set_goal(3.45, 3.45);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(NavMapPlannerTest, AChangedMapWithTheSameSizeIsNotPlannedOnTheOldGraph)
{
  set_goal(3.45, 0.55);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  // Same number of cells, shifted 10 m: the cached centroids must be rebuilt
  grid_.info.origin.position.x = 10.0;
  update_map();
  set_robot(10.55, 0.55);
  set_goal(13.45, 0.55);
  planner_->update(nav_state_);
  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  EXPECT_GT(p.poses.front().pose.position.x, 10.0);
}
