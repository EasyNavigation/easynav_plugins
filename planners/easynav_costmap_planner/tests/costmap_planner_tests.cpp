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
/// \brief Tests for CostmapPlanner: a failed plan leaves an empty path, not the previous one.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_planner/CostmapPlanner.hpp"

class TestableCostmapPlanner : public easynav::CostmapPlanner
{
public:
  using CostmapPlanner::a_star_path;
};

class CostmapPlannerTest : public ::testing::Test
{
protected:
  static constexpr unsigned int kCells = 40;
  static constexpr double kResolution = 0.1;

  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // One node per test: initialize() declares parameters.
    static int test_count = 0;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "costmap_planner_test_" + std::to_string(test_count++));
    planner_ = std::make_shared<TestableCostmapPlanner>();
    planner_->initialize(node_, "planner");

    // Free 4 x 4 m map, robot near a corner.
    map_ = easynav::Costmap2D(kCells, kCells, kResolution, 0.0, 0.0, easynav::FREE_SPACE);
    nav_state_.set("map", map_);
    nav_state_.set("map_time", node_->now());

    nav_msgs::msg::Odometry robot;
    robot.header.frame_id = "map";
    robot.pose.pose.position.x = 0.55;
    robot.pose.pose.position.y = 0.55;
    robot.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", robot);
  }

  void set_goal(double x, double y)
  {
    nav_msgs::msg::Goals goals;
    goals.header.frame_id = "map";
    goals.header.stamp = node_->now();
    geometry_msgs::msg::PoseStamped goal;
    goal.header = goals.header;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    goals.goals.push_back(goal);
    nav_state_.set("goals", goals);
  }

  // Lethal ring around (x, y).
  void wall_off(double x, double y)
  {
    unsigned int cx, cy;
    ASSERT_TRUE(map_.worldToMap(x, y, cx, cy));
    for (int dx = -2; dx <= 2; ++dx) {
      for (int dy = -2; dy <= 2; ++dy) {
        if (std::abs(dx) == 2 || std::abs(dy) == 2) {
          map_.setCost(cx + dx, cy + dy, easynav::LETHAL_OBSTACLE);
        }
      }
    }
    nav_state_.set("map", map_);
  }

  // Vertical lethal wall at x, across the whole map, optionally leaving a gap at gap_y.
  void vertical_wall(double x, double gap_y = -1.0)
  {
    unsigned int cx, cy, gx = 0, gy = 0;
    ASSERT_TRUE(map_.worldToMap(x, 0.05, cx, cy));
    const bool gap = gap_y >= 0.0 && map_.worldToMap(x, gap_y, gx, gy);
    for (unsigned int y = 0; y < kCells; ++y) {
      if (!gap || y + 1 < gy || y > gy + 1) {
        map_.setCost(cx, y, easynav::LETHAL_OBSTACLE);
      }
    }
    nav_state_.set("map", map_);
  }

  void set_cell(double x, double y, unsigned char cost)
  {
    unsigned int cx, cy;
    ASSERT_TRUE(map_.worldToMap(x, y, cx, cy));
    map_.setCost(cx, cy, cost);
    nav_state_.set("map", map_);
  }

  // Every point along the path's segments, not just its poses, is out of collision.
  void expect_path_clear(const nav_msgs::msg::Path & p) const
  {
    for (std::size_t i = 1; i < p.poses.size(); ++i) {
      const auto & a = p.poses[i - 1].pose.position;
      const auto & b = p.poses[i].pose.position;
      const int n = std::max(1, static_cast<int>(std::hypot(b.x - a.x, b.y - a.y) / 0.005));
      for (int k = 0; k <= n; ++k) {
        const double t = static_cast<double>(k) / n;
        const double x = a.x + t * (b.x - a.x);
        const double y = a.y + t * (b.y - a.y);
        unsigned int cx, cy;
        ASSERT_TRUE(map_.worldToMap(x, y, cx, cy));
        EXPECT_LT(map_.getCost(cx, cy), easynav::INSCRIBED_INFLATED_OBSTACLE) <<
          "segment " << i << " (" << a.x << ", " << a.y << ") -> (" << b.x << ", " << b.y <<
          ") crosses (" << x << ", " << y << ")";
      }
    }
  }

  // Lethal rectangle [x0, x1] x [y0, y1] (m).
  void block(double x0, double y0, double x1, double y1)
  {
    unsigned int cx0, cy0, cx1, cy1;
    ASSERT_TRUE(map_.worldToMap(x0, y0, cx0, cy0));
    ASSERT_TRUE(map_.worldToMap(x1, y1, cx1, cy1));
    for (unsigned int x = cx0; x <= cx1; ++x) {
      for (unsigned int y = cy0; y <= cy1; ++y) {
        map_.setCost(x, y, easynav::LETHAL_OBSTACLE);
      }
    }
    nav_state_.set("map", map_);
  }

  nav_msgs::msg::Path path() const
  {
    return nav_state_.get<nav_msgs::msg::Path>("path");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<TestableCostmapPlanner> planner_;
  easynav::Costmap2D map_;
  easynav::NavState nav_state_;
};

TEST_F(CostmapPlannerTest, NothingPlannedWithoutInputs)
{
  easynav::NavState empty;
  planner_->update(empty);
  EXPECT_FALSE(empty.has("path"));
}

TEST_F(CostmapPlannerTest, ReachableGoalProducesAPathEndingAtTheGoal)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);

  ASSERT_TRUE(nav_state_.has("path"));
  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_NEAR(p.poses.back().pose.position.x, 3.05, kResolution);
  EXPECT_NEAR(p.poses.back().pose.position.y, 3.05, kResolution);
}

TEST_F(CostmapPlannerTest, GoalOnTheRobotCellGivesASinglePose)
{
  set_goal(0.55, 0.55);
  planner_->update(nav_state_);

  ASSERT_EQ(path().poses.size(), 1u);
}

TEST_F(CostmapPlannerTest, PathGoesThroughTheGapOfAWall)
{
  vertical_wall(2.05, 3.55);
  set_goal(3.05, 0.55);
  planner_->update(nav_state_);

  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  expect_path_clear(p);
}

TEST_F(CostmapPlannerTest, PathAroundACornerNeverCutsIt)
{
  // Block between robot and goal: the path turns around its corners.
  block(1.2, 0.0, 2.4, 2.6);
  set_goal(3.05, 0.55);
  planner_->update(nav_state_);

  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  expect_path_clear(p);
}

TEST_F(CostmapPlannerTest, PathThroughANarrowPassageStaysInside)
{
  // Three-cell gap in a wall: smoothing must not pull the path into its sides.
  vertical_wall(2.05, 2.05);
  set_goal(3.55, 0.55);
  planner_->update(nav_state_);

  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  expect_path_clear(p);
}

TEST_F(CostmapPlannerTest, LowInflationCostDoesNotOutweighASingleCellDetour)
{
  set_cell(2.05, 0.55, 1);

  geometry_msgs::msg::Pose start;
  start.position.x = 0.55;
  start.position.y = 0.55;
  geometry_msgs::msg::Pose goal;
  goal.position.x = 3.45;
  goal.position.y = 0.55;
  goal.orientation.w = 1.0;

  const auto poses = planner_->a_star_path(map_, start, goal);
  ASSERT_FALSE(poses.empty());

  unsigned int low_cost_x, low_cost_y;
  ASSERT_TRUE(map_.worldToMap(2.05, 0.55, low_cost_x, low_cost_y));
  const bool traverses_low_cost_cell = std::any_of(
    poses.begin(), poses.end(), [&](const geometry_msgs::msg::Pose & pose) {
      unsigned int x, y;
      return map_.worldToMap(pose.position.x, pose.position.y, x, y) &&
             x == low_cost_x && y == low_cost_y;
    });
  EXPECT_TRUE(traverses_low_cost_cell);
}

TEST_F(CostmapPlannerTest, UnreachableGoalsGiveAnEmptyPath)
{
  // Walled off.
  wall_off(1.55, 3.05);
  set_goal(1.55, 3.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "walled off";

  // On a lethal cell.
  set_cell(3.05, 1.05, easynav::LETHAL_OBSTACLE);
  set_goal(3.05, 1.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "lethal goal";

  // On an inscribed (inflated) cell.
  set_cell(3.05, 2.05, easynav::INSCRIBED_INFLATED_OBSTACLE);
  set_goal(3.05, 2.05);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "inflated goal";

  // Behind a wall that splits the map.
  vertical_wall(2.55);
  set_goal(3.55, 0.55);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "split map";
}

TEST_F(CostmapPlannerTest, UnreachableGoalDoesNotKeepThePreviousPath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  wall_off(1.55, 3.05);
  set_goal(1.55, 3.05);
  planner_->update(nav_state_);

  EXPECT_TRUE(path().poses.empty()) << "a path to the previous goal was kept";
}

TEST_F(CostmapPlannerTest, UnreachableStaysEmptyAcrossCycles)
{
  wall_off(1.55, 3.05);
  set_goal(1.55, 3.05);
  for (int i = 0; i < 5; ++i) {
    planner_->update(nav_state_);
    EXPECT_TRUE(path().poses.empty()) << "cycle " << i;
  }
}

TEST_F(CostmapPlannerTest, PlansAgainOnceTheGoalIsReachable)
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
  const auto p = path();
  ASSERT_FALSE(p.poses.empty());
  EXPECT_NEAR(p.poses.back().pose.position.x, 3.05, kResolution);
  EXPECT_NEAR(p.poses.back().pose.position.y, 0.55, kResolution);
}

TEST_F(CostmapPlannerTest, NoGoalClearsThePath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  nav_state_.set("goals", nav_msgs::msg::Goals());
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());

  // A new goal after the mission ended plans normally.
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  EXPECT_FALSE(path().poses.empty());
}

TEST_F(CostmapPlannerTest, GoalOutsideTheMapClearsThePath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  set_goal(10.0, 10.0);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "beyond the map";

  set_goal(-1.0, 1.0);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "negative coordinates";
}

TEST_F(CostmapPlannerTest, GoalInAnotherFrameClearsThePath)
{
  set_goal(3.05, 3.05);
  planner_->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());

  auto goals = nav_state_.get<nav_msgs::msg::Goals>("goals");
  goals.header.frame_id = "odom";
  nav_state_.set("goals", goals);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}
