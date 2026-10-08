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
/// \brief Tests for SimplePlanner: a failed plan leaves an empty path, not the previous one.

#include <cmath>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_simple_planner/SimplePlanner.hpp"

class SimplePlannerTest : public ::testing::Test
{
protected:
  static constexpr int kCells = 80;
  static constexpr double kResolution = 0.1;

  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // One node per test: initialize() declares parameters.
    static int test_count = 0;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "simple_planner_test_" + std::to_string(test_count++));
    planner_ = std::make_shared<easynav::SimplePlanner>();
    planner_->initialize(node_, "planner");

    // Free 8 x 8 m map, robot near a corner.
    map_.initialize(kCells, kCells, kResolution, 0.0, 0.0, false);
    nav_state_.set("map", map_);

    nav_msgs::msg::Odometry robot;
    robot.header.frame_id = "map";
    robot.pose.pose.position.x = 1.0;
    robot.pose.pose.position.y = 1.0;
    robot.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", robot);
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

  // Occupied square ring of the given half side (m) around (x, y).
  void wall_off(double x, double y, double half_side)
  {
    const auto [cx, cy] = map_.metric_to_cell(x, y);
    const int r = static_cast<int>(std::round(half_side / kResolution));
    for (int d = -r; d <= r; ++d) {
      map_.at(cx + d, cy - r) = 1;
      map_.at(cx + d, cy + r) = 1;
      map_.at(cx - r, cy + d) = 1;
      map_.at(cx + r, cy + d) = 1;
    }
    nav_state_.set("map", map_);
  }

  // Vertical wall at x across the map, leaving [gap_from, gap_to] free if gap_to > gap_from.
  void vertical_wall(double x, double gap_from = 0.0, double gap_to = 0.0)
  {
    const auto [cx, unused] = map_.metric_to_cell(x, 0.0);
    (void)unused;
    for (int y = 0; y < kCells; ++y) {
      const double wy = (y + 0.5) * kResolution;
      if (gap_to <= gap_from || wy < gap_from || wy > gap_to) {
        map_.at(cx, y) = 1;
      }
    }
    nav_state_.set("map", map_);
  }

  void occupy(double x, double y)
  {
    const auto [cx, cy] = map_.metric_to_cell(x, y);
    map_.at(cx, cy) = 1;
    nav_state_.set("map", map_);
  }

  nav_msgs::msg::Path path() const
  {
    return nav_state_.get<nav_msgs::msg::Path>("path");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::SimplePlanner> planner_;
  easynav::SimpleMap map_;
  easynav::NavState nav_state_;
};

TEST_F(SimplePlannerTest, ReachableGoalProducesAPathEndingAtTheGoal)
{
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);

  const auto p = path();
  ASSERT_GT(p.poses.size(), 1u);
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_NEAR(p.poses.back().pose.position.x, 6.0, 0.2);
  EXPECT_NEAR(p.poses.back().pose.position.y, 6.0, 0.2);
}

TEST_F(SimplePlannerTest, GoalOnTheRobotCellGivesASinglePose)
{
  set_goal(1.05, 1.05);
  planner_->update(nav_state_);

  ASSERT_EQ(path().poses.size(), 1u);
}

TEST_F(SimplePlannerTest, PathGoesThroughTheGapOfAWall)
{
  vertical_wall(4.0, 5.0, 7.0);
  set_goal(6.0, 1.0);
  planner_->update(nav_state_);

  const auto p = path();
  ASSERT_GT(p.poses.size(), 1u);
  for (const auto & pose : p.poses) {
    const auto [cx, cy] = map_.metric_to_cell(pose.pose.position.x, pose.pose.position.y);
    EXPECT_EQ(map_.at(cx, cy), 0) << "(" << pose.pose.position.x << ", " <<
      pose.pose.position.y << ")";
  }
}

TEST_F(SimplePlannerTest, UnreachableGoalsGiveAnEmptyPath)
{
  // Walled off.
  wall_off(6.0, 2.0, 1.0);
  set_goal(6.0, 2.0);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "walled off";

  // On an occupied cell.
  occupy(2.0, 6.0);
  set_goal(2.0, 6.0);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "occupied goal";

  // Behind a wall that splits the map.
  vertical_wall(3.0);
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "split map";
}

TEST_F(SimplePlannerTest, UnreachableGoalDoesNotKeepThePreviousPath)
{
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  ASSERT_GT(path().poses.size(), 1u);

  wall_off(6.0, 2.0, 1.0);
  set_goal(6.0, 2.0);
  planner_->update(nav_state_);

  EXPECT_TRUE(path().poses.empty()) << "a path to the previous goal was kept";
}

TEST_F(SimplePlannerTest, UnreachableStaysEmptyAcrossCycles)
{
  wall_off(6.0, 2.0, 1.0);
  set_goal(6.0, 2.0);
  for (int i = 0; i < 5; ++i) {
    planner_->update(nav_state_);
    EXPECT_TRUE(path().poses.empty()) << "cycle " << i;
  }
}

TEST_F(SimplePlannerTest, PlansAgainOnceTheGoalIsReachable)
{
  // reachable -> unreachable -> reachable
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  ASSERT_GT(path().poses.size(), 1u);

  wall_off(6.0, 2.0, 1.0);
  set_goal(6.0, 2.0);
  planner_->update(nav_state_);
  ASSERT_TRUE(path().poses.empty());

  set_goal(2.0, 6.0);
  planner_->update(nav_state_);
  const auto p = path();
  ASSERT_GT(p.poses.size(), 1u);
  EXPECT_NEAR(p.poses.back().pose.position.x, 2.0, 0.2);
  EXPECT_NEAR(p.poses.back().pose.position.y, 6.0, 0.2);
}

TEST_F(SimplePlannerTest, NoGoalClearsThePath)
{
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  ASSERT_GT(path().poses.size(), 1u);

  nav_state_.set("goals", nav_msgs::msg::Goals());
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());

  // A new goal after the mission ended plans normally.
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  EXPECT_GT(path().poses.size(), 1u);
}

TEST_F(SimplePlannerTest, GoalOutsideTheMapClearsThePath)
{
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  ASSERT_GT(path().poses.size(), 1u);

  set_goal(20.0, 20.0);
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(SimplePlannerTest, GoalInAnotherFrameClearsThePath)
{
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  ASSERT_GT(path().poses.size(), 1u);

  set_goal(6.0, 6.0, "odom");
  planner_->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(SimplePlannerTest, MissingMapClearsThePath)
{
  set_goal(6.0, 6.0);
  planner_->update(nav_state_);
  ASSERT_GT(path().poses.size(), 1u);

  // Same goal, but no map in a fresh NavState: nothing to plan on.
  easynav::NavState no_map;
  no_map.set("robot_pose", nav_state_.get<nav_msgs::msg::Odometry>("robot_pose"));
  no_map.set("goals", nav_state_.get<nav_msgs::msg::Goals>("goals"));
  no_map.set("path", path());
  planner_->update(no_map);
  EXPECT_TRUE(no_map.get<nav_msgs::msg::Path>("path").poses.empty());
}
