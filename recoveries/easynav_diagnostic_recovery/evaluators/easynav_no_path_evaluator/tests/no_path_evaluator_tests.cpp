// Copyright 2026 Intelligent Robotics Lab
//
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

#include <chrono>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "easynav_no_path_evaluator/NoPathEvaluator.hpp"

class NoPathEvaluatorTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void set_active_goal(easynav::NavState & nav_state)
  {
    nav_msgs::msg::Goals goals;
    goals.goals.push_back(geometry_msgs::msg::PoseStamped());
    nav_state.set("goals", goals);
  }
};

TEST_F(NoPathEvaluatorTestCase, OkWithoutAnActiveGoal)
{
  // No "goals" key at all (e.g. before GoalManager's first non-RT cycle): an empty/missing
  // path is expected, not a failure.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_goals_key_node");
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path0");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.no_path0").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(NoPathEvaluatorTestCase, OkWithEmptyGoalsListEvenWithNoPath)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_empty_goals_node");
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path1");

  easynav::NavState nav_state;
  nav_state.set("goals", nav_msgs::msg::Goals());  // present but empty: no active goal

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.no_path1").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(NoPathEvaluatorTestCase, WarnsWhenNoPathYetWithAnActiveGoal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_path_node");
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path2");

  easynav::NavState nav_state;
  set_active_goal(nav_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);

  ASSERT_TRUE(nav_state.has("diagnostics.no_path2"));
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.no_path2").level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST_F(NoPathEvaluatorTestCase, ErrorsOnEmptyPathWithAnActiveGoal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_empty_path_node", rclcpp::NodeOptions().parameter_overrides(
      {{"no_path3.debounce_duration", 0.0}}));
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path3");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_state.set("path", nav_msgs::msg::Path());

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.no_path3").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(NoPathEvaluatorTestCase, OkWhenPathHasPosesWithAnActiveGoal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ok_path_node");
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path4");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_msgs::msg::Path path;
  path.poses.push_back(geometry_msgs::msg::PoseStamped());
  nav_state.set("path", path);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.no_path4").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

// Debounce: a new goal leaves the path empty until the next planner cycle.
class NoPathDebounceTest : public NoPathEvaluatorTestCase
{
protected:
  void make(double debounce)
  {
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_no_path_debounce_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(
        {{"no_path.freq", 1000.0}, {"no_path.debounce_duration", debounce}}));
    eval_.initialize(node_, "no_path");
    set_active_goal(nav_state_);
  }

  void set_path(bool with_poses)
  {
    nav_msgs::msg::Path path;
    if (with_poses) {
      path.poses.push_back(geometry_msgs::msg::PoseStamped());
    }
    nav_state_.set("path", path);
  }

  uint8_t level_after(int ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    eval_.internal_update(nav_state_);
    return nav_state_.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.no_path").level;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NoPathEvaluator eval_;
  easynav::NavState nav_state_;
};

using diagnostic_msgs::msg::DiagnosticStatus;

TEST_F(NoPathDebounceTest, DefaultsToTwoSeconds)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_path_default");
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path");
  EXPECT_DOUBLE_EQ(node->get_parameter("no_path.debounce_duration").as_double(), 2.0);
}

TEST_F(NoPathDebounceTest, AShortEmptyPathIsOnlyAWarning)
{
  make(0.5);
  set_path(false);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);
  EXPECT_EQ(level_after(100), DiagnosticStatus::WARN);
  set_path(true);
  EXPECT_EQ(level_after(5), DiagnosticStatus::OK);
}

TEST_F(NoPathDebounceTest, ASustainedEmptyPathIsAnError)
{
  make(0.2);
  set_path(false);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);
  EXPECT_EQ(level_after(250), DiagnosticStatus::ERROR);
  EXPECT_EQ(level_after(5), DiagnosticStatus::ERROR);   // and stays so
}

TEST_F(NoPathDebounceTest, APathInBetweenRestartsTheWait)
{
  make(0.3);
  set_path(false);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);
  set_path(true);
  EXPECT_EQ(level_after(200), DiagnosticStatus::OK);
  set_path(false);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);
  EXPECT_EQ(level_after(200), DiagnosticStatus::WARN);   // 0.2 s since it became empty again
  EXPECT_EQ(level_after(150), DiagnosticStatus::ERROR);
}

TEST_F(NoPathDebounceTest, LosingTheGoalRestartsTheWait)
{
  make(0.3);
  set_path(false);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);
  nav_state_.set("goals", nav_msgs::msg::Goals());
  EXPECT_EQ(level_after(200), DiagnosticStatus::OK);
  set_active_goal(nav_state_);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);    // the wait starts again here
  EXPECT_EQ(level_after(200), DiagnosticStatus::WARN);
  EXPECT_EQ(level_after(150), DiagnosticStatus::ERROR);
}

TEST_F(NoPathDebounceTest, AnErrorClearsOnceThereIsAPath)
{
  make(0.1);
  set_path(false);
  level_after(5);
  EXPECT_EQ(level_after(150), DiagnosticStatus::ERROR);
  set_path(true);
  EXPECT_EQ(level_after(5), DiagnosticStatus::OK);
  set_path(false);
  EXPECT_EQ(level_after(5), DiagnosticStatus::WARN);
}
