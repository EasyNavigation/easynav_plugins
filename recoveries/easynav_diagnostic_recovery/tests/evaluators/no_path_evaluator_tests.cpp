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

#include "easynav_diagnostic_recovery/evaluators/NoPathEvaluator.hpp"

class NoPathEvaluatorTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static uint8_t level(const easynav::NavState & nav_state, const std::string & name)
  {
    return nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics." + name).level;
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

TEST_F(NoPathEvaluatorTestCase, AnEmptyPathIsAnErrorOnlyAfterTheDebounce)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_empty_path_node", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("no_path3.debounce_duration", 0.5)}));
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path3");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_state.set("path", nav_msgs::msg::Path());

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path3"), diagnostic_msgs::msg::DiagnosticStatus::WARN);

  // Still within the 0.5 s
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path3"), diagnostic_msgs::msg::DiagnosticStatus::WARN);

  // Past it, and while it lasts
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path3"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path3"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(NoPathEvaluatorTestCase, ByDefaultAnEmptyPathMustLastOneSecond)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_default_debounce_node");
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path5");
  EXPECT_DOUBLE_EQ(node->get_parameter("no_path5.debounce_duration").as_double(), 1.0);

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_state.set("path", nav_msgs::msg::Path());

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path5"), diagnostic_msgs::msg::DiagnosticStatus::WARN);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path5"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(NoPathEvaluatorTestCase, ZeroDebounceIsAnErrorAtOnce)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_zero_debounce_node", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("no_path6.debounce_duration", 0.0)}));
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path6");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_state.set("path", nav_msgs::msg::Path());

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path6"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(NoPathEvaluatorTestCase, APathInBetweenRestartsTheDebounce)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_restart_debounce_node", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("no_path7.debounce_duration", 0.4)}));
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path7");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_msgs::msg::Path path;
  path.poses.push_back(geometry_msgs::msg::PoseStamped());

  // Empty, a path again (one cycle), empty: never 0.4 s in a row
  for (int i = 0; i < 4; ++i) {
    nav_state.set("path", nav_msgs::msg::Path());
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    eval.internal_update(nav_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    eval.internal_update(nav_state);
    EXPECT_EQ(level(nav_state, "no_path7"), diagnostic_msgs::msg::DiagnosticStatus::WARN);

    nav_state.set("path", path);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    eval.internal_update(nav_state);
    EXPECT_EQ(level(nav_state, "no_path7"), diagnostic_msgs::msg::DiagnosticStatus::OK);
  }
}

TEST_F(NoPathEvaluatorTestCase, ANewGoalRestartsTheDebounce)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_goal_debounce_node", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("no_path8.debounce_duration", 0.4)}));
  easynav::NoPathEvaluator eval;
  eval.initialize(node, "no_path8");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  nav_state.set("path", nav_msgs::msg::Path());
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path8"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  // The goal is gone (finished or cancelled), then a new one, still without a path
  nav_state.set("goals", nav_msgs::msg::Goals());
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path8"), diagnostic_msgs::msg::DiagnosticStatus::OK);

  set_active_goal(nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
  EXPECT_EQ(level(nav_state, "no_path8"), diagnostic_msgs::msg::DiagnosticStatus::WARN);
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
