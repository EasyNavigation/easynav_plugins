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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_empty_path_node");
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
