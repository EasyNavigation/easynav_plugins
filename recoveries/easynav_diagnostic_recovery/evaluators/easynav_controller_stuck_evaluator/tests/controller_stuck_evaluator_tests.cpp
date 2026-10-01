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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_controller_stuck_evaluator/ControllerStuckEvaluator.hpp"

class ControllerStuckEvaluatorTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::ControllerStuckEvaluator> make_ready_evaluator(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto eval = std::make_shared<easynav::ControllerStuckEvaluator>();
    eval->initialize(node, name);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    return eval;
  }

  static void set_active_goal(easynav::NavState & nav_state)
  {
    nav_msgs::msg::Goals goals;
    goals.goals.push_back(geometry_msgs::msg::PoseStamped());
    nav_state.set("goals", goals);
  }

  static void set_commanded_motion(easynav::NavState & nav_state, double linear_x = 0.3)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = linear_x;
    nav_state.set("cmd_vel", cmd);
  }

  static void set_robot_position(easynav::NavState & nav_state, double x, double y)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    nav_state.set("robot_pose", odom);
  }
};

TEST_F(ControllerStuckEvaluatorTestCase, OkWithoutActiveGoal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_goal_node");
  auto eval = make_ready_evaluator(node, "stuck0");

  easynav::NavState nav_state;
  set_commanded_motion(nav_state);
  set_robot_position(nav_state, 0.0, 0.0);
  eval->internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck0").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerStuckEvaluatorTestCase, OkWhenControlOwnerIsNotController)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_owner_node");
  auto eval = make_ready_evaluator(node, "stuck1");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state);
  set_robot_position(nav_state, 0.0, 0.0);
  nav_state.set("control_owner", std::string("recovery:retreat"));
  eval->internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck1").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerStuckEvaluatorTestCase, OkWhenSafetyReflexIsIntervening)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_node");
  auto eval = make_ready_evaluator(node, "stuck2");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state);
  set_robot_position(nav_state, 0.0, 0.0);

  diagnostic_msgs::msg::DiagnosticStatus reflex_status;
  reflex_status.hardware_id = "safety_reflex";
  reflex_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  nav_state.set("diagnostics.collision", reflex_status);
  nav_state.set_group("diagnostics", {"diagnostics.collision"});

  eval->internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck2").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerStuckEvaluatorTestCase, OkWhenNotCommandedToMove)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_cmd_node");
  auto eval = make_ready_evaluator(node, "stuck3");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state, 0.0);
  set_robot_position(nav_state, 0.0, 0.0);
  eval->internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck3").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerStuckEvaluatorTestCase, OkWhileMakingProgress)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_progress_node",
    rclcpp::NodeOptions().append_parameter_override("stuck4.stuck_time_threshold", 0.05));
  auto eval = make_ready_evaluator(node, "stuck4");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state);

  for (double x = 0.0; x < 0.5; x += 0.2) {
    set_robot_position(nav_state, x, 0.0);
    eval->internal_update(nav_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck4").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerStuckEvaluatorTestCase, OkWhileNavigationPaused)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_paused_node",
    rclcpp::NodeOptions().append_parameter_override("stuck6.stuck_time_threshold", 0.05));
  auto eval = make_ready_evaluator(node, "stuck6");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state);
  set_robot_position(nav_state, 1.0, 1.0);
  nav_state.set("navigation_paused", true);

  eval->internal_update(nav_state);  // reference position established, still OK
  ASSERT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck6").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  // Long enough to trip stuck_time_threshold_ if the reference were frozen instead of re-armed.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  eval->internal_update(nav_state);  // still paused, same position: must stay OK

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck6");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "navigation paused");
}

TEST_F(ControllerStuckEvaluatorTestCase, OkImmediatelyAfterResumingFromPause)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_resume_node",
    rclcpp::NodeOptions().append_parameter_override("stuck7.stuck_time_threshold", 0.05));
  auto eval = make_ready_evaluator(node, "stuck7");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state);
  set_robot_position(nav_state, 1.0, 1.0);
  nav_state.set("navigation_paused", true);

  eval->internal_update(nav_state);  // reference established while paused

  // Elapse (while still paused) past what would be stuck_time_threshold_ if the reference had
  // been frozen instead of re-armed each cycle.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  eval->internal_update(nav_state);

  // Resume: same position (robot has not moved yet), but the pause re-armed the reference on
  // the last paused cycle, so this must not immediately report stuck.
  nav_state.set("navigation_paused", false);
  eval->internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck7").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerStuckEvaluatorTestCase, ErrorAfterNotMovingLongEnough)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_error_node",
    rclcpp::NodeOptions()
    .append_parameter_override("stuck5.stuck_time_threshold", 0.05)
    .append_parameter_override("stuck5.freq", 200.0));
  auto eval = make_ready_evaluator(node, "stuck5");

  easynav::NavState nav_state;
  set_active_goal(nav_state);
  set_commanded_motion(nav_state);
  set_robot_position(nav_state, 1.0, 1.0);

  eval->internal_update(nav_state);  // establishes the reference position, still OK
  ASSERT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck5").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));  // past the 50 ms debounce
  eval->internal_update(nav_state);  // same position: stuck

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.stuck5");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.hardware_id, "controller_stuck");
  ASSERT_EQ(status.values.size(), 1u);
  EXPECT_EQ(status.values[0].key, "stuck_duration");
}
