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
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_diagnostic_recovery/mitigations/RotateRecovery.hpp"
#include "easynav_core/VelocityCommand.hpp"

using easynav_diagnostic_recovery::RecoveryStatus;

class RotateRecoveryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }

  std::shared_ptr<easynav::RotateRecovery> make(
    const std::string & name, const std::vector<rclcpp::Parameter> & params = {})
  {
    static int count = 0;
    rclcpp::NodeOptions opts;
    for (const auto & p : params) {
      opts.append_parameter_override(name + "." + p.get_name(), p.get_parameter_value());
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "rotate_recovery_" + std::to_string(count++), opts);
    auto rec = std::make_shared<easynav::RotateRecovery>();
    rec->initialize(node_, name);
    return rec;
  }

  void yaw(double a)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.z = std::sin(a / 2.0);
    odom.pose.pose.orientation.w = std::cos(a / 2.0);
    nav_state_.set("robot_pose", odom);
  }

  double commanded_wz()
  {
    const auto cmd = easynav::velocity_command::peek(nav_state_, easynav::VelocitySource::TAKEOVER);
    EXPECT_TRUE(cmd.has_value());
    return cmd ? cmd->twist.angular.z : NAN;
  }

  // Turns a full turn in steps of 1.5 rad (crossing +-pi) and returns the status after it
  RecoveryStatus full_turn(easynav::RotateRecovery & rec)
  {
    RecoveryStatus st = RecoveryStatus::RUNNING;
    for (double a = 0.0; a <= 2.0 * M_PI + 1.5 && st == RecoveryStatus::RUNNING; a += 1.5) {
      yaw(std::remainder(a, 2.0 * M_PI));
      st = rec.internal_cycle(nav_state_);
      if (a + 1.5 <= 2.0 * M_PI) {EXPECT_EQ(st, RecoveryStatus::RUNNING) << a;}
    }
    return st;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

TEST_F(RotateRecoveryTest, HandlesNoPathErrorsAndTakesControl)
{
  auto rec = make("rotate");
  EXPECT_TRUE(rec->requires_control());
  diagnostic_msgs::msg::DiagnosticStatus s;
  s.hardware_id = "planner";
  s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(s));
  s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;  // no path published yet
  EXPECT_FALSE(rec->can_handle(s));
  s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  s.hardware_id = "controller_stuck";
  EXPECT_FALSE(rec->can_handle(s));
  auto custom = make(
    "rotate", {rclcpp::Parameter("hardware_ids", std::vector<std::string>{"controller_stuck"})});
  EXPECT_TRUE(custom->can_handle(s));
}

TEST_F(RotateRecoveryTest, TurnsAFullTurnThenSettlesAndSucceeds)
{
  auto rec = make("rotate", {rclcpp::Parameter("settle_time", 0.05)});
  yaw(0.0);
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.5);
  // Through +-pi: the turned angle accumulates
  EXPECT_EQ(full_turn(*rec), RecoveryStatus::RUNNING);  // turned: now standing still
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.0);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::SUCCEEDED);
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.0);
}

TEST_F(RotateRecoveryTest, ANegativeAngleTurnsClockwiseAndNoSettleSucceedsAtOnce)
{
  auto rec = make(
    "rotate", {rclcpp::Parameter("angle", -M_PI), rclcpp::Parameter("angular_speed", 0.8),
      rclcpp::Parameter("settle_time", 0.0)});
  yaw(0.0);
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  EXPECT_DOUBLE_EQ(commanded_wz(), -0.8);
  yaw(-2.0);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  yaw(2.9);  // -3.38 rad: past half a turn, across -pi
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::SUCCEEDED);
}

TEST_F(RotateRecoveryTest, OncePerEpisodeThenEscalates)
{
  auto rec = make("rotate", {rclcpp::Parameter("settle_time", 0.0)});
  yaw(0.0);
  rec->internal_start(nav_state_);
  EXPECT_EQ(full_turn(*rec), RecoveryStatus::SUCCEEDED);
  rec->internal_stop(nav_state_);
  // Still no path right after: the turn did not help, give way to the next mitigation
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::FAILED);
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.0);
  rec->internal_stop(nav_state_);
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::FAILED);
}

TEST_F(RotateRecoveryTest, ALongGapStartsANewEpisode)
{
  auto rec = make(
    "rotate", {rclcpp::Parameter("settle_time", 0.0), rclcpp::Parameter("episode_gap", 0.03)});
  yaw(0.0);
  rec->internal_start(nav_state_);
  EXPECT_EQ(full_turn(*rec), RecoveryStatus::SUCCEEDED);
  rec->internal_stop(nav_state_);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.5);
}

TEST_F(RotateRecoveryTest, FailsIfTheRobotDoesNotTurn)
{
  auto rec = make("rotate", {rclcpp::Parameter("timeout", 0.05)});
  yaw(1.0);
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::FAILED);  // blocked: never turned
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.0);
  rec->internal_stop(nav_state_);
  rec->internal_start(nav_state_);  // and it does not try again in this episode
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::FAILED);
}

TEST_F(RotateRecoveryTest, InvalidParametersFallBackToTheDefaults)
{
  auto rec = make(
    "rotate", {rclcpp::Parameter("angle", 0.0), rclcpp::Parameter("angular_speed", 0.0),
      rclcpp::Parameter("settle_time", 0.0)});
  yaw(0.0);
  rec->internal_start(nav_state_);
  EXPECT_EQ(rec->internal_cycle(nav_state_), RecoveryStatus::RUNNING);
  EXPECT_DOUBLE_EQ(commanded_wz(), 0.5);
  EXPECT_EQ(full_turn(*rec), RecoveryStatus::SUCCEEDED);  // a full turn
}
