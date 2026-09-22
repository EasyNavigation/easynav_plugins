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

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_advance_recovery/AdvanceRecovery.hpp"

class AdvanceRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }

  std::shared_ptr<easynav::AdvanceRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::AdvanceRecovery>();
    rec->initialize(node, name);
    return rec;
  }

  static void set_position(easynav::NavState & nav_state, double x, double y)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    nav_state.set("robot_pose", odom);
  }
};

TEST_F(AdvanceRecoveryTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "advance0");
  EXPECT_TRUE(rec->requires_control());
}

TEST_F(AdvanceRecoveryTestCase, CanHandleOnlyControllerStuckErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "advance1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "controller_stuck";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus wrong_hardware = matching;
  wrong_hardware.hardware_id = "planner";
  EXPECT_FALSE(rec->can_handle(wrong_hardware));

  diagnostic_msgs::msg::DiagnosticStatus not_an_error = matching;
  not_an_error.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  EXPECT_FALSE(rec->can_handle(not_an_error));
}

TEST_F(AdvanceRecoveryTestCase, AdvancesForwardWhileNotYetAtDistance)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_advance_node",
    rclcpp::NodeOptions().append_parameter_override("advance2.advance_speed", 0.4));
  auto rec = make_recovery(node, "advance2");

  easynav::NavState nav_state;
  set_position(nav_state, 0.0, 0.0);

  rec->internal_start(nav_state);
  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.4);
}

TEST_F(AdvanceRecoveryTestCase, SucceedsOnceDistanceReachedButDoesNotClaimFixed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_succeed_node",
    rclcpp::NodeOptions().append_parameter_override("advance3.advance_distance", 0.2));
  auto rec = make_recovery(node, "advance3");

  easynav::NavState nav_state;
  set_position(nav_state, 0.0, 0.0);
  rec->internal_start(nav_state);

  set_position(nav_state, 0.25, 0.0);  // past advance_distance
  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::SUCCEEDED);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

TEST_F(AdvanceRecoveryTestCase, EscalatesAfterTotalEpisodeTimeExceeded)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_escalate_node",
    rclcpp::NodeOptions().append_parameter_override("advance4.escalate_after", 0.05));
  auto rec = make_recovery(node, "advance4");

  easynav::NavState nav_state;
  set_position(nav_state, 0.0, 0.0);
  rec->internal_start(nav_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));  // past escalate_after
  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::FAILED);
}

TEST_F(AdvanceRecoveryTestCase, AccumulatesTotalTimeAcrossQuickReactivations)
{
  // episode_gap large: the short gap between the two activations below must NOT be treated as
  // a new episode, so the second activation's escalation check sees the *total* elapsed time
  // since the very first activation.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_accumulate_node",
    rclcpp::NodeOptions()
    .append_parameter_override("advance5.escalate_after", 0.05)
    .append_parameter_override("advance5.episode_gap", 1.0)
    .append_parameter_override("advance5.advance_distance", 100.0));  // never "reached" here
  auto rec = make_recovery(node, "advance5");

  easynav::NavState nav_state;
  set_position(nav_state, 0.0, 0.0);

  rec->internal_start(nav_state);
  auto first_status = rec->internal_cycle(nav_state);
  ASSERT_EQ(first_status, easynav::RecoveryStatus::RUNNING);  // too soon to escalate yet
  rec->internal_stop(nav_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));  // << episode_gap, same episode

  rec->internal_start(nav_state);
  auto second_status = rec->internal_cycle(nav_state);

  EXPECT_EQ(second_status, easynav::RecoveryStatus::FAILED);
}

TEST_F(AdvanceRecoveryTestCase, LongGapBetweenActivationsStartsAFreshEpisode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_gap_reset_node",
    rclcpp::NodeOptions()
    .append_parameter_override("advance6.escalate_after", 0.05)
    .append_parameter_override("advance6.episode_gap", 0.03)
    .append_parameter_override("advance6.advance_distance", 100.0));
  auto rec = make_recovery(node, "advance6");

  easynav::NavState nav_state;
  set_position(nav_state, 0.0, 0.0);

  rec->internal_start(nav_state);
  rec->internal_cycle(nav_state);
  rec->internal_stop(nav_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));  // >> episode_gap: new episode

  rec->internal_start(nav_state);
  auto status = rec->internal_cycle(nav_state);

  // If the gap had NOT reset the episode clock, elapsed time since the very first activation
  // would already exceed escalate_after and this would incorrectly be FAILED.
  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
}
