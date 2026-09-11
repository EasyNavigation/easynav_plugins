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

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_human_assistance_recovery/HumanAssistanceRecovery.hpp"

class HumanAssistanceRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::HumanAssistanceRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::HumanAssistanceRecovery>();
    rec->initialize(node, name);
    return rec;
  }

  static diagnostic_msgs::msg::DiagnosticStatus make_status(
    uint8_t level, const std::string & hardware_id = "any_component")
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.hardware_id = hardware_id;
    status.message = "test";
    return status;
  }
};

TEST_F(HumanAssistanceRecoveryTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "human0");
  EXPECT_TRUE(rec->requires_control());
}

TEST_F(HumanAssistanceRecoveryTestCase, CanHandleAnyHardwareIdAtErrorLevelOrAbove)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "human1");

  EXPECT_TRUE(
    rec->can_handle(make_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "planner")));
  EXPECT_TRUE(
    rec->can_handle(make_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "localizer.amcl")));
  EXPECT_TRUE(
    rec->can_handle(make_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "whatever")));
}

TEST_F(HumanAssistanceRecoveryTestCase, DoesNotHandleWarnOrOk)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_ch_node");
  auto rec = make_recovery(node, "human2");

  EXPECT_FALSE(rec->can_handle(make_status(diagnostic_msgs::msg::DiagnosticStatus::WARN)));
  EXPECT_FALSE(rec->can_handle(make_status(diagnostic_msgs::msg::DiagnosticStatus::OK)));
}

TEST_F(HumanAssistanceRecoveryTestCase, RunsWhileAnyDiagnosticIsStillError)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_running_node");
  auto rec = make_recovery(node, "human3");

  easynav::NavState nav_state;
  nav_state.set("diagnostics.planner", make_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
  nav_state.set_group("diagnostics", {"diagnostics.planner"});

  rec->internal_start(nav_state);
  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}

TEST_F(HumanAssistanceRecoveryTestCase, SucceedsOnceEveryDiagnosticIsOkAgain)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_succeed_node");
  auto rec = make_recovery(node, "human4");

  easynav::NavState nav_state;
  nav_state.set("diagnostics.planner", make_status(diagnostic_msgs::msg::DiagnosticStatus::OK));
  nav_state.set_group("diagnostics", {"diagnostics.planner"});

  rec->internal_start(nav_state);
  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::SUCCEEDED);
}

TEST_F(HumanAssistanceRecoveryTestCase, SucceedsWithNoDiagnosticsGroupAtAll)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_group_node");
  auto rec = make_recovery(node, "human5");

  easynav::NavState nav_state;  // no "diagnostics" group at all

  rec->internal_start(nav_state);
  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::SUCCEEDED);
}
