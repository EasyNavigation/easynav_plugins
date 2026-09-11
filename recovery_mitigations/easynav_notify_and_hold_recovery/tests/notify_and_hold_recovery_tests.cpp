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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "easynav_notify_and_hold_recovery/NotifyAndHoldRecovery.hpp"

class NotifyAndHoldRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::NotifyAndHoldRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::NotifyAndHoldRecovery>();
    rec->initialize(node, name);
    return rec;
  }
};

TEST_F(NotifyAndHoldRecoveryTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "hold0");
  EXPECT_TRUE(rec->requires_control());
}

TEST_F(NotifyAndHoldRecoveryTestCase, CanHandleAnyErrorOrAboveDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "hold1");

  diagnostic_msgs::msg::DiagnosticStatus error_status;
  error_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(error_status));

  diagnostic_msgs::msg::DiagnosticStatus warn_status;
  warn_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  EXPECT_FALSE(rec->can_handle(warn_status));
}

TEST_F(NotifyAndHoldRecoveryTestCase, OnStartDefaultsToReportingFailed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_default_node");
  auto rec = make_recovery(node, "hold2");

  easynav::NavState nav_state;
  rec->internal_start(nav_state);

  ASSERT_TRUE(nav_state.has("goal_manager_request"));
  EXPECT_EQ(nav_state.get<std::string>("goal_manager_request"), "failed");
  ASSERT_TRUE(nav_state.has("goal_manager_reason"));
  EXPECT_FALSE(nav_state.get<std::string>("goal_manager_reason").empty());
}

TEST_F(NotifyAndHoldRecoveryTestCase, ReportAsErrorParameterIsHonored)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_error_node",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("hold3.report_as", "error")}));
  auto rec = make_recovery(node, "hold3");

  easynav::NavState nav_state;
  rec->internal_start(nav_state);

  EXPECT_EQ(nav_state.get<std::string>("goal_manager_request"), "error");
}

TEST_F(NotifyAndHoldRecoveryTestCase, InvalidReportAsFallsBackToFailed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_invalid_node",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("hold4.report_as", "bogus")}));
  auto rec = make_recovery(node, "hold4");

  easynav::NavState nav_state;
  rec->internal_start(nav_state);

  EXPECT_EQ(nav_state.get<std::string>("goal_manager_request"), "failed");
}

TEST_F(NotifyAndHoldRecoveryTestCase, HoldsAndStopsWhileNoActiveGoal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_hold_node");
  auto rec = make_recovery(node, "hold5");

  easynav::NavState nav_state;  // "goals" absent -> treated as no active goal

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}

TEST_F(NotifyAndHoldRecoveryTestCase, ReleasesControlOnceANewGoalIsActive)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_release_node");
  auto rec = make_recovery(node, "hold6");

  easynav::NavState nav_state;
  nav_msgs::msg::Goals goals;
  goals.goals.push_back(geometry_msgs::msg::PoseStamped());
  nav_state.set("goals", goals);

  EXPECT_EQ(rec->internal_cycle(nav_state), easynav::RecoveryStatus::SUCCEEDED);
}
