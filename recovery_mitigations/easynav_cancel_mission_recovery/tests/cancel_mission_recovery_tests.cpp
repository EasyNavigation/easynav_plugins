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

#include "easynav_cancel_mission_recovery/CancelMissionRecovery.hpp"

class CancelMissionRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::CancelMissionRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::CancelMissionRecovery>();
    rec->initialize(node, name);
    return rec;
  }
};

TEST_F(CancelMissionRecoveryTestCase, DoesNotRequireControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "cancel0");
  EXPECT_FALSE(rec->requires_control());
}

TEST_F(CancelMissionRecoveryTestCase, CanHandleAnyHardwareIdAtErrorLevelOrAbove)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "cancel1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "controller_stuck";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus not_an_error = matching;
  not_an_error.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  EXPECT_FALSE(rec->can_handle(not_an_error));
}

TEST_F(CancelMissionRecoveryTestCase, OnStartSetsTheCancelRequestFlag)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_start_node");
  auto rec = make_recovery(node, "cancel2");

  easynav::NavState nav_state;
  rec->internal_start(nav_state);

  ASSERT_TRUE(nav_state.has("mission_cancel_requested"));
  EXPECT_TRUE(nav_state.get<bool>("mission_cancel_requested"));
}

TEST_F(CancelMissionRecoveryTestCase, RunsWhileGoalManagerHasNotConsumedTheRequestYet)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_running_node");
  auto rec = make_recovery(node, "cancel3");

  easynav::NavState nav_state;
  rec->internal_start(nav_state);  // sets mission_cancel_requested = true

  auto status = rec->internal_cycle(nav_state);
  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
}

TEST_F(CancelMissionRecoveryTestCase, ReportsFailedOnceGoalManagerResetsTheFlag)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_succeed_node");
  auto rec = make_recovery(node, "cancel4");

  easynav::NavState nav_state;
  rec->internal_start(nav_state);

  // Simulate GoalManager::update() having consumed the request.
  nav_state.set("mission_cancel_requested", false);

  // FAILED, not SUCCEEDED: cancelling the mission does not resolve the diagnostic that
  // triggered this mitigation, so it must not claim success.
  auto status = rec->internal_cycle(nav_state);
  EXPECT_EQ(status, easynav::RecoveryStatus::FAILED);
}
