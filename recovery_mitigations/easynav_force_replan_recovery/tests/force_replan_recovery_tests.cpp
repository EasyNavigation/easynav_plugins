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

#include "easynav_force_replan_recovery/ForceReplanRecovery.hpp"

class ForceReplanRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::ForceReplanRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::ForceReplanRecovery>();
    rec->initialize(node, name);
    return rec;
  }
};

TEST_F(ForceReplanRecoveryTestCase, DoesNotRequireControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "replan0");
  EXPECT_FALSE(rec->requires_control());
}

TEST_F(ForceReplanRecoveryTestCase, CanHandleOnlyPlannerErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "replan1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "planner";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus wrong_hardware = matching;
  wrong_hardware.hardware_id = "obstacle_proximity";
  EXPECT_FALSE(rec->can_handle(wrong_hardware));

  diagnostic_msgs::msg::DiagnosticStatus just_warn = matching;
  just_warn.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  EXPECT_FALSE(rec->can_handle(just_warn));
}

TEST_F(ForceReplanRecoveryTestCase, RunningUntilSystemNodeConsumesTheRequest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_run_node");
  auto rec = make_recovery(node, "replan2");
  easynav::NavState nav_state;

  rec->internal_start(nav_state);

  ASSERT_TRUE(nav_state.has("force_replan_requested"));
  EXPECT_TRUE(nav_state.get<bool>("force_replan_requested"));

  // Still pending: SystemNode has not consumed it yet.
  EXPECT_EQ(rec->internal_cycle(nav_state), easynav::RecoveryStatus::RUNNING);

  // Simulate SystemNode::system_cycle() consuming the request.
  nav_state.set("force_replan_requested", false);

  EXPECT_EQ(rec->internal_cycle(nav_state), easynav::RecoveryStatus::SUCCEEDED);
}

TEST_F(ForceReplanRecoveryTestCase, SucceedsImmediatelyIfNeverStarted)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_never_node");
  auto rec = make_recovery(node, "replan3");
  easynav::NavState nav_state;  // "force_replan_requested" absent

  EXPECT_EQ(rec->internal_cycle(nav_state), easynav::RecoveryStatus::SUCCEEDED);
}
