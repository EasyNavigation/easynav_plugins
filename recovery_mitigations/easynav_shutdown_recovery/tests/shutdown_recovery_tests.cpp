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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "easynav_shutdown_recovery/ShutdownRecovery.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;

class ShutdownRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::ShutdownRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::ShutdownRecovery>();
    rec->initialize(node, name);
    return rec;
  }

  static DiagnosticStatus make_status(uint8_t level, const std::string & hardware_id)
  {
    DiagnosticStatus status;
    status.level = level;
    status.hardware_id = hardware_id;
    status.message = "broken " + hardware_id;
    return status;
  }

  static void add_diagnostic(
    easynav::NavState & nav_state, const std::string & name, const DiagnosticStatus & status)
  {
    const std::string key = "diagnostics." + name;
    auto named = status;
    named.name = name;
    nav_state.set(key, named);
    auto members = nav_state.get_group_keys("diagnostics");
    members.push_back(key);
    nav_state.set_group("diagnostics", members);
  }
};

TEST_F(ShutdownRecoveryTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sd_rc_node");
  auto rec = make_recovery(node, "shutdown0");
  EXPECT_TRUE(rec->requires_control());
}

TEST_F(ShutdownRecoveryTestCase, HandlesOnlyRosGraphErrorsByDefault)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sd_ch_node");
  auto rec = make_recovery(node, "shutdown1");

  EXPECT_TRUE(rec->can_handle(make_status(DiagnosticStatus::ERROR, "ros_graph")));
  EXPECT_TRUE(rec->can_handle(make_status(DiagnosticStatus::STALE, "ros_graph")));
  EXPECT_FALSE(rec->can_handle(make_status(DiagnosticStatus::WARN, "ros_graph")));
  EXPECT_FALSE(rec->can_handle(make_status(DiagnosticStatus::ERROR, "planner")));
}

TEST_F(ShutdownRecoveryTestCase, HandledHardwareIdsAreConfigurable)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"shutdown2.handled_hardware_ids", std::vector<std::string>{"localizer.amcl"}}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sd_param_node", options);
  auto rec = make_recovery(node, "shutdown2");

  EXPECT_TRUE(rec->can_handle(make_status(DiagnosticStatus::ERROR, "localizer.amcl")));
  EXPECT_FALSE(rec->can_handle(make_status(DiagnosticStatus::ERROR, "ros_graph")));
}

TEST_F(ShutdownRecoveryTestCase, OnStartRequestsShutdownWithTheOffendingDiagnostics)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sd_start_node");
  auto rec = make_recovery(node, "shutdown3");

  easynav::NavState nav_state;
  add_diagnostic(nav_state, "graph", make_status(DiagnosticStatus::ERROR, "ros_graph"));
  add_diagnostic(nav_state, "planner", make_status(DiagnosticStatus::ERROR, "planner"));

  rec->internal_start(nav_state);

  ASSERT_TRUE(nav_state.has("system_shutdown_requested"));
  EXPECT_TRUE(nav_state.get<bool>("system_shutdown_requested"));
  const auto reason = nav_state.get<std::string>("system_shutdown_reason");
  EXPECT_EQ(reason, "graph: broken ros_graph");
  // Diagnostics it does not handle are left to their own mitigations.
  EXPECT_EQ(reason.find("planner"), std::string::npos) << reason;
  // No active goal: nothing to cancel.
  EXPECT_FALSE(nav_state.has("mission_cancel_requested"));
}

TEST_F(ShutdownRecoveryTestCase, OnStartCancelsTheActiveMission)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sd_goal_node");
  auto rec = make_recovery(node, "shutdown4");

  easynav::NavState nav_state;
  add_diagnostic(nav_state, "graph", make_status(DiagnosticStatus::ERROR, "ros_graph"));
  nav_msgs::msg::Goals goals;
  goals.goals.push_back(geometry_msgs::msg::PoseStamped());
  nav_state.set("goals", goals);

  rec->internal_start(nav_state);

  ASSERT_TRUE(nav_state.has("mission_cancel_requested"));
  EXPECT_TRUE(nav_state.get<bool>("mission_cancel_requested"));
}

TEST_F(ShutdownRecoveryTestCase, HoldsTheRobotStillAndNeverFinishes)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sd_cycle_node");
  auto rec = make_recovery(node, "shutdown5");

  easynav::NavState nav_state;
  geometry_msgs::msg::TwistStamped moving;
  moving.twist.linear.x = 0.5;
  nav_state.set("cmd_vel", moving);

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(rec->internal_cycle(nav_state), easynav::RecoveryStatus::RUNNING);
  }
  const auto cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}
