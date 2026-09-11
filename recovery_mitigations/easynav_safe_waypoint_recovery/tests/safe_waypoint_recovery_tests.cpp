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

#include "easynav_safe_waypoint_recovery/SafeWaypointRecovery.hpp"

using namespace std::chrono_literals;

class SafeWaypointRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::SafeWaypointRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::SafeWaypointRecovery>();
    rec->initialize(node, name);
    return rec;
  }
};

TEST_F(SafeWaypointRecoveryTestCase, DoesNotRequireControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "waypoint0");
  EXPECT_FALSE(rec->requires_control());
}

TEST_F(SafeWaypointRecoveryTestCase, DisabledByDefaultCannotHandleAnything)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_disabled_node");
  auto rec = make_recovery(node, "waypoint1");

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_FALSE(rec->can_handle(status));
}

TEST_F(SafeWaypointRecoveryTestCase, EnabledHandlesAnyErrorOrAboveDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_enabled_node",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("waypoint2.enabled", true)}));
  auto rec = make_recovery(node, "waypoint2");

  diagnostic_msgs::msg::DiagnosticStatus error_status;
  error_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(error_status));

  diagnostic_msgs::msg::DiagnosticStatus warn_status;
  warn_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  EXPECT_FALSE(rec->can_handle(warn_status));
}

TEST_F(SafeWaypointRecoveryTestCase, PublishesTheConfiguredWaypointOnStart)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_publish_node",
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("waypoint3.enabled", true),
    rclcpp::Parameter("waypoint3.safe_waypoint.x", 1.5),
    rclcpp::Parameter("waypoint3.safe_waypoint.y", -2.0),
    rclcpp::Parameter("waypoint3.safe_waypoint.frame", "map"),
  }));
  auto rec = make_recovery(node, "waypoint3");

  auto listener_node = std::make_shared<rclcpp::Node>("test_listener_node");
  geometry_msgs::msg::PoseStamped::SharedPtr received;
  auto sub = listener_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    [&received](geometry_msgs::msg::PoseStamped::SharedPtr msg) {received = msg;});

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(listener_node);

  easynav::NavState nav_state;
  rec->internal_start(nav_state);

  auto start = listener_node->now();
  while (!received && listener_node->now() - start < 2s) {
    exe.spin_some();
  }

  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(received->pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(received->pose.position.y, -2.0);

  EXPECT_EQ(rec->internal_cycle(nav_state), easynav::RecoveryStatus::SUCCEEDED);
}
