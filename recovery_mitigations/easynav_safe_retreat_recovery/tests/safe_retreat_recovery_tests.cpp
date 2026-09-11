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
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_safe_retreat_recovery/SafeRetreatRecovery.hpp"

class SafeRetreatRecoveryTestCase : public ::testing::Test
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

  static easynav::PointPerception make_obstacle_at(double x, double y)
  {
    easynav::PointPerception perception;
    perception.frame_id = "base_link";
    perception.stamp = rclcpp::Time(0);
    perception.valid = true;
    perception.data.points.resize(1);
    perception.data.points[0].x = x;
    perception.data.points[0].y = y;
    perception.data.points[0].z = 0.0;
    return perception;
  }

  std::shared_ptr<easynav::SafeRetreatRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::SafeRetreatRecovery>();
    rec->initialize(node, name);
    return rec;
  }
};

TEST_F(SafeRetreatRecoveryTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "retreat0");
  EXPECT_TRUE(rec->requires_control());
}

TEST_F(SafeRetreatRecoveryTestCase, CanHandleOnlyObstacleProximityErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "retreat1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "obstacle_proximity";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus wrong_hardware = matching;
  wrong_hardware.hardware_id = "planner";
  EXPECT_FALSE(rec->can_handle(wrong_hardware));

  diagnostic_msgs::msg::DiagnosticStatus not_an_error = matching;
  not_an_error.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  EXPECT_FALSE(rec->can_handle(not_an_error));
}

TEST_F(SafeRetreatRecoveryTestCase, RetreatsBackwardWhileObstacleAheadAndClose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_retreat_node");
  auto rec = make_recovery(node, "retreat2");

  easynav::NavState nav_state;
  nav_state.set("scan", make_obstacle_at(0.2, 0.0));  // ahead, well within default safe_distance

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_LT(cmd.twist.linear.x, 0.0);
}

TEST_F(SafeRetreatRecoveryTestCase, SucceedsOnceFarEnough)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_far_node");
  auto rec = make_recovery(node, "retreat3");

  easynav::NavState nav_state;
  nav_state.set("scan", make_obstacle_at(5.0, 0.0));  // far away

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::SUCCEEDED);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

TEST_F(SafeRetreatRecoveryTestCase, SucceedsWhenNoObstaclePerceptionAtAll)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_none_node");
  auto rec = make_recovery(node, "retreat4");

  easynav::NavState nav_state;  // no perception at all

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::SUCCEEDED);
}

TEST_F(SafeRetreatRecoveryTestCase, FailsSafelyWhenObstacleIsBehind)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_behind_node");
  auto rec = make_recovery(node, "retreat5");

  easynav::NavState nav_state;
  nav_state.set("scan", make_obstacle_at(-0.2, 0.0));  // directly behind, close

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::FAILED);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}
