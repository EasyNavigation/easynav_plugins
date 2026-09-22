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
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_costmap_localizer/AmclRelocalizeMitigation.hpp"

class AmclRelocalizeMitigationTestCase : public ::testing::Test
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

  std::shared_ptr<easynav::AmclRelocalizeMitigation> make_mitigation(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto mit = std::make_shared<easynav::AmclRelocalizeMitigation>();
    mit->initialize(node, name);
    return mit;
  }
};

TEST_F(AmclRelocalizeMitigationTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto mit = make_mitigation(node, "amcl_mit0");
  EXPECT_TRUE(mit->requires_control());
}

TEST_F(AmclRelocalizeMitigationTestCase, CanHandleOnlyLocalizerAmclErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto mit = make_mitigation(node, "amcl_mit1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "localizer.amcl";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(mit->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus wrong_hardware = matching;
  wrong_hardware.hardware_id = "planner";
  EXPECT_FALSE(mit->can_handle(wrong_hardware));

  diagnostic_msgs::msg::DiagnosticStatus not_an_error = matching;
  not_an_error.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  EXPECT_FALSE(mit->can_handle(not_an_error));
}

TEST_F(AmclRelocalizeMitigationTestCase, RotatesInPlaceWhileDivergedAndWithinTimeout)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_rotate_node",
    rclcpp::NodeOptions().append_parameter_override("amcl_mit2.rotation_speed", 0.4));
  auto mit = make_mitigation(node, "amcl_mit2");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 5.0);  // well above default threshold

  mit->internal_start(nav_state);
  auto status = mit->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.4);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

TEST_F(AmclRelocalizeMitigationTestCase, SucceedsOnceCovarianceDropsBelowThreshold)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_succeed_node",
    rclcpp::NodeOptions().append_parameter_override("amcl_mit3.covariance_threshold", 0.5));
  auto mit = make_mitigation(node, "amcl_mit3");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 0.1);  // relocalized

  mit->internal_start(nav_state);
  auto status = mit->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::SUCCEEDED);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}

TEST_F(AmclRelocalizeMitigationTestCase, FailsAfterTimeoutWithoutRelocalizing)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_timeout_node",
    rclcpp::NodeOptions().append_parameter_override("amcl_mit4.timeout", 0.05));
  auto mit = make_mitigation(node, "amcl_mit4");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 5.0);  // still diverged

  mit->internal_start(nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));  // past the 50 ms timeout
  auto status = mit->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav::RecoveryStatus::FAILED);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}
