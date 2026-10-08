// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

/// \file
/// \brief After a reconfiguration, the localizer starts from the pose the previous one left in
/// NavState ("robot_pose"), whatever localizer that was.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_simple_localizer/AMCLLocalizer.hpp"

namespace
{

constexpr double kInitX = 1.0, kInitY = 2.0, kInitYaw = 0.3;

nav_msgs::msg::Odometry pose_at(double x, double y, double yaw)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 1e-6;
  return odom;
}

class SimpleLastKnownPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::AMCLLocalizer> make_localizer(bool use_last_known = true)
  {
    auto node = std::make_shared<easynav::LocalizerNode>(
      rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("loc.initial_pose.x", kInitX),
        rclcpp::Parameter("loc.initial_pose.y", kInitY),
        rclcpp::Parameter("loc.initial_pose.yaw", kInitYaw),
        rclcpp::Parameter("loc.initial_pose.std_dev_xy", 1e-6),
        rclcpp::Parameter("loc.initial_pose.std_dev_yaw", 1e-6),
        rclcpp::Parameter("loc.initial_pose.use_last_known", use_last_known),
        rclcpp::Parameter("loc.min_noise_xy", 1e-6),
        rclcpp::Parameter("loc.min_noise_yaw", 1e-6),
      }));
    nodes_.push_back(node);
    auto localizer = std::make_shared<easynav::AMCLLocalizer>();
    localizer->initialize(node, "loc");
    return localizer;
  }

  static void expect_pose(
    const nav_msgs::msg::Odometry & odom, double x, double y, double yaw)
  {
    EXPECT_NEAR(odom.pose.pose.position.x, x, 0.05);
    EXPECT_NEAR(odom.pose.pose.position.y, y, 0.05);
    EXPECT_NEAR(tf2::getYaw(odom.pose.pose.orientation), yaw, 0.05);
  }

  std::vector<std::shared_ptr<easynav::LocalizerNode>> nodes_;
};

}  // namespace

TEST_F(SimpleLastKnownPoseTest, StartsFromTheLastKnownPose)
{
  auto localizer = make_localizer();
  easynav::NavState nav_state;
  nav_state.set("robot_pose", pose_at(-3.0, 4.0, -1.2));

  localizer->internal_update_rt(nav_state, true);
  expect_pose(localizer->get_pose(), -3.0, 4.0, -1.2);
}

TEST_F(SimpleLastKnownPoseTest, KeepsTheInitialPoseWhenDisabled)
{
  auto localizer = make_localizer(false);
  easynav::NavState nav_state;
  nav_state.set("robot_pose", pose_at(-3.0, 4.0, -1.2));

  localizer->internal_update_rt(nav_state, true);
  expect_pose(localizer->get_pose(), kInitX, kInitY, kInitYaw);
}

TEST_F(SimpleLastKnownPoseTest, KeepsTheInitialPoseOnAFreshStart)
{
  auto localizer = make_localizer();
  easynav::NavState nav_state;

  localizer->internal_update_rt(nav_state, true);
  expect_pose(localizer->get_pose(), kInitX, kInitY, kInitYaw);
}

TEST_F(SimpleLastKnownPoseTest, IgnoresAPoseInAnotherFrame)
{
  auto localizer = make_localizer();
  easynav::NavState nav_state;
  auto odom = pose_at(-3.0, 4.0, -1.2);
  odom.header.frame_id = "odom";
  nav_state.set("robot_pose", odom);

  localizer->internal_update_rt(nav_state, true);
  expect_pose(localizer->get_pose(), kInitX, kInitY, kInitYaw);
}

TEST_F(SimpleLastKnownPoseTest, ChainOfReconfigurations)
{
  // Each new instance continues from where the previous one left the robot.
  easynav::NavState nav_state;
  nav_state.set("robot_pose", pose_at(-3.0, 4.0, -1.2));
  for (int i = 0; i < 3; ++i) {
    auto localizer = make_localizer();
    localizer->internal_update_rt(nav_state, true);
    expect_pose(localizer->get_pose(), -3.0 + i, 4.0, -1.2);
    nav_state.set("robot_pose", pose_at(-3.0 + i + 1, 4.0, -1.2));
  }
}
