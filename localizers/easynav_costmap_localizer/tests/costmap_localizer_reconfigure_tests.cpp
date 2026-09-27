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
/// \brief Regression test: a plugin must tolerate initialize() being
/// called twice on the same node (as happens across a cleanup/reconfigure
/// cycle) without throwing.

#include "easynav_costmap_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

class CostmapLocalizerReconfigureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(CostmapLocalizerReconfigureTest, InitializeTwiceOnSameNodeDoesNotThrow)
{
  // AMCLLocalizer::on_initialize() requires a LocalizerNode, not a plain LifecycleNode.
  auto node = std::make_shared<easynav::LocalizerNode>();

  auto plugin1 = std::make_shared<easynav::AMCLLocalizer>();
  ASSERT_NO_THROW(plugin1->initialize(node, "test_localizer"));

  auto plugin2 = std::make_shared<easynav::AMCLLocalizer>();
  ASSERT_NO_THROW(plugin2->initialize(node, "test_localizer"));
}

namespace
{

// What a previous localizer instance leaves in NavState.
void set_robot_at(
  easynav::NavState & nav_state, double x, double y, const std::string & frame = "map")
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = frame;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.01;
  odom.pose.covariance[35] = 0.01;

  nav_state.set("robot_pose", odom);
}

nav_msgs::msg::Odometry first_cycle(easynav::AMCLLocalizer & plugin, easynav::NavState & nav_state)
{
  plugin.internal_update_rt(nav_state, true);
  return nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
}

}  // namespace

TEST_F(CostmapLocalizerReconfigureTest, NewInstanceStartsFromTheLastKnownPose)
{
  // Initial pose is (0, 0); the previous instance had the robot at (3, 2).
  auto node = std::make_shared<easynav::LocalizerNode>();
  auto plugin = std::make_shared<easynav::AMCLLocalizer>();
  plugin->initialize(node, "last_pose_localizer");

  easynav::NavState nav_state;
  set_robot_at(nav_state, 3.0, 2.0);
  const auto pose = first_cycle(*plugin, nav_state);

  EXPECT_NEAR(pose.pose.pose.position.x, 3.0, 0.3);
  EXPECT_NEAR(pose.pose.pose.position.y, 2.0, 0.3);
}

TEST_F(CostmapLocalizerReconfigureTest, LastKnownPoseIsIgnoredWhenDisabled)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions().parameter_overrides(
      {{"disabled_localizer.initial_pose.use_last_known", false}}));
  auto plugin = std::make_shared<easynav::AMCLLocalizer>();
  plugin->initialize(node, "disabled_localizer");

  easynav::NavState nav_state;
  set_robot_at(nav_state, 3.0, 2.0);
  const auto pose = first_cycle(*plugin, nav_state);

  EXPECT_NEAR(pose.pose.pose.position.x, 0.0, 1.0);
  EXPECT_NEAR(pose.pose.pose.position.y, 0.0, 1.0);
}

TEST_F(CostmapLocalizerReconfigureTest, LastKnownPoseOutsideTheMapFrameIsIgnored)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  auto plugin = std::make_shared<easynav::AMCLLocalizer>();
  plugin->initialize(node, "odom_frame_localizer");

  easynav::NavState nav_state;
  set_robot_at(nav_state, 3.0, 2.0, "odom");
  const auto pose = first_cycle(*plugin, nav_state);

  EXPECT_NEAR(pose.pose.pose.position.x, 0.0, 1.0);
  EXPECT_NEAR(pose.pose.pose.position.y, 0.0, 1.0);
}
