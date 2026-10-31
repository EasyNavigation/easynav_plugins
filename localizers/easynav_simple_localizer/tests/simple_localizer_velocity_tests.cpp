// Copyright 2026 Intelligent Robotics Lab
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
/// \brief robot_pose carries the robot's velocity, from the odometry topic.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "easynav_simple_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{

class TestAMCLLocalizer : public easynav::AMCLLocalizer
{
public:
  using easynav::AMCLLocalizer::odom_callback;
};

geometry_msgs::msg::Quaternion yaw_q(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

class AMCLLocalizerVelocityTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}

  std::shared_ptr<TestAMCLLocalizer> make()
  {
    node_ = std::make_shared<easynav::LocalizerNode>(rclcpp::NodeOptions());
    auto localizer = std::make_shared<TestAMCLLocalizer>();
    localizer->initialize(node_, "test");
    return localizer;
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
};

}  // namespace

TEST_F(AMCLLocalizerVelocityTest, NoOdometryYetIsStopped)
{
  const auto twist = make()->get_pose().twist.twist;
  EXPECT_EQ(twist.linear.x, 0.0);
  EXPECT_EQ(twist.linear.y, 0.0);
  EXPECT_EQ(twist.angular.z, 0.0);
}

TEST_F(AMCLLocalizerVelocityTest, TheOdometryTopicGivesTheVelocity)
{
  auto localizer = make();

  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->pose.pose.orientation = yaw_q(0.0);
  odom->twist.twist.linear.x = 0.4;
  odom->twist.twist.linear.y = -0.1;
  odom->twist.twist.angular.z = 0.7;
  localizer->odom_callback(std::move(odom));
  auto twist = localizer->get_pose().twist.twist;
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(twist.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.7);

  // Turning in place is not stopped.
  odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->pose.pose.orientation = yaw_q(0.0);
  odom->twist.twist.angular.z = -1.0;
  localizer->odom_callback(std::move(odom));
  twist = localizer->get_pose().twist.twist;
  EXPECT_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, -1.0);
}
