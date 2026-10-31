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
/// \brief robot_pose carries the robot's velocity, from the odometry topic or from TF.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_navmap_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{

class TestAMCLLocalizer : public easynav::navmap::AMCLLocalizer
{
public:
  using easynav::navmap::AMCLLocalizer::odom_callback;
  using easynav::navmap::AMCLLocalizer::update_odom_from_tf;
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

/// odom -> robot footprint at \p t seconds.
void set_odom_tf(double t, double x, double y, double yaw)
{
  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
  tf.header.frame_id = tf_info.odom_frame;
  tf.child_frame_id = tf_info.robot_footprint_frame;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.rotation = yaw_q(yaw);
  easynav::RTTFBuffer::getInstance()->setTransform(tf, "test", false);
}

class AMCLLocalizerVelocityTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}

  std::shared_ptr<TestAMCLLocalizer> make(bool from_tf)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("test.compute_odom_from_tf", from_tf)});
    node_ = std::make_shared<easynav::LocalizerNode>(options);
    auto localizer = std::make_shared<TestAMCLLocalizer>();
    localizer->initialize(node_, "test");
    return localizer;
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
};

}  // namespace

TEST_F(AMCLLocalizerVelocityTest, NoOdometryYetIsStopped)
{
  const auto twist = make(false)->get_pose().twist.twist;
  EXPECT_EQ(twist.linear.x, 0.0);
  EXPECT_EQ(twist.linear.y, 0.0);
  EXPECT_EQ(twist.angular.z, 0.0);
}

TEST_F(AMCLLocalizerVelocityTest, TheOdometryTopicGivesTheVelocity)
{
  auto localizer = make(false);

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

TEST_F(AMCLLocalizerVelocityTest, TFGivesTheVelocityFromItsMotion)
{
  auto localizer = make(true);

  // The first sample: no motion yet.
  set_odom_tf(10.0, 1.0, 2.0, M_PI / 2.0);
  localizer->update_odom_from_tf();
  EXPECT_EQ(localizer->get_pose().twist.twist.linear.x, 0.0);

  // 0.2 m ahead (along +y in odom, the robot's heading) and 0.3 rad in 0.5 s.
  set_odom_tf(10.5, 1.0, 2.2, M_PI / 2.0 + 0.3);
  localizer->update_odom_from_tf();
  auto twist = localizer->get_pose().twist.twist;
  EXPECT_NEAR(twist.linear.x, 0.4, 1e-9);
  EXPECT_NEAR(twist.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(twist.angular.z, 0.6, 1e-9);

  // The same sample again (TF slower than the cycle): kept.
  localizer->update_odom_from_tf();
  twist = localizer->get_pose().twist.twist;
  EXPECT_NEAR(twist.linear.x, 0.4, 1e-9);
  EXPECT_NEAR(twist.angular.z, 0.6, 1e-9);

  // Turning in place.
  set_odom_tf(11.0, 1.0, 2.2, M_PI / 2.0 - 0.2);
  localizer->update_odom_from_tf();
  twist = localizer->get_pose().twist.twist;
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(twist.angular.z, -1.0, 1e-9);

  // Stopped.
  set_odom_tf(11.5, 1.0, 2.2, M_PI / 2.0 - 0.2);
  localizer->update_odom_from_tf();
  twist = localizer->get_pose().twist.twist;
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(twist.angular.z, 0.0, 1e-9);
}
