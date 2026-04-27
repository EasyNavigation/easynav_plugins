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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "easynav_costmap_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

double yaw_from_tf(const tf2::Transform & tf)
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

class FriendAMCLLocalizer : public easynav::AMCLLocalizer
{
public:
  using easynav::AMCLLocalizer::init_pose_sub_;
};

class AMCLLocalizerInitialPoseTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

}  // namespace

TEST_F(AMCLLocalizerInitialPoseTest, SubscribesToInitialPoseWithDefaultCallbackGroup)
{
  const double x0 = 1.25;
  const double y0 = -2.5;
  const double yaw0 = 0.4;

  const double x1 = -0.75;
  const double y1 = 0.9;
  const double yaw1 = -1.2;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.num_particles", 100),
    rclcpp::Parameter("test.initial_pose.x", x0),
    rclcpp::Parameter("test.initial_pose.y", y0),
    rclcpp::Parameter("test.initial_pose.yaw", yaw0),
    rclcpp::Parameter("test.initial_pose.std_dev_xy", 0.0),
    rclcpp::Parameter("test.initial_pose.std_dev_yaw", 0.0),
    rclcpp::Parameter("test.min_noise_xy", 0.0),
    rclcpp::Parameter("test.min_noise_yaw", 0.0),
    rclcpp::Parameter("test.compute_odom_from_tf", true),
  });

  auto node = std::make_shared<easynav::LocalizerNode>(options);
  auto localizer = std::make_shared<FriendAMCLLocalizer>();

  localizer->initialize(node, "test");

  ASSERT_NE(localizer->init_pose_sub_, nullptr);

  {
    const tf2::Transform tf = localizer->getEstimatedPose();
    EXPECT_NEAR(tf.getOrigin().x(), x0, 1e-9);
    EXPECT_NEAR(tf.getOrigin().y(), y0, 1e-9);
    EXPECT_NEAR(yaw_from_tf(tf), yaw0, 1e-9);
  }

  {
    const nav_msgs::msg::Odometry odom = localizer->get_pose();
    EXPECT_NEAR(odom.pose.pose.position.x, x0, 1e-9);
    EXPECT_NEAR(odom.pose.pose.position.y, y0, 1e-9);
    EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), yaw0, 1e-9);
  }

  const auto infos = node->get_subscriptions_info_by_topic("initialpose");
  ASSERT_FALSE(infos.empty());

  const bool has_expected_type = std::any_of(
    infos.begin(), infos.end(),
    [](const rclcpp::TopicEndpointInfo & info) {
      return info.topic_type() == "geometry_msgs/msg/PoseWithCovarianceStamped";
    });
  EXPECT_TRUE(has_expected_type);

  auto pub_node = std::make_shared<rclcpp::Node>("initialpose_pub_node");
  auto pub = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node->get_node_base_interface());

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = pub_node->now();
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = x1;
  msg.pose.pose.position.y = y1;
  msg.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw1);
  msg.pose.pose.orientation = tf2::toMsg(q);
  msg.pose.covariance.fill(0.0);

  {
    const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < connect_deadline &&
      pub->get_subscription_count() == 0)
    {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  pub->publish(msg);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    const tf2::Transform tf = localizer->getEstimatedPose();
    const nav_msgs::msg::Odometry odom = localizer->get_pose();

    const bool pose_ok =
      std::abs(tf.getOrigin().x() - x1) < 1e-9 &&
      std::abs(tf.getOrigin().y() - y1) < 1e-9 &&
      std::abs(yaw_from_tf(tf) - yaw1) < 1e-9 &&
      std::abs(odom.pose.pose.position.x - x1) < 1e-9 &&
      std::abs(odom.pose.pose.position.y - y1) < 1e-9 &&
      std::abs(yaw_from_quat(odom.pose.pose.orientation) - yaw1) < 1e-9;

    if (pose_ok) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    const tf2::Transform tf = localizer->getEstimatedPose();
    EXPECT_NEAR(tf.getOrigin().x(), x1, 1e-9);
    EXPECT_NEAR(tf.getOrigin().y(), y1, 1e-9);
    EXPECT_NEAR(yaw_from_tf(tf), yaw1, 1e-9);
  }

  {
    const nav_msgs::msg::Odometry odom = localizer->get_pose();
    EXPECT_NEAR(odom.pose.pose.position.x, x1, 1e-9);
    EXPECT_NEAR(odom.pose.pose.position.y, y1, 1e-9);
    EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), yaw1, 1e-9);
  }
}
