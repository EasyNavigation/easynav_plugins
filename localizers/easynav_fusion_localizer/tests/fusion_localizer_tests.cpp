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
#include <thread>
#include <vector>

#include "easynav_fusion_localizer/FusionLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

#include "easynav_common/types/NavState.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
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

class FriendFusionLocalizer : public easynav::FusionLocalizer
{
public:
  using easynav::FusionLocalizer::init_pose_sub_;
  using easynav::FusionLocalizer::update_rt;
};

class FusionLocalizerInitialPoseTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

}  // namespace

TEST_F(FusionLocalizerInitialPoseTest, SubscribesToInitialPoseWithDefaultCallbackGroup)
{
  const double x0 = 1.5;
  const double y0 = -0.25;
  const double yaw0 = 0.7;

  const double x1 = -0.8;
  const double y1 = 2.1;
  const double yaw1 = -1.2;

  // Provide at least one parameter override so FusionLocalizer considers a filter configured.
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.global_filter.frequency", 30.0),
    rclcpp::Parameter("test.initial_pose.x", x0),
    rclcpp::Parameter("test.initial_pose.y", y0),
    rclcpp::Parameter("test.initial_pose.yaw", yaw0),
  });

  auto node = std::make_shared<easynav::LocalizerNode>(options);
  node->declare_parameter<std::vector<std::string>>("localizer_types", std::vector<std::string>{});

  auto localizer = std::make_shared<FriendFusionLocalizer>();

  ASSERT_NO_THROW(localizer->initialize(node, "test"));

  ASSERT_NE(localizer->init_pose_sub_, nullptr);

  const auto infos = node->get_subscriptions_info_by_topic("initialpose");
  ASSERT_FALSE(infos.empty());

  const bool has_expected_type = std::any_of(
    infos.begin(), infos.end(),
    [](const rclcpp::TopicEndpointInfo & info) {
      return info.topic_type() == "geometry_msgs/msg/PoseWithCovarianceStamped";
    });
  EXPECT_TRUE(has_expected_type);

  auto pub_node = std::make_shared<rclcpp::Node>("fusion_test_pub_node");
  auto initialpose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node->get_node_base_interface());

  {
    const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < connect_deadline &&
      initialpose_pub->get_subscription_count() == 0)
    {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  easynav::NavState nav_state;
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      localizer->update_rt(nav_state);
      if (nav_state.has("robot_pose")) {
        const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
        const bool ok =
          std::abs(odom.pose.pose.position.x - x0) < 1e-6 &&
          std::abs(odom.pose.pose.position.y - y0) < 1e-6 &&
          std::abs(yaw_from_quat(odom.pose.pose.orientation) - yaw0) < 1e-3;
        if (ok) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  ASSERT_TRUE(nav_state.has("robot_pose"));
  {
    const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
    EXPECT_NEAR(odom.pose.pose.position.x, x0, 1e-6);
    EXPECT_NEAR(odom.pose.pose.position.y, y0, 1e-6);
    EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), yaw0, 1e-3);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
  init_pose_msg.header.stamp = pub_node->now();
  init_pose_msg.header.frame_id = "map";
  init_pose_msg.pose.pose.position.x = x1;
  init_pose_msg.pose.pose.position.y = y1;
  init_pose_msg.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw1);
  init_pose_msg.pose.pose.orientation = tf2::toMsg(q);
  init_pose_msg.pose.covariance.fill(0.0);
  initialpose_pub->publish(init_pose_msg);

  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      localizer->update_rt(nav_state);

      const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
      const bool ok =
        std::abs(odom.pose.pose.position.x - x1) < 1e-6 &&
        std::abs(odom.pose.pose.position.y - y1) < 1e-6 &&
        std::abs(yaw_from_quat(odom.pose.pose.orientation) - yaw1) < 1e-3;

      if (ok) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  {
    const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
    EXPECT_NEAR(odom.pose.pose.position.x, x1, 1e-6);
    EXPECT_NEAR(odom.pose.pose.position.y, y1, 1e-6);
    EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), yaw1, 1e-3);
  }
}
