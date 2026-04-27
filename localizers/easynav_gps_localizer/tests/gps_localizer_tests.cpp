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

#include "easynav_gps_localizer/GpsLocalizer.hpp"

#include "easynav_common/types/NavState.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{

class FriendGpsLocalizer : public easynav::GpsLocalizer
{
public:
  using easynav::GpsLocalizer::init_pose_sub_;
  using easynav::GpsLocalizer::update;
};

class GpsLocalizerInitialPoseTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

}  // namespace

TEST_F(GpsLocalizerInitialPoseTest, SubscribesToInitialPoseWithDefaultCallbackGroup)
{
  const double x0 = 1.1;
  const double y0 = -0.7;

  const double x1 = -2.3;
  const double y1 = 0.4;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.initial_pose.x", x0),
    rclcpp::Parameter("test.initial_pose.y", y0),
    rclcpp::Parameter("test.initial_pose.yaw", 0.0),
  });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_gps_localizer_node", options);
  auto localizer = std::make_shared<FriendGpsLocalizer>();

  localizer->initialize(node, "test");

  ASSERT_NE(localizer->init_pose_sub_, nullptr);

  const auto infos = node->get_subscriptions_info_by_topic("initialpose");
  ASSERT_FALSE(infos.empty());

  const bool has_expected_type = std::any_of(
    infos.begin(), infos.end(),
    [](const rclcpp::TopicEndpointInfo & info) {
      return info.topic_type() == "geometry_msgs/msg/PoseWithCovarianceStamped";
    });
  EXPECT_TRUE(has_expected_type);

  auto pub_node = std::make_shared<rclcpp::Node>("gps_test_pub_node");
  auto initialpose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "robot/gps/fix", rclcpp::SensorDataQoS().reliable());
  auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node->get_node_base_interface());

  {
    const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < connect_deadline &&
      (initialpose_pub->get_subscription_count() == 0 ||
      gps_pub->get_subscription_count() == 0 ||
      imu_pub->get_subscription_count() == 0))
    {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = pub_node->now();
  imu_msg.header.frame_id = "";
  imu_msg.angular_velocity.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_pub->publish(imu_msg);

  sensor_msgs::msg::NavSatFix gps_msg;
  gps_msg.header.stamp = pub_node->now();
  gps_msg.header.frame_id = "";
  gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  gps_msg.latitude = 42.0;
  gps_msg.longitude = -1.0;
  gps_msg.altitude = 0.0;
  gps_pub->publish(gps_msg);

  easynav::NavState nav_state;

  // Deliver GPS/IMU callbacks and run update() until the initial pose is applied.
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      localizer->update(nav_state);

      if (nav_state.has("robot_pose")) {
        const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
        const bool ok =
          std::abs(odom.pose.pose.position.x - x0) < 1e-9 &&
          std::abs(odom.pose.pose.position.y - y0) < 1e-9;
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
    EXPECT_NEAR(odom.pose.pose.position.x, x0, 1e-9);
    EXPECT_NEAR(odom.pose.pose.position.y, y0, 1e-9);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
  init_pose_msg.header.stamp = pub_node->now();
  init_pose_msg.header.frame_id = "map";
  init_pose_msg.pose.pose.position.x = x1;
  init_pose_msg.pose.pose.position.y = y1;
  init_pose_msg.pose.pose.position.z = 0.0;
  init_pose_msg.pose.pose.orientation.w = 1.0;
  init_pose_msg.pose.covariance.fill(0.0);
  initialpose_pub->publish(init_pose_msg);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    localizer->update(nav_state);

    const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
    const bool ok =
      std::abs(odom.pose.pose.position.x - x1) < 1e-9 &&
      std::abs(odom.pose.pose.position.y - y1) < 1e-9;

    if (ok) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    const auto odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
    EXPECT_NEAR(odom.pose.pose.position.x, x1, 1e-9);
    EXPECT_NEAR(odom.pose.pose.position.y, y1, 1e-9);
  }
}
