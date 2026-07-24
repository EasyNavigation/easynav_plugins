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

#include "easynav_simple_localizer/AMCLLocalizer.hpp"

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
  const double x0 = 0.3;
  const double y0 = 1.7;
  const double yaw0 = -0.8;

  const double x1 = 2.2;
  const double y1 = -0.4;
  const double yaw1 = 1.1;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.num_particles", 100),
    rclcpp::Parameter("test.initial_pose.x", x0),
    rclcpp::Parameter("test.initial_pose.y", y0),
    rclcpp::Parameter("test.initial_pose.yaw", yaw0),
    rclcpp::Parameter("test.initial_pose.std_dev_xy", 1e-12),
    rclcpp::Parameter("test.initial_pose.std_dev_yaw", 1e-12),
    rclcpp::Parameter("test.min_noise_xy", 1e-12),
    rclcpp::Parameter("test.min_noise_yaw", 1e-12),
  });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_simple_localizer_node", options);
  auto localizer = std::make_shared<FriendAMCLLocalizer>();

  localizer->initialize(node, "test");

  ASSERT_NE(localizer->init_pose_sub_, nullptr);

  {
    const tf2::Transform tf = localizer->getEstimatedPose();
    EXPECT_NEAR(tf.getOrigin().x(), x0, 1e-6);
    EXPECT_NEAR(tf.getOrigin().y(), y0, 1e-6);
    EXPECT_NEAR(yaw_from_tf(tf), yaw0, 1e-6);
  }

  {
    const nav_msgs::msg::Odometry odom = localizer->get_pose();
    EXPECT_NEAR(odom.pose.pose.position.x, x0, 1e-6);
    EXPECT_NEAR(odom.pose.pose.position.y, y0, 1e-6);
    EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), yaw0, 1e-6);
  }

  const auto infos = node->get_subscriptions_info_by_topic("initialpose");
  ASSERT_FALSE(infos.empty());

<<<<<<< HEAD
  auto cell2 = map_ptr->metric_to_cell(-1.0, -1.0);
  EXPECT_TRUE(map_ptr->at(cell2.first, cell2.second));
}

/// \brief Map loading via subscription test
TEST_F(AMCLLocalizerTest, IncomingOccupancyGridUpdatesMaps)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node2");
  auto manager = std::make_shared<easynav::AMCLLocalizer>();
  easynav::TFInfo tf_info;
  tf_info.map_frame = "world_map";
  tf_info.odom_frame = "world_odom";
  tf_info.robot_frame = "world_base";
  tf_info.robot_frame = "world_footprint_base";
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  manager->initialize(node, "test2");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "test_node2/test2/incoming_map", rclcpp::QoS(1).transient_local().reliable());

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = tf_info.map_frame;
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.resolution = 0.2;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -0.6;
  grid.data.assign(100, 0);
  grid.data[55] = 100;  // Mark cell (5,5) as occupied

  pub->publish(grid);

  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto static_map = std::dynamic_pointer_cast<easynav::SimpleMap>(manager->get_static_map());
  ASSERT_TRUE(static_map != nullptr);

  EXPECT_EQ(static_map->at(5, 5), 1);
  EXPECT_EQ(static_map->at(1, 1), 0);
}

class FriendAMCLLocalizer : public easynav::AMCLLocalizer {
public:
  void force_path(const std::string & path) {map_path_ = path;}
};

TEST_F(AMCLLocalizerTest, SavemapServiceWorks)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_savemap_node");
  auto manager = std::make_shared<easynav::AMCLLocalizer>();
  manager->initialize(node, "test_savemap");

  auto static_map = std::make_shared<easynav::SimpleMap>();
  static_map->initialize(4, 4, 0.5, -1.0, -1.0, 0);
  static_map->at(1, 1) = true;
  static_map->at(2, 2) = true;
  manager->set_static_map(static_map);

  const std::string test_map_file = "/tmp/savemap_test_map.txt";
  const std::string service_name = "/test_savemap_node/test_savemap/savemap";

  std::static_pointer_cast<FriendAMCLLocalizer>(manager)->force_path(test_map_file);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  executor.spin_until_future_complete(future);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("saved"), std::string::npos);

  std::ifstream infile(test_map_file);
  ASSERT_TRUE(infile.is_open());

  std::string first_line;
  std::getline(infile, first_line);
  EXPECT_NE(first_line.find("4 4"), std::string::npos);
  EXPECT_NE(first_line.find("0.5"), std::string::npos);

  infile.close();
  std::remove(test_map_file.c_str());
=======
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
>>>>>>> kilted
}
