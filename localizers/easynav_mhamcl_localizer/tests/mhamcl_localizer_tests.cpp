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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "easynav_mhamcl_localizer/MHAMCLLocalizer.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
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

class FriendMHAMCLLocalizer : public easynav::mhamcl::MHAMCLLocalizer
{
public:
  using easynav::mhamcl::MHAMCLLocalizer::init_pose_sub_;
  using easynav::mhamcl::MHAMCLLocalizer::hypotheses_;
  using easynav::mhamcl::MHAMCLLocalizer::current_;
  using easynav::mhamcl::MHAMCLLocalizer::manage_hypotheses;
  using easynav::mhamcl::MHAMCLLocalizer::update_rt;
};

tf2::Transform make_pose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
}

easynav::mhamcl::TransformWeighted candidate(double x, double y, double yaw, double weight)
{
  easynav::mhamcl::TransformWeighted c;
  c.transform = make_pose(x, y, yaw);
  c.weight = weight;
  return c;
}

/// Localizer with a known pose at (0, 0) and up to three hypotheses, in a 20 x 20 m free map
struct HypothesesFixture
{
  HypothesesFixture()
  : map(200, 200, 0.1, -10.0, -10.0, easynav::FREE_SPACE)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("test.max_hypotheses", 3),
        rclcpp::Parameter("test.initial_pose.std_dev_xy", 0.05),
        rclcpp::Parameter("test.initial_pose.std_dev_yaw", 0.05),
    });
    node = std::make_shared<easynav::LocalizerNode>(options);
    localizer = std::make_shared<FriendMHAMCLLocalizer>();
    localizer->initialize(node, "test");
  }

  easynav::Costmap2D map;
  std::shared_ptr<easynav::LocalizerNode> node;
  std::shared_ptr<FriendMHAMCLLocalizer> localizer;
};

class MHAMCLLocalizerTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

}  // namespace

TEST_F(MHAMCLLocalizerTest, StartsAtTheInitialPoseWithOneHypothesis)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.initial_pose.x", 1.25),
    rclcpp::Parameter("test.initial_pose.y", -2.5),
    rclcpp::Parameter("test.initial_pose.yaw", 0.4),
    rclcpp::Parameter("test.initial_pose.std_dev_xy", 1e-3),
    rclcpp::Parameter("test.initial_pose.std_dev_yaw", 1e-3),
  });

  auto node = std::make_shared<easynav::LocalizerNode>(options);
  auto localizer = std::make_shared<FriendMHAMCLLocalizer>();
  localizer->initialize(node, "test");

  EXPECT_EQ(localizer->get_num_hypotheses(), 1u);

  const auto tf = localizer->getEstimatedPose();
  EXPECT_NEAR(tf.getOrigin().x(), 1.25, 1e-2);
  EXPECT_NEAR(tf.getOrigin().y(), -2.5, 1e-2);

  const auto odom = localizer->get_pose();
  EXPECT_NEAR(odom.pose.pose.position.x, 1.25, 1e-2);
  EXPECT_NEAR(odom.pose.pose.position.y, -2.5, 1e-2);
  EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), 0.4, 1e-2);
}

TEST_F(MHAMCLLocalizerTest, RejectsNonPositiveDeviations)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.distance_perception_error", 0.0),
  });

  auto node = std::make_shared<easynav::LocalizerNode>(options);
  auto localizer = std::make_shared<FriendMHAMCLLocalizer>();
  EXPECT_THROW(localizer->initialize(node, "test"), std::runtime_error);
}

TEST_F(MHAMCLLocalizerTest, InitialPoseTopicResetsThePose)
{
  const double x1 = -0.75;
  const double y1 = 0.9;
  const double yaw1 = -1.2;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.min_noise_xy", 1e-9),
    rclcpp::Parameter("test.min_noise_yaw", 1e-9),
  });

  auto node = std::make_shared<easynav::LocalizerNode>(options);
  auto localizer = std::make_shared<FriendMHAMCLLocalizer>();
  localizer->initialize(node, "test");

  ASSERT_NE(localizer->init_pose_sub_, nullptr);

  const auto infos = node->get_subscriptions_info_by_topic("initialpose");
  ASSERT_FALSE(infos.empty());
  EXPECT_TRUE(
    std::any_of(
      infos.begin(), infos.end(),
      [](const rclcpp::TopicEndpointInfo & info) {
        return info.topic_type() == "geometry_msgs/msg/PoseWithCovarianceStamped";
      }));

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
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw1);
  msg.pose.pose.orientation = tf2::toMsg(q);
  msg.pose.covariance.fill(0.0);

  const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < connect_deadline &&
    pub->get_subscription_count() == 0)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  pub->publish(msg);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    const auto tf = localizer->getEstimatedPose();
    if (std::abs(tf.getOrigin().x() - x1) < 1e-3 && std::abs(tf.getOrigin().y() - y1) < 1e-3) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const auto odom = localizer->get_pose();
  EXPECT_NEAR(odom.pose.pose.position.x, x1, 1e-3);
  EXPECT_NEAR(odom.pose.pose.position.y, y1, 1e-3);
  EXPECT_NEAR(yaw_from_quat(odom.pose.pose.orientation), yaw1, 1e-3);
  EXPECT_EQ(localizer->get_num_hypotheses(), 1u);
}

TEST_F(MHAMCLLocalizerTest, CandidatesCreateHypothesesOnlyWhereThereIsNoneNearby)
{
  HypothesesFixture f;
  ASSERT_EQ(f.localizer->hypotheses_.size(), 1u);
  f.localizer->hypotheses_.front()->set_quality(0.9);

  f.localizer->manage_hypotheses(
  {
    candidate(0.3, 0.1, 0.1, 0.95),     // covered by the initial hypothesis
    candidate(5.0, 5.0, 1.0, 0.9),      // new
    candidate(5.2, 5.1, 1.1, 0.85),     // covered by the previous one
    candidate(-5.0, 4.0, 0.0, 0.8),     // new
    candidate(-6.0, -6.0, 0.0, 0.7),    // there is no room for more
    candidate(-3.0, -3.0, 0.0, 0.3),    // below the threshold
    },
    f.map);

  EXPECT_EQ(f.localizer->hypotheses_.size(), 3u);
  EXPECT_EQ(f.localizer->get_num_hypotheses(), 3u);
}

TEST_F(MHAMCLLocalizerTest, HypothesesWithVeryLowQualityAreRemoved)
{
  HypothesesFixture f;
  f.localizer->manage_hypotheses({candidate(5.0, 5.0, 1.0, 0.9)}, f.map);
  ASSERT_EQ(f.localizer->hypotheses_.size(), 2u);

  f.localizer->hypotheses_[0]->set_quality(0.05);
  f.localizer->hypotheses_[1]->set_quality(0.9);
  f.localizer->manage_hypotheses({}, f.map);

  ASSERT_EQ(f.localizer->hypotheses_.size(), 1u);
  EXPECT_NEAR(f.localizer->getEstimatedPose().getOrigin().x(), 5.0, 0.5);
}

TEST_F(MHAMCLLocalizerTest, LastHypothesisIsNeverRemoved)
{
  HypothesesFixture f;
  f.localizer->hypotheses_.front()->set_quality(0.0);
  f.localizer->manage_hypotheses({}, f.map);
  EXPECT_EQ(f.localizer->hypotheses_.size(), 1u);
}

TEST_F(MHAMCLLocalizerTest, HypothesesOutOfFreeSpaceAreRemoved)
{
  HypothesesFixture f;
  f.localizer->manage_hypotheses({candidate(5.0, 5.0, 1.0, 0.9)}, f.map);
  ASSERT_EQ(f.localizer->hypotheses_.size(), 2u);

  for (auto & h : f.localizer->hypotheses_) {
    h->set_quality(0.9);
                                                                 }

  // Wall on top of the new hypothesis
  for (unsigned int x = 140; x < 160; ++x) {
    for (unsigned int y = 140; y < 160; ++y) {
      f.map.setCost(x, y, easynav::LETHAL_OBSTACLE);
    }
  }
  f.localizer->manage_hypotheses({}, f.map);

  ASSERT_EQ(f.localizer->hypotheses_.size(), 1u);
  EXPECT_NEAR(f.localizer->getEstimatedPose().getOrigin().x(), 0.0, 0.5);
}

TEST_F(MHAMCLLocalizerTest, HypothesesThatConvergeAreMerged)
{
  HypothesesFixture f;
  f.localizer->manage_hypotheses({candidate(5.0, 5.0, 1.0, 0.9)}, f.map);
  ASSERT_EQ(f.localizer->hypotheses_.size(), 2u);

  // Two hypotheses on the same place: reinit the second on top of the first
  f.localizer->hypotheses_[1]->init(make_pose(0.1, 0.0, 0.0), 0.05, 0.05, 0.9);
  f.localizer->hypotheses_[0]->set_quality(0.9);
  f.localizer->current_ = f.localizer->hypotheses_[1];
  f.localizer->manage_hypotheses({}, f.map);

  ASSERT_EQ(f.localizer->hypotheses_.size(), 1u);
  EXPECT_EQ(f.localizer->current_, f.localizer->hypotheses_.front());
}

TEST_F(MHAMCLLocalizerTest, SelectedHypothesisChangesOnlyIfClearlyBetter)
{
  HypothesesFixture f;
  f.localizer->manage_hypotheses({candidate(5.0, 5.0, 1.0, 0.9)}, f.map);
  ASSERT_EQ(f.localizer->hypotheses_.size(), 2u);

  auto first = f.localizer->hypotheses_[0];
  auto second = f.localizer->hypotheses_[1];
  f.localizer->current_ = first;

  // Better, but not enough
  first->set_quality(0.7);
  second->set_quality(0.8);
  f.localizer->manage_hypotheses({}, f.map);
  EXPECT_EQ(f.localizer->current_, first);

  // Clearly better
  second->set_quality(0.95);
  f.localizer->manage_hypotheses({}, f.map);
  EXPECT_EQ(f.localizer->current_, second);
  EXPECT_NEAR(f.localizer->getEstimatedPose().getOrigin().x(), 5.0, 0.5);
}

namespace
{

nav_msgs::msg::Odometry make_odom(double x, double y, double yaw, int sec)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = sec;
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.pose.orientation = tf2::toMsg(q);
  return msg;
}

}  // namespace

TEST_F(MHAMCLLocalizerTest, PredictsWithTheOdometryPerceptionOfNavState)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.initial_pose.x", 1.0),
    rclcpp::Parameter("test.initial_pose.y", 1.0),
    rclcpp::Parameter("test.initial_pose.yaw", 0.0),
    rclcpp::Parameter("test.initial_pose.std_dev_xy", 1e-3),
    rclcpp::Parameter("test.initial_pose.std_dev_yaw", 1e-3),
    rclcpp::Parameter("test.noise_translation", 1e-9),
    rclcpp::Parameter("test.noise_rotation", 1e-9),
    rclcpp::Parameter("test.noise_translation_to_rotation", 1e-9),
  });
  auto node = std::make_shared<easynav::LocalizerNode>(options);
  auto localizer = std::make_shared<FriendMHAMCLLocalizer>();
  localizer->initialize(node, "test");

  easynav::NavState nav_state;

  // The first odometry only sets the reference
  nav_state.set("odom", make_odom(5.0, 0.0, 0.0, 1));
  localizer->update_rt(nav_state);
  EXPECT_NEAR(localizer->getEstimatedPose().getOrigin().x(), 1.0, 1e-2);

  // The robot advances 2 m in x in the odom frame
  nav_state.set("odom", make_odom(7.0, 0.0, 0.0, 2));
  localizer->update_rt(nav_state);
  EXPECT_NEAR(localizer->getEstimatedPose().getOrigin().x(), 3.0, 1e-2);
  EXPECT_NEAR(localizer->getEstimatedPose().getOrigin().y(), 1.0, 1e-2);

  // The pose in NavState follows
  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  EXPECT_NEAR(robot_pose.pose.pose.position.x, 3.0, 1e-2);
}

TEST_F(MHAMCLLocalizerTest, WithoutOdometryPerceptionItFallsBackToTF)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("test.initial_pose.x", 1.0),
    rclcpp::Parameter("test.initial_pose.std_dev_xy", 1e-3),
    rclcpp::Parameter("test.initial_pose.std_dev_yaw", 1e-3),
  });
  auto node = std::make_shared<easynav::LocalizerNode>(options);
  auto localizer = std::make_shared<FriendMHAMCLLocalizer>();
  localizer->initialize(node, "test");

  easynav::NavState nav_state;

  // No odometry perception (nor a TF): the estimation does not move and nothing breaks
  localizer->update_rt(nav_state);
  EXPECT_NEAR(localizer->getEstimatedPose().getOrigin().x(), 1.0, 1e-2);

  // An odometry perception that never received a message is ignored too
  nav_state.set("odom", nav_msgs::msg::Odometry());
  localizer->update_rt(nav_state);
  EXPECT_NEAR(localizer->getEstimatedPose().getOrigin().x(), 1.0, 1e-2);
}
