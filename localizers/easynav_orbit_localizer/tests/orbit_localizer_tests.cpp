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

/// \brief The plugin on a synthetic map: it tracks a moving robot from odometry and scans,
/// keeps the twin of an ambiguous place alive, survives dynamic obstacles, scan dropouts and
/// odometry slips, and relocalizes after a kidnapping.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_orbit_localizer/OrbitLocalizer.hpp"
#include "test_maps.hpp"

using namespace easynav::orbit;  // NOLINT

namespace
{

class TestLocalizer : public OrbitLocalizer
{
public:
  using OrbitLocalizer::update;
  using OrbitLocalizer::update_rt;
};

geometry_msgs::msg::TransformStamped transformOf(
  const Pose & X, const std::string & parent, const std::string & child, const rclcpp::Time & t)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  const Eigen::Quaterniond q(X.linear());
  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.translation.x = X.translation().x();
  msg.transform.translation.y = X.translation().y();
  msg.transform.translation.z = X.translation().z();
  return msg;
}

}  // namespace

class OrbitLocalizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node_ = std::make_shared<easynav::LocalizerNode>();
    tf_info_ = easynav::RTTFBuffer::getInstance()->get_tf_info();
    map_ = std::make_unique<easynav::Costmap2D>(test_maps::twinRooms());
    field_ = std::make_unique<GridDistanceField2D>(*map_);
    nav_state_.set("map.base", *map_);
  }

  void TearDown() override
  {
    localizer_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void start(const Pose & truth, bool symmetry = true, double std_xy = 0.3)
  {
    truth_ = truth;
    X_ob_ = Pose::Identity();   // the odometry starts at its own origin
    node_->declare_parameter("sym.initial_pose.x", truth.translation().x());
    node_->declare_parameter("sym.initial_pose.y", truth.translation().y());
    node_->declare_parameter("sym.initial_pose.yaw", yawOf(truth));
    node_->declare_parameter("sym.initial_pose.std_dev_xy", std_xy);
    node_->declare_parameter("sym.initial_pose.std_dev_yaw", 0.2);
    node_->declare_parameter("sym.symmetry.enabled", symmetry);
    node_->declare_parameter("sym.symmetry.sample_step", 0.5);
    node_->declare_parameter("sym.symmetry.threads", 4);
    node_->declare_parameter("sym.max_range", 12.0);
    node_->declare_parameter("sym.min_winner_time", 0.0);
    node_->declare_parameter("sym.lost_cycles", 3);
    localizer_ = std::make_shared<TestLocalizer>();
    localizer_->initialize(node_, "sym");
    publishOdom();
    localizer_->update_rt(nav_state_);
  }

  void publishOdom()
  {
    easynav::RTTFBuffer::getInstance()->setTransform(
      transformOf(X_ob_, tf_info_.odom_frame, tf_info_.robot_footprint_frame, node_->now()),
      "test", false);
  }

  // The robot moves by delta (body frame): the truth, and the odometry with a scale (1: exact)
  void move(const Vec6 & delta, double odom_scale = 1.0)
  {
    truth_ = truth_ * expSE3(delta);
    X_ob_ = X_ob_ * expSE3(odom_scale * delta);
    publishOdom();
  }

  void scan(bool valid = true, const std::vector<Eigen::Vector3d> & extra = {})
  {
    easynav::PointPerception p;
    p.frame_id = tf_info_.robot_footprint_frame;
    p.stamp = node_->now();
    p.valid = valid;
    p.data.header.frame_id = p.frame_id;
    for (const auto & q : test_maps::scan(*field_, truth_, 360, 12.0, 0.01, ++seed_)) {
      p.data.push_back(pcl::PointXYZ(q.x(), q.y(), 0.1f));
    }
    for (const auto & q : extra) {p.data.push_back(pcl::PointXYZ(q.x(), q.y(), 0.1f));}
    nav_state_.set("scan", p);
    geometry_msgs::msg::Pose tp;   // where the robot really is, for the ORBIT_TRACE diagnostics
    tp.position.x = truth_.translation().x();
    tp.position.y = truth_.translation().y();
    tp.orientation.z = std::sin(0.5 * yawOf(truth_));
    tp.orientation.w = std::cos(0.5 * yawOf(truth_));
    nav_state_.set("debug.truth", tp);
  }

  void cycle(bool valid = true, const std::vector<Eigen::Vector3d> & extra = {})
  {
    scan(valid, extra);
    localizer_->update_rt(nav_state_);
    localizer_->update(nav_state_);
    localizer_->update_rt(nav_state_);
  }

  void waitForGraph()
  {
    cycle();   // the map, and with it the graph's build, are taken in the first update
    for (int i = 0; i < 600 && !localizer_->graphReady(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(localizer_->graphReady());
  }

  double error() const {return test_maps::planarError(localizer_->estimate(), truth_);}
  double yawError() const {return test_maps::yawError(localizer_->estimate(), truth_);}

  std::shared_ptr<easynav::LocalizerNode> node_;
  easynav::TFInfo tf_info_;
  std::unique_ptr<easynav::Costmap2D> map_;
  std::unique_ptr<GridDistanceField2D> field_;
  easynav::NavState nav_state_;
  std::shared_ptr<TestLocalizer> localizer_;
  Pose truth_, X_ob_;
  unsigned seed_ {0};
};

TEST_F(OrbitLocalizerTest, ConvergesAndTracksFromOdometryAndScans)
{
  start(fromXYZYaw(2.0, 3.0, 0.0, 0.0), false);
  cycle();
  EXPECT_LT(error(), 0.1);
  const auto pose = nav_state_.get<nav_msgs::msg::Odometry>("robot_pose");
  EXPECT_EQ(pose.header.frame_id, tf_info_.map_frame);
  EXPECT_NEAR(pose.pose.pose.position.x, 2.0, 0.1);
  // Drive around the room with a slightly short odometry
  for (int k = 0; k < 20; ++k) {
    move((Vec6() << 0.1, 0.0, 0.0, 0.0, 0.0, 0.1).finished(), 0.97);
    cycle();
    EXPECT_LT(error(), 0.1) << "cycle " << k;
    EXPECT_LT(yawError(), 0.05) << "cycle " << k;
  }
  EXPECT_FALSE(localizer_->lost());
  EXPECT_GT(nav_state_.get<double>("localizer.orbit.match_score"), 0.9);
  EXPECT_LT(nav_state_.get<double>("localizer.orbit.cycle_ms"), 50.0);
  // map -> odom is in the TF buffer
  const auto t = easynav::RTTFBuffer::getInstance()->lookupTransform(
    tf_info_.map_frame, tf_info_.odom_frame, tf2::TimePointZero);
  EXPECT_EQ(t.child_frame_id, tf_info_.odom_frame);
}

TEST_F(OrbitLocalizerTest, TheTwinRoomStaysAliveAndTheCorridorResolvesIt)
{
  start(fromXYZYaw(2.0, 3.0, 0.0, 0.0));
  waitForGraph();
  cycle();
  cycle();
  const auto hyps = localizer_->hypotheses();
  ASSERT_GE(hyps.size(), 2u) << "the twin room's hypothesis was not spawned";
  EXPECT_GT(nav_state_.get<int>("localizer.orbit.hypotheses"), 1);
  // The winner is where the robot is, the twin 5 m away, both fit the scans
  EXPECT_LT(error(), 0.1);
  bool twin = false;
  for (const auto & h : hyps) {
    const Pose X = h.X_mo * h.X_ob_at;
    if (std::abs(X.translation().x() - 7.0) < 0.5) {twin = true; EXPECT_GT(h.score, 0.8);}
  }
  EXPECT_TRUE(twin);
  // Through the door into the corridor and east along it: from the left room's door the
  // corridor goes on for 6.5 m, from the right room's for 1.5 m. The twin dies.
  move((Vec6() << 0.0, 1.5, 0.0, 0.0, 0.0, 0.0).finished());   // (2, 4.5): in the corridor
  for (int k = 0; k < 15; ++k) {
    move((Vec6() << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    cycle();
  }
  EXPECT_LT(error(), 0.15);
  EXPECT_NEAR(truth_.translation().x(), 5.0, 0.01);
  for (const auto & h : localizer_->hypotheses()) {
    if (h.id != localizer_->hypotheses().front().id) {EXPECT_LT(h.weight, 0.2);}
  }
}

TEST_F(OrbitLocalizerTest, DynamicObstaclesAndScanDropoutsDoNotLoseIt)
{
  start(fromXYZYaw(2.0, 3.0, 0.0, 0.5), false);
  cycle();
  std::vector<Eigen::Vector3d> people;
  for (int k = 0; k < 120; ++k) {
    const double a = 0.05 * k;
    people.emplace_back(1.2 * std::cos(a), 1.2 * std::sin(a), 0.0);
  }
  for (int k = 0; k < 15; ++k) {
    move((Vec6() << 0.1, 0.0, 0.0, 0.0, 0.0, -0.05).finished());
    cycle(true, people);
    EXPECT_LT(error(), 0.15) << "cycle " << k;
  }
  EXPECT_FALSE(localizer_->lost());
  // The scanner stops for a while: odometry alone, the covariance grows, no lost flag
  const double trace_before = nav_state_.get<double>("localizer.orbit.covariance_trace");
  for (int k = 0; k < 10; ++k) {
    move((Vec6() << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    cycle(false);
  }
  EXPECT_GT(nav_state_.get<double>("localizer.orbit.covariance_trace"), trace_before);
  EXPECT_FALSE(localizer_->lost());
  EXPECT_LT(error(), 0.2);
  cycle();
  cycle();
  EXPECT_LT(error(), 0.1);
}

TEST_F(OrbitLocalizerTest, OdometrySlipIsCorrectedByTheScans)
{
  start(fromXYZYaw(2.0, 3.0, 0.0, 0.0));
  waitForGraph();   // the frozen odometry below ends in a search by the map
  cycle();
  // The wheels spin: the odometry says 0.4 m per cycle, the robot moves 0.1
  for (int k = 0; k < 8; ++k) {
    truth_ = truth_ * expSE3((Vec6() << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    X_ob_ = X_ob_ * expSE3((Vec6() << 0.4, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    publishOdom();
    cycle();
    EXPECT_LT(error(), 0.15) << "cycle " << k;
  }
  // And the odometry freezes while the robot keeps turning: without odometry motion the pose
  // is not estimated again, but the scans stop fitting where it stands, the track is declared
  // lost and the search by the map keeps a hypothesis where the robot really is...
  for (int k = 0; k < 8; ++k) {
    truth_ = truth_ * expSE3((Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.05).finished());
    cycle();
  }
  EXPECT_GT(yawError(), 0.2);
  bool was_lost = false;
  for (int k = 0; k < 10; ++k) {
    cycle();
    was_lost = was_lost || localizer_->lost();
  }
  EXPECT_TRUE(was_lost);
  bool covered = false;
  for (const auto & h : localizer_->hypotheses()) {
    const Pose X = h.X_mo * h.X_ob_at;
    if (test_maps::planarError(X, truth_) < 0.3 && test_maps::yawError(X, truth_) < 0.15) {
      covered = true;
    }
  }
  EXPECT_TRUE(covered) << "no hypothesis at the heading the robot really has";
  // ...which takes over as soon as the odometry moves again
  for (int k = 0; k < 12; ++k) {
    move((Vec6() << 0.15, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    cycle();
  }
  EXPECT_LT(yawError(), 0.1);
  EXPECT_LT(error(), 0.2);
}

TEST_F(OrbitLocalizerTest, KidnappedToTheOtherRoomItKeepsBothAndTheCorridorDecides)
{
  start(fromXYZYaw(2.0, 1.2, 0.0, 1.0));
  waitForGraph();
  cycle();
  cycle();
  EXPECT_LT(error(), 0.1);
  // Carried to the right room, placed facing another way; the odometry notices nothing. The
  // two rooms look the same: honest is to hold a hypothesis in each
  truth_ = fromXYZYaw(7.5, 3.0, 0.0, -2.0);
  for (int k = 0; k < 6; ++k) {
    cycle();
                                       }
  bool covered = false;
  for (const auto & h : localizer_->hypotheses()) {
    if (test_maps::planarError(h.X_mo * h.X_ob_at, truth_) < 0.3) {covered = true;}
  }
  EXPECT_TRUE(covered) << "no hypothesis at the place the robot was carried to";
  // It drives to the door (7, 4) and east along the corridor: 1.5 m to its end from this room
  move((Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, M_PI + 2.0 - 2.0 * M_PI).finished());  // facing west
  cycle();
  for (int k = 0; k < 5; ++k) {
    move((Vec6() << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0).finished());         // (7, 3)
    cycle();
  }
  move((Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, -M_PI / 2.0).finished());   // facing north
  cycle();
  for (int k = 0; k < 10; ++k) {
    move((Vec6() << 0.15, 0.0, 0.0, 0.0, 0.0, 0.0).finished());        // (7, 4.5): the corridor
    cycle();
  }
  move((Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, -M_PI / 2.0).finished());   // heading east
  for (int k = 0; k < 8; ++k) {
    move((Vec6() << 0.15, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    cycle();
  }
  EXPECT_LT(error(), 0.2);
  EXPECT_LT(yawError(), 0.1);
}

TEST_F(OrbitLocalizerTest, KidnappedInAnAsymmetricRoomItRelocalizes)
{
  map_ = std::make_unique<easynav::Costmap2D>(test_maps::lRoom());
  field_ = std::make_unique<GridDistanceField2D>(*map_);
  nav_state_.set("map.base", *map_);
  start(fromXYZYaw(2.0, 2.0, 0.0, 0.5));
  waitForGraph();
  cycle();
  EXPECT_LT(error(), 0.1);
  truth_ = fromXYZYaw(1.5, 6.5, 0.0, -2.0);   // carried up the left arm
  // Put down and turning on the spot: the new place earns the estimate with the evidence the
  // motion brings (a scan seen again from the same place is not more evidence)
  for (int k = 0; k < 16; ++k) {
    move((Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.15).finished());
    cycle();
  }
  EXPECT_LT(error(), 0.3) << "not relocalized after the kidnapping";
  EXPECT_LT(yawError(), 0.1);
  for (int k = 0; k < 5; ++k) {
    move((Vec6() << 0.1, 0.0, 0.0, 0.0, 0.0, 0.1).finished());
    cycle();
  }
  EXPECT_LT(error(), 0.1);
  EXPECT_FALSE(localizer_->lost());
}

TEST_F(OrbitLocalizerTest, GlobalInitWithoutAnInitialPose)
{
  node_->declare_parameter("sym.global_init", true);
  start(fromXYZYaw(1.5, 3.2, 0.0, 2.5));   // the initial pose is ignored
  waitForGraph();
  for (int k = 0; k < 4; ++k) {
    cycle();
                                       }
  // The twin rooms: two hypotheses, one of them right; a walk along the corridor settles it
  EXPECT_GE(localizer_->hypotheses().size(), 2u);
  bool right = false;
  for (const auto & h : localizer_->hypotheses()) {
    if (test_maps::planarError(h.X_mo * h.X_ob_at, truth_) < 0.3) {right = true;}
  }
  EXPECT_TRUE(right);
}

TEST_F(OrbitLocalizerTest, InitialPoseTopicResetsTheHypotheses)
{
  start(fromXYZYaw(2.0, 3.0, 0.0, 0.0), false);
  cycle();
  truth_ = fromXYZYaw(7.5, 1.5, 0.0, 1.5);
  geometry_msgs::msg::PoseWithCovarianceStamped init;
  init.header.frame_id = tf_info_.map_frame;
  init.pose.pose.position.x = 7.3;
  init.pose.pose.position.y = 1.7;
  init.pose.pose.orientation.z = std::sin(0.75);
  init.pose.pose.orientation.w = std::cos(0.75);
  init.pose.covariance[0] = init.pose.covariance[7] = 0.25;
  init.pose.covariance[35] = 0.1;
  auto pub_node = std::make_shared<rclcpp::Node>("initialpose_pub_node");
  auto pub = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_->get_node_base_interface());
  for (int k = 0; k < 100 && pub->get_subscription_count() == 0; ++k) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);
  for (int k = 0; k < 10; ++k) {
    pub->publish(init);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  cycle();
  cycle();
  EXPECT_LT(error(), 0.1);
  EXPECT_LT(yawError(), 0.05);
}
