// Copyright 2026 Intelligent Robotics Lab
//
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

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_costmap_localizer/AmclRelocalizeMitigation.hpp"
#include "easynav_core/VelocityCommand.hpp"

using easynav_diagnostic_recovery::RecoveryStatus;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void set_robot_pose(easynav::NavState & nav_state, double x, double y, double yaw)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
  odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
  nav_state.set("robot_pose", odom);
}

void set_localization(easynav::NavState & nav_state, double trace, double match = -1.0)
{
  nav_state.set("localizer.amcl.covariance_trace", trace);
  if (match >= 0.0) {
    nav_state.set("localizer.amcl.match_score", match);
  }
}

// The poses published on "initialpose", as AMCLLocalizer gets them
class PoseCatcher
{
public:
  PoseCatcher()
  : node_(std::make_shared<rclcpp::Node>("initial_pose_catcher"))
  {
    sub_ = node_->create_subscription<PoseMsg>(
      "initialpose", 10, [this](PoseMsg::SharedPtr msg) {poses.push_back(*msg);});
    executor_.add_node(node_);
  }

  // Takes what has arrived
  void spin() {executor_.spin_some();}

  // Waits for the publications so far: n poses in all
  bool wait_for(std::size_t n, double timeout = 2.0)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout);
    while (poses.size() < n && std::chrono::steady_clock::now() < end) {
      spin();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    spin();
    return poses.size() == n;
  }

  // Waits until the publisher is matched, so that the first pose is not lost
  void wait_matched(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (node->count_subscribers("initialpose") == 0 && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::vector<PoseMsg> poses;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<PoseMsg>::SharedPtr sub_;
};

}  // namespace

class AmclRelocalizeMitigationTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    tf_info.map_frame = "map";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }

  std::shared_ptr<easynav::AmclRelocalizeMitigation> make_mitigation(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto mit = std::make_shared<easynav::AmclRelocalizeMitigation>();
    mit->initialize(node, name);
    return mit;
  }

  static rclcpp::NodeOptions options(
    const std::string & name, int attempts, double attempt_timeout, double max_offset = 1.0)
  {
    return rclcpp::NodeOptions()
           .append_parameter_override(name + ".attempts", attempts)
           .append_parameter_override(name + ".attempt_timeout", attempt_timeout)
           .append_parameter_override(name + ".max_offset", max_offset);
  }

  // Cycles until the status is not RUNNING (or the time is up), taking the poses published
  static RecoveryStatus run(
    const std::shared_ptr<easynav::AmclRelocalizeMitigation> & mit,
    easynav::NavState & nav_state, double timeout = 5.0, PoseCatcher * catcher = nullptr)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    RecoveryStatus status = RecoveryStatus::RUNNING;
    while (status == RecoveryStatus::RUNNING && std::chrono::steady_clock::now() < end) {
      status = mit->internal_cycle(nav_state);
      if (catcher) {catcher->spin();}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return status;
  }

  static void expect_still(const easynav::NavState & nav_state)
  {
    const auto proposed =
      easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
    ASSERT_TRUE(proposed.has_value());
    EXPECT_DOUBLE_EQ(proposed->twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(proposed->twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(proposed->twist.angular.z, 0.0);
  }
};

TEST_F(AmclRelocalizeMitigationTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto mit = make_mitigation(node, "amcl_mit0");
  EXPECT_TRUE(mit->requires_control());
}

TEST_F(AmclRelocalizeMitigationTestCase, CanHandleOnlyLocalizerAmclErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto mit = make_mitigation(node, "amcl_mit1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "localizer.amcl";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(mit->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus wrong_hardware = matching;
  wrong_hardware.hardware_id = "planner";
  EXPECT_FALSE(mit->can_handle(wrong_hardware));

  diagnostic_msgs::msg::DiagnosticStatus not_an_error = matching;
  not_an_error.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  EXPECT_FALSE(mit->can_handle(not_an_error));
}

TEST_F(AmclRelocalizeMitigationTestCase, DefaultsAreFiveAttemptsOfFiveSecondsWithinAMeter)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_defaults_node");
  auto mit = make_mitigation(node, "amcl_mit_defaults");
  EXPECT_EQ(node->get_parameter("amcl_mit_defaults.attempts").as_int(), 5);
  EXPECT_DOUBLE_EQ(node->get_parameter("amcl_mit_defaults.attempt_timeout").as_double(), 5.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("amcl_mit_defaults.max_offset").as_double(), 1.0);
}

TEST_F(AmclRelocalizeMitigationTestCase, RejectsInvalidParameters)
{
  struct Case {const char * name; int attempts; double timeout; double offset;};
  for (const auto & c : {Case{"no_attempts", 0, 3.0, 1.0}, Case{"negative_attempts", -2, 3.0, 1.0},
      Case{"zero_timeout", 5, 0.0, 1.0}, Case{"negative_timeout", 5, -1.0, 1.0},
      Case{"negative_offset", 5, 3.0, -0.1}})
  {
    const std::string name = std::string("amcl_mit_") + c.name;
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      std::string("test_invalid_") + c.name, options(name, c.attempts, c.timeout, c.offset));
    auto mit = std::make_shared<easynav::AmclRelocalizeMitigation>();
    EXPECT_ANY_THROW(mit->initialize(node, name)) << c.name;
  }
}

TEST_F(AmclRelocalizeMitigationTestCase, FirstAttemptIsTheLostPoseWithRvizCovariance)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_first_node", options("amcl_mit2", 5, 3.0));
  auto mit = make_mitigation(node, "amcl_mit2");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 2.0, -1.5, 0.7);
  set_localization(nav_state, 5.0, 0.1);  // lost

  mit->internal_start(nav_state);
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::RUNNING);
  expect_still(nav_state);

  ASSERT_TRUE(catcher.wait_for(1));
  const auto & pose = catcher.poses[0];
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.y, -1.5);
  EXPECT_NEAR(yaw_of(pose.pose.pose.orientation), 0.7, 1e-9);
  for (std::size_t i = 0; i < 36; ++i) {
    const double expected = (i == 0 || i == 7) ? 0.25 : (i == 35 ? 0.06853891945200942 : 0.0);
    EXPECT_DOUBLE_EQ(pose.pose.covariance[i], expected) << "covariance[" << i << "]";
  }
}

TEST_F(AmclRelocalizeMitigationTestCase, ConvergenceIsNotCheckedRightAfterAReset)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_settle_node", options("amcl_mit3", 5, 0.3));
  auto mit = make_mitigation(node, "amcl_mit3");

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  set_localization(nav_state, 5.0, 0.1);
  mit->internal_start(nav_state);

  // Right after the reset the cloud is still spread: its first scores may look converged
  set_localization(nav_state, 0.57, 0.6);
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::RUNNING);

  // Within the attempt, once it has had time to correct
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::SUCCEEDED);
  expect_still(nav_state);
}

TEST_F(AmclRelocalizeMitigationTestCase, AlreadyConvergedAtStartResetsNothing)
{
  // Selected on a diagnostic evaluated before AMCL converged
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_converged_node", options("amcl_mit15", 5, 3.0));
  auto mit = make_mitigation(node, "amcl_mit15");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 1.0, 2.0, 0.0);
  set_localization(nav_state, 0.05, 0.7);
  nav_state.set("localizer.amcl.quality_ratio", 1.0);

  mit->internal_start(nav_state);
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::SUCCEEDED);
  expect_still(nav_state);
  EXPECT_TRUE(catcher.wait_for(0, 0.3));

  // Next time it is lost: it resets
  set_localization(nav_state, 0.05, 0.2);
  mit->internal_start(nav_state);
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::RUNNING);
  ASSERT_TRUE(catcher.wait_for(1));
}

TEST_F(AmclRelocalizeMitigationTestCase, TheQualityRatioMustAlsoBeSteady)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_ratio_node",
    options("amcl_mit16", 5, 0.05)
    .append_parameter_override("amcl_mit16.quality_ratio_threshold", 0.6));
  auto mit = make_mitigation(node, "amcl_mit16");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  set_localization(nav_state, 0.05, 0.7);
  nav_state.set("localizer.amcl.quality_ratio", 0.3);  // the scores just dropped: kidnapped

  // Not converged at start: it resets, and does not succeed while the ratio stays low
  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
  ASSERT_TRUE(catcher.wait_for(5));

  nav_state.set("localizer.amcl.quality_ratio", 0.3);
  set_localization(nav_state, 5.0, 0.1);
  mit->internal_start(nav_state);
  nav_state.set("localizer.amcl.quality_ratio", 1.0);  // after the reset
  set_localization(nav_state, 0.05, 0.7);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::SUCCEEDED);

  // 0: not checked
  auto node2 = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_ratio_off_node",
    options("amcl_mit17", 5, 0.05)
    .append_parameter_override("amcl_mit17.quality_ratio_threshold", 0.0));
  auto mit2 = make_mitigation(node2, "amcl_mit17");
  nav_state.set("localizer.amcl.quality_ratio", 0.1);
  mit2->internal_start(nav_state);
  EXPECT_EQ(mit2->internal_cycle(nav_state), RecoveryStatus::SUCCEEDED);
}

TEST_F(AmclRelocalizeMitigationTestCase, ConvergenceIsCheckedUntilTheEndOfTheAttempt)
{
  // The default settle time (1 s) is within a 3 s attempt
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_window_node", options("amcl_mit4", 2, 3.0));
  auto mit = make_mitigation(node, "amcl_mit4");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 1.0, 1.0, 0.0);
  set_localization(nav_state, 5.0, 0.1);

  mit->internal_start(nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::RUNNING);

  set_localization(nav_state, 0.2, 0.8);  // converges 1.5 s into the attempt
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::SUCCEEDED);
  ASSERT_TRUE(catcher.wait_for(1));  // no other attempt
}

TEST_F(AmclRelocalizeMitigationTestCase, NextAttemptsAreOffsetUpToMaxOffset)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_offset_node", options("amcl_mit5", 5, 0.05, 0.4));
  auto mit = make_mitigation(node, "amcl_mit5");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 3.0, 4.0, -2.0);
  set_localization(nav_state, 5.0, 0.1);  // never converges

  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
  expect_still(nav_state);

  ASSERT_TRUE(catcher.wait_for(5));
  EXPECT_DOUBLE_EQ(catcher.poses[0].pose.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(catcher.poses[0].pose.pose.position.y, 4.0);
  for (std::size_t i = 1; i < catcher.poses.size(); ++i) {
    const auto & p = catcher.poses[i].pose;
    const double offset = std::hypot(p.pose.position.x - 3.0, p.pose.position.y - 4.0);
    EXPECT_GT(offset, 0.0) << "attempt " << i + 1;
    EXPECT_LE(offset, 0.4 + 1e-9) << "attempt " << i + 1;
    EXPECT_NEAR(yaw_of(p.pose.orientation), -2.0, 1e-9) << "attempt " << i + 1;
    EXPECT_DOUBLE_EQ(p.covariance[0], 0.25);
    EXPECT_DOUBLE_EQ(p.covariance[35], 0.06853891945200942);
  }
}

TEST_F(AmclRelocalizeMitigationTestCase, OffsetsSpreadOverTheWholeDisc)
{
  // Many attempts: every offset within max_offset, some near it and in every quadrant
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_spread_node", options("amcl_mit6", 200, 0.001, 1.0));
  auto mit = make_mitigation(node, "amcl_mit6");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  set_localization(nav_state, 5.0);

  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state, 20.0, &catcher), RecoveryStatus::FAILED);
  ASSERT_TRUE(catcher.wait_for(200, 5.0));

  double farthest = 0.0;
  int quadrants[4] = {0, 0, 0, 0};
  for (std::size_t i = 1; i < catcher.poses.size(); ++i) {
    const auto & p = catcher.poses[i].pose.pose.position;
    const double r = std::hypot(p.x, p.y);
    EXPECT_LE(r, 1.0 + 1e-9);
    farthest = std::max(farthest, r);
    ++quadrants[(p.x >= 0.0 ? 0 : 1) + (p.y >= 0.0 ? 0 : 2)];
  }
  EXPECT_GT(farthest, 0.8);
  for (int q : quadrants) {
    EXPECT_GT(q, 0);
  }
}

TEST_F(AmclRelocalizeMitigationTestCase, ZeroMaxOffsetRepeatsTheLostPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_zero_offset_node", options("amcl_mit7", 3, 0.05, 0.0));
  auto mit = make_mitigation(node, "amcl_mit7");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, -1.0, 2.0, 1.0);
  set_localization(nav_state, 5.0);

  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
  ASSERT_TRUE(catcher.wait_for(3));
  for (const auto & pose : catcher.poses) {
    EXPECT_DOUBLE_EQ(pose.pose.pose.position.x, -1.0);
    EXPECT_DOUBLE_EQ(pose.pose.pose.position.y, 2.0);
  }
}

TEST_F(AmclRelocalizeMitigationTestCase, OneAttemptOnlyTriesTheLostPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_one_node", options("amcl_mit8", 1, 0.05));
  auto mit = make_mitigation(node, "amcl_mit8");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.5, 0.5, 0.0);
  set_localization(nav_state, 5.0);

  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
  ASSERT_TRUE(catcher.wait_for(1));
}

TEST_F(AmclRelocalizeMitigationTestCase, SucceedsOnALaterAttempt)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_later_node", options("amcl_mit9", 5, 0.1));
  auto mit = make_mitigation(node, "amcl_mit9");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  set_localization(nav_state, 5.0, 0.1);

  mit->internal_start(nav_state);
  // The first attempt times out: the second one starts
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::RUNNING);
  ASSERT_TRUE(catcher.wait_for(2));

  set_localization(nav_state, 0.2, 0.8);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::SUCCEEDED);
  EXPECT_TRUE(catcher.wait_for(2, 0.2));  // no third attempt
}

TEST_F(AmclRelocalizeMitigationTestCase, ACompactCloudMustAlsoMatchTheMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_match_node",
    options("amcl_mit10", 5, 0.05)
    .append_parameter_override("amcl_mit10.covariance_threshold", 0.5)
    .append_parameter_override("amcl_mit10.match_threshold", 0.3));
  auto mit = make_mitigation(node, "amcl_mit10");

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  set_localization(nav_state, 0.1, 0.1);  // compact, but lost

  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);

  set_localization(nav_state, 5.0, 0.1);
  mit->internal_start(nav_state);
  set_localization(nav_state, 0.1, 0.5);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::SUCCEEDED);

  set_localization(nav_state, 5.0, 0.1);
  mit->internal_start(nav_state);
  set_localization(nav_state, 0.8, 0.9);  // matches, but spread
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
}

TEST_F(AmclRelocalizeMitigationTestCase, WithoutAMatchScoreOnlyTheCovarianceCounts)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_no_match_node",
    options("amcl_mit11", 5, 0.05)
    .append_parameter_override("amcl_mit11.covariance_threshold", 0.5)
    .append_parameter_override("amcl_mit11.match_threshold", 0.3));
  auto mit = make_mitigation(node, "amcl_mit11");

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  set_localization(nav_state, 5.0);
  mit->internal_start(nav_state);
  set_localization(nav_state, 0.1);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::SUCCEEDED);
}

TEST_F(AmclRelocalizeMitigationTestCase, WithoutACovarianceItDoesNotConverge)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_no_trace_node", options("amcl_mit12", 2, 0.05));
  auto mit = make_mitigation(node, "amcl_mit12");

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 0.0, 0.0, 0.0);
  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
}

TEST_F(AmclRelocalizeMitigationTestCase, WithoutARobotPoseItFailsWithoutResetting)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_no_pose_node", options("amcl_mit13", 5, 3.0));
  auto mit = make_mitigation(node, "amcl_mit13");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_localization(nav_state, 5.0);
  mit->internal_start(nav_state);
  EXPECT_EQ(mit->internal_cycle(nav_state), RecoveryStatus::FAILED);
  expect_still(nav_state);
  EXPECT_TRUE(catcher.wait_for(0, 0.2));
}

TEST_F(AmclRelocalizeMitigationTestCase, EachRecoveryStartsFromTheRobotPoseThen)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_restart_node", options("amcl_mit14", 2, 0.05, 0.0));
  auto mit = make_mitigation(node, "amcl_mit14");
  PoseCatcher catcher;
  catcher.wait_matched(node);

  easynav::NavState nav_state;
  set_robot_pose(nav_state, 1.0, 0.0, 0.0);
  set_localization(nav_state, 5.0);
  mit->internal_start(nav_state);
  set_robot_pose(nav_state, 9.0, 9.0, 0.0);  // the estimate moves during the recovery: ignored
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
  mit->internal_stop(nav_state);
  ASSERT_TRUE(catcher.wait_for(2));
  EXPECT_DOUBLE_EQ(catcher.poses[1].pose.pose.position.x, 1.0);

  // A new recovery: from where the robot is now, all its attempts again
  set_robot_pose(nav_state, -3.0, 2.0, 0.0);
  mit->internal_start(nav_state);
  EXPECT_EQ(run(mit, nav_state), RecoveryStatus::FAILED);
  ASSERT_TRUE(catcher.wait_for(4));
  EXPECT_DOUBLE_EQ(catcher.poses[2].pose.pose.position.x, -3.0);
  EXPECT_DOUBLE_EQ(catcher.poses[3].pose.pose.position.y, 2.0);
}
