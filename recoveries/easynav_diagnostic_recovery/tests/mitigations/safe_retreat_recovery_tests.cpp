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

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_diagnostic_recovery/mitigations/SafeRetreatRecovery.hpp"
#include "easynav_core/VelocityCommand.hpp"

class SafeRetreatRecoveryTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }

  static easynav::PointPerception make_obstacle_at(double x, double y)
  {
    easynav::PointPerception perception;
    perception.frame_id = "base_link";
    perception.stamp = rclcpp::Time(0);
    perception.valid = true;
    perception.data.points.resize(1);
    perception.data.points[0].x = x;
    perception.data.points[0].y = y;
    perception.data.points[0].z = 0.0;
    return perception;
  }

  std::shared_ptr<easynav::SafeRetreatRecovery> make_recovery(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto rec = std::make_shared<easynav::SafeRetreatRecovery>();
    rec->initialize(node, name);
    return rec;
  }
};

TEST_F(SafeRetreatRecoveryTestCase, RequiresControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rc_node");
  auto rec = make_recovery(node, "retreat0");
  EXPECT_TRUE(rec->requires_control());
}

TEST_F(SafeRetreatRecoveryTestCase, CanHandleOnlyObstacleProximityErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ch_node");
  auto rec = make_recovery(node, "retreat1");

  diagnostic_msgs::msg::DiagnosticStatus matching;
  matching.hardware_id = "obstacle_proximity";
  matching.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  EXPECT_TRUE(rec->can_handle(matching));

  diagnostic_msgs::msg::DiagnosticStatus wrong_hardware = matching;
  wrong_hardware.hardware_id = "planner";
  EXPECT_FALSE(rec->can_handle(wrong_hardware));

  diagnostic_msgs::msg::DiagnosticStatus not_an_error = matching;
  not_an_error.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  EXPECT_FALSE(rec->can_handle(not_an_error));
}

TEST_F(SafeRetreatRecoveryTestCase, RetreatsBackwardWhileObstacleAheadAndClose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_retreat_node");
  auto rec = make_recovery(node, "retreat2");

  easynav::NavState nav_state;
  nav_state.set("scan", make_obstacle_at(0.2, 0.0));  // ahead, well within default safe_distance

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav_diagnostic_recovery::RecoveryStatus::RUNNING);
  // Movement mitigations propose their command; ControllerNode publishes it.
  const auto proposed =
    easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
  ASSERT_TRUE(proposed.has_value());
  const auto & cmd = *proposed;
  EXPECT_LT(cmd.twist.linear.x, 0.0);
}

TEST_F(SafeRetreatRecoveryTestCase, SucceedsOnceFarEnough)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_far_node");
  auto rec = make_recovery(node, "retreat3");

  easynav::NavState nav_state;
  nav_state.set("scan", make_obstacle_at(5.0, 0.0));  // far away

  auto status = rec->internal_cycle(nav_state);

  EXPECT_EQ(status, easynav_diagnostic_recovery::RecoveryStatus::SUCCEEDED);
  // Movement mitigations propose their command; ControllerNode publishes it.
  const auto proposed =
    easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
  ASSERT_TRUE(proposed.has_value());
  const auto & cmd = *proposed;
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

TEST_F(SafeRetreatRecoveryTestCase, FailsStoppedWithoutPerception)
{
  // Blind: the obstacle may still be there; another mitigation must take over.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_none_node");
  auto rec = make_recovery(node, "retreat4");

  easynav::NavState nav_state;  // no perception at all
  EXPECT_EQ(rec->internal_cycle(nav_state), easynav_diagnostic_recovery::RecoveryStatus::FAILED);
  auto stop = easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
  ASSERT_TRUE(stop.has_value());
  EXPECT_DOUBLE_EQ(stop->twist.linear.x, 0.0);

  easynav::PointPerception no_data_yet;
  nav_state.set("scan", no_data_yet);
  EXPECT_EQ(rec->internal_cycle(nav_state), easynav_diagnostic_recovery::RecoveryStatus::FAILED);
}

TEST_F(SafeRetreatRecoveryTestCase, SucceedsWhenTheSceneIsClear)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_clear_node");
  auto rec = make_recovery(node, "retreat5");

  easynav::NavState nav_state;
  easynav::PointPerception empty_scan;
  empty_scan.frame_id = "base_link";
  empty_scan.valid = true;  // Perceiving, nothing in range
  nav_state.set("scan", empty_scan);
  EXPECT_EQ(
    rec->internal_cycle(nav_state), easynav_diagnostic_recovery::RecoveryStatus::SUCCEEDED);
}

TEST_F(SafeRetreatRecoveryTestCase, IgnoresTheGround)
{
  // A 3D lidar sees the ground right in front of the robot: not an obstacle to retreat from.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_ground_node",
    rclcpp::NodeOptions().append_parameter_override("retreat6.z_min_filter", 0.1));
  auto rec = make_recovery(node, "retreat6");

  easynav::NavState nav_state;
  auto ground = make_obstacle_at(0.2, 0.0);  // z = 0.0
  nav_state.set("scan", ground);
  EXPECT_EQ(
    rec->internal_cycle(nav_state), easynav_diagnostic_recovery::RecoveryStatus::SUCCEEDED);
}

// ─── Direction ──────────────────────────────────────────────────────────────────────────────

namespace
{

// Robot radius 0.3 (robot_geometry default); min_clearance 0.05.
easynav::PointPerception obstacles(std::vector<std::pair<double, double>> points)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  for (const auto & [x, y] : points) {
    perception.data.points.emplace_back(x, y, 0.0);
  }
  return perception;
}

double proposed_vx(easynav::NavState & nav_state)
{
  const auto cmd = easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
  return cmd ? cmd->twist.linear.x : std::nan("");
}

}  // namespace

class SafeRetreatDirectionTest : public SafeRetreatRecoveryTestCase
{
protected:
  void SetUp() override
  {
    SafeRetreatRecoveryTestCase::SetUp();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("retreat_direction_node");
    rec_ = make_recovery(node_, "retreat");
    rec_->internal_start(nav_state_);
  }

  easynav_diagnostic_recovery::RecoveryStatus cycle(std::vector<std::pair<double, double>> points)
  {
    nav_state_.set("scan", obstacles(points));
    return rec_->internal_cycle(nav_state_);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::SafeRetreatRecovery> rec_;
  easynav::NavState nav_state_;
};

using easynav_diagnostic_recovery::RecoveryStatus;

TEST_F(SafeRetreatDirectionTest, AnObstacleBehindMakesItMoveForward)
{
  EXPECT_EQ(cycle({{-0.35, 0.0}}), RecoveryStatus::RUNNING);
  EXPECT_GT(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, AnObstacleBesideMakesItMoveForward)
{
  // The case seen in simulation: 0.36 m away, to the right and slightly behind.
  EXPECT_EQ(cycle({{0.36 * std::cos(-1.71), 0.36 * std::sin(-1.71)}}), RecoveryStatus::RUNNING);
  EXPECT_GT(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, AnObstacleBesideAndAWallAheadMakeItMoveBackward)
{
  EXPECT_EQ(cycle({{0.0, -0.36}, {0.32, 0.0}}), RecoveryStatus::RUNNING);
  EXPECT_LT(proposed_vx(nav_state_), 0.0) << "forward is not clear, backward is";
}

TEST_F(SafeRetreatDirectionTest, AnObstacleAheadWithTheWayBackBlockedFails)
{
  EXPECT_EQ(cycle({{0.32, 0.0}, {-0.34, 0.0}}), RecoveryStatus::FAILED);
  EXPECT_DOUBLE_EQ(proposed_vx(nav_state_), 0.0) << "stopped";
}

TEST_F(SafeRetreatDirectionTest, AnObstacleBehindWithTheWayForwardBlockedFails)
{
  // Not beside: moving backward would approach the nearest obstacle.
  EXPECT_EQ(cycle({{-0.32, 0.0}, {0.34, 0.0}}), RecoveryStatus::FAILED);
  EXPECT_DOUBLE_EQ(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, PointsOutsideTheCorridorDoNotBlockTheWay)
{
  // Beside the robot, ahead: not in the corridor it sweeps moving forward.
  EXPECT_EQ(cycle({{-0.35, 0.0}, {0.32, 0.31}}), RecoveryStatus::RUNNING);
  EXPECT_GT(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, TheDirectionIsKeptDuringTheEpisode)
{
  ASSERT_EQ(cycle({{-0.35, 0.0}}), RecoveryStatus::RUNNING);
  ASSERT_GT(proposed_vx(nav_state_), 0.0);
  // The nearest obstacle is now ahead-ish but outside the corridor: it keeps moving forward.
  EXPECT_EQ(cycle({{0.1, 0.4}, {-0.45, 0.0}}), RecoveryStatus::RUNNING);
  EXPECT_GT(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, ItStopsWhenTheWayGetsBlocked)
{
  ASSERT_EQ(cycle({{-0.35, 0.0}}), RecoveryStatus::RUNNING);
  EXPECT_EQ(cycle({{-0.4, 0.0}, {0.33, 0.0}}), RecoveryStatus::FAILED);
  EXPECT_DOUBLE_EQ(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, ANewEpisodeChoosesAgain)
{
  ASSERT_EQ(cycle({{-0.35, 0.0}}), RecoveryStatus::RUNNING);
  ASSERT_GT(proposed_vx(nav_state_), 0.0);
  rec_->internal_start(nav_state_);
  EXPECT_EQ(cycle({{0.35, 0.0}}), RecoveryStatus::RUNNING);
  EXPECT_LT(proposed_vx(nav_state_), 0.0);
}

TEST_F(SafeRetreatDirectionTest, ItSucceedsOnceFarEnough)
{
  ASSERT_EQ(cycle({{-0.35, 0.0}}), RecoveryStatus::RUNNING);
  EXPECT_EQ(cycle({{-0.65, 0.0}}), RecoveryStatus::SUCCEEDED);
  EXPECT_DOUBLE_EQ(proposed_vx(nav_state_), 0.0);
}
