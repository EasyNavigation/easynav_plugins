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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_collision_safety_reflex/CollisionSafetyReflex.hpp"
#include "easynav_core/VelocityCommand.hpp"
#include "easynav_common/RobotGeometry.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

class CollisionSafetyReflexTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(CollisionSafetyReflexTest, InitializeTwiceOnSameNodeDoesNotThrow)
{
  // As happens across a cleanup/configure cycle of the node that loads it.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("reflex_reconfigure_node");

  auto reflex1 = std::make_shared<easynav::CollisionSafetyReflex>();
  ASSERT_NO_THROW(reflex1->initialize(node, "collision"));

  auto reflex2 = std::make_shared<easynav::CollisionSafetyReflex>();
  ASSERT_NO_THROW(reflex2->initialize(node, "collision"));
}

TEST_F(CollisionSafetyReflexTest, DoesNotInterveneWithoutCommandedMotion)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("reflex_idle_node");
  auto reflex = std::make_shared<easynav::CollisionSafetyReflex>();
  reflex->initialize(node, "collision");

  easynav::NavState nav_state;
  EXPECT_FALSE(reflex->internal_check_and_mitigate(nav_state));

  easynav::velocity_command::propose(
    nav_state, easynav::VelocitySource::CONTROLLER, geometry_msgs::msg::TwistStamped());
  EXPECT_FALSE(reflex->internal_check_and_mitigate(nav_state));
}

// ─── When it brakes ──────────────────────────────────────────────────────────────────────────

using easynav::VelocitySource;
namespace vc = easynav::velocity_command;

// Robot radius 0.3 (robot_geometry), height 0.5; brake_acc 0.5; commands in the robot frame.
class CollisionSafetyReflexCheckTest : public CollisionSafetyReflexTest
{
protected:
  void SetUp() override
  {
    CollisionSafetyReflexTest::SetUp();
    easynav::RobotGeometryRegistry::getInstance()->set_geometry({0.3, 0.3, 0.5});
  }

  void TearDown() override
  {
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
  }

  void make_reflex(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "reflex_check_node", rclcpp::NodeOptions().parameter_overrides(overrides));
    reflex_ = std::make_shared<easynav::CollisionSafetyReflex>();
    reflex_->initialize(node_, "collision");
  }

  void obstacle_at(double x, double y, double z = 0.2)
  {
    easynav::PointPerception perception;
    perception.data.push_back(pcl::PointXYZ(x, y, z));
    perception.frame_id = "base_link";
    perception.stamp = node_->now();
    perception.valid = true;
    nav_state_.set("scan", perception);
  }

  void command(VelocitySource source, double vx, double wz = 0.0)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = vx;
    cmd.twist.angular.z = wz;
    vc::propose(nav_state_, source, cmd);
  }

  // Whether it brakes this cycle (an OVERRIDE of zero).
  bool brakes()
  {
    const bool intervened = reflex_->internal_check_and_mitigate(nav_state_);
    const auto stop = vc::take(nav_state_, VelocitySource::OVERRIDE);
    EXPECT_EQ(intervened, stop.has_value());
    if (stop) {
      EXPECT_DOUBLE_EQ(stop->twist.linear.x, 0.0);
      EXPECT_DOUBLE_EQ(stop->twist.angular.z, 0.0);
    }
    return intervened;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::CollisionSafetyReflex> reflex_;
  easynav::NavState nav_state_;
};

TEST_F(CollisionSafetyReflexCheckTest, BrakesBeforeAnObstacleInItsPath)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);  // Stops in 0.25 m
  obstacle_at(0.5, 0.0);
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, IgnoresObstaclesBeyondItsStoppingDistance)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(1.5, 0.0);
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, StoppingDistanceGrowsWithSpeedAndShrinksWithBraking)
{
  make_reflex({{"collision.brake_acc", 0.1}});  // 0.5 m/s stops in 1.25 m
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.9, 0.0);
  EXPECT_TRUE(brakes());

  make_reflex({{"collision.brake_acc", 2.0}});  // 0.5 m/s stops in 0.06 m
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.9, 0.0);
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, IgnoresObstaclesBehindWhenMovingForward)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(-0.35, 0.0);
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, BrakesWhenReversingIntoAnObstacle)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, -0.5);
  obstacle_at(-0.35, 0.0);
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, ReversingLooksAsFarAsItsStoppingDistance)
{
  // Behind, beyond robot_radius + safety_margin but within the reverse braking distance.
  make_reflex({{"collision.brake_acc", 0.1}});  // 0.5 m/s stops in 1.25 m
  command(VelocitySource::CONTROLLER, -0.5);
  obstacle_at(-1.2, 0.0);
  EXPECT_TRUE(brakes());

  command(VelocitySource::CONTROLLER, 0.5);  // Moving away from it
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, LooksBeyondTwoMetersWhenItsStoppingDistanceDoes)
{
  // No fixed range limit: what matters is the stopping distance.
  make_reflex({{"collision.brake_acc", 0.1}});  // 0.8 m/s stops in 3.2 m
  command(VelocitySource::CONTROLLER, 0.8);
  obstacle_at(2.8, 0.0);
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, IgnoresObstaclesBesideItsPath)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.5, 0.35);  // Passes 0.35 m from the robot's center: radius 0.3
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, UsesTheRobotRadius)
{
  easynav::RobotGeometryRegistry::getInstance()->set_geometry({0.4, 0.4, 0.5}, {"radius"});
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.5, 0.35);  // Within radius 0.4 now
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, UsesTheRobotHeight)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.5, 0.0, 0.8);  // Above height 0.5
  EXPECT_FALSE(brakes());

  easynav::RobotGeometryRegistry::getInstance()->set_geometry({0.3, 0.3, 1.0}, {"height"});
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.5, 0.0, 0.8);
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, IgnoresPointsBelowZMin)
{
  make_reflex({{"collision.z_min_filter", 0.1}});
  command(VelocitySource::CONTROLLER, 0.5);
  obstacle_at(0.5, 0.0, 0.05);  // E.g. the floor
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, StoppedRobotNeverBrakes)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.0);
  obstacle_at(0.35, 0.0);
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, WithoutPerceptionsItBrakesWhenMoving)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  EXPECT_TRUE(brakes()) << "nothing to check against: fail safe";
  command(VelocitySource::CONTROLLER, -0.2);
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, WithoutPerceptionsItLetsTheRobotRotateInPlace)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.0, 1.0);
  EXPECT_FALSE(brakes()) << "a round robot rotating in place cannot hit anything";
}

TEST_F(CollisionSafetyReflexCheckTest, InvalidPerceptionsCountAsNone)
{
  make_reflex();
  obstacle_at(5.0, 0.0);  // Far away: it would not brake if the data were valid.
  auto perception = nav_state_.get<easynav::PointPerception>("scan");
  perception.valid = false;  // As sensors_node leaves data older than forget_time.
  nav_state_.set("scan", perception);
  command(VelocitySource::CONTROLLER, 0.5);
  EXPECT_TRUE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, OneValidPerceptionIsEnough)
{
  make_reflex();
  obstacle_at(5.0, 0.0);
  easynav::PointPerception stale;
  stale.data.push_back(pcl::PointXYZ(0.35, 0.0, 0.2));  // Would brake if it were used.
  stale.frame_id = "base_link";
  stale.stamp = node_->now();
  stale.valid = false;
  nav_state_.set("old_scan", stale);
  command(VelocitySource::CONTROLLER, 0.5);
  EXPECT_FALSE(brakes()) << "the valid scan sees the path clear; the stale one is ignored";
}

TEST_F(CollisionSafetyReflexCheckTest, AValidScanWithNoPointsIsAFreePath)
{
  make_reflex();
  easynav::PointPerception empty;
  empty.frame_id = "base_link";
  empty.stamp = node_->now();
  empty.valid = true;
  nav_state_.set("scan", empty);
  command(VelocitySource::CONTROLLER, 0.5);
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, ItMovesAgainWhenFreshDataArrives)
{
  make_reflex();
  command(VelocitySource::CONTROLLER, 0.5);
  ASSERT_TRUE(brakes());
  obstacle_at(5.0, 0.0);
  command(VelocitySource::CONTROLLER, 0.5);
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, ChecksTheCommandAboutToBeSent)
{
  // A takeover (a mitigation driving) is what will be sent, not the controller's command.
  make_reflex();
  obstacle_at(0.5, 0.0);
  command(VelocitySource::CONTROLLER, 0.0);
  command(VelocitySource::TAKEOVER, 0.5);
  EXPECT_TRUE(brakes());

  command(VelocitySource::CONTROLLER, 0.5);
  command(VelocitySource::TAKEOVER, -0.1);  // Backing away
  EXPECT_FALSE(brakes());
}

TEST_F(CollisionSafetyReflexCheckTest, BrakesEveryCycleWhileTheDangerLasts)
{
  make_reflex();
  obstacle_at(0.5, 0.0);
  for (int i = 0; i < 3; ++i) {
    command(VelocitySource::CONTROLLER, 0.5);
    EXPECT_TRUE(brakes()) << i;
  }
  obstacle_at(1.5, 0.0);
  command(VelocitySource::CONTROLLER, 0.5);
  EXPECT_FALSE(brakes()) << "the obstacle went away";
}
