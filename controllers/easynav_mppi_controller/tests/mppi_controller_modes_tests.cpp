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
/// \brief The MPPI plugin: its parameters, the commands it writes for each motion model and
/// heading mode, heading_reference, and the robot's footprint.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/RobotGeometry.hpp"
#include "easynav_mppi_controller/MPPIController.hpp"
#include "easynav_system/GoalManager.hpp"

namespace
{

class TestMppi : public easynav::MPPIController
{
public:
  using MPPIController::params_;
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

nav_msgs::msg::Path straight_path(double gx, double gy, double goal_yaw)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const int n = static_cast<int>(std::hypot(gx, gy) / 0.05);
  for (int i = 0; i <= n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = gx * i / n;
    p.pose.position.y = gy * i / n;
    p.pose.orientation = yaw_q(i == n ? goal_yaw : std::atan2(gy, gx));
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace

class MppiModesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
  }

  void TearDown() override
  {
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
  }

  std::shared_ptr<TestMppi> make(std::vector<rclcpp::Parameter> params)
  {
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "mppi_modes_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(params));
    auto mppi = std::make_shared<TestMppi>();
    mppi->initialize(node_, "mppi");
    return mppi;
  }

  /// The robot at (0, 0, yaw) for \p cycles cycles of \p mppi with \p path: the last command.
  static geometry_msgs::msg::Twist drive(
    TestMppi & mppi, const nav_msgs::msg::Path & path, double yaw, easynav::NavState & nav_state,
    int cycles = 15)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.orientation = yaw_q(yaw);
    nav_state.set("robot_pose", odom);
    nav_state.set("path", path);
    for (int i = 0; i < cycles; ++i) {
      mppi.update_rt(nav_state);
    }
    return nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist;
  }

  geometry_msgs::msg::Twist drive(TestMppi & mppi, const nav_msgs::msg::Path & path, double yaw)
  {
    return drive(mppi, path, yaw, nav_state_);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

TEST_F(MppiModesTest, UnknownModesFailToInitialize)
{
  EXPECT_THROW(make({{"mppi.motion_model", "ackermann"}}), std::runtime_error);
  EXPECT_THROW(make({{"mppi.heading_mode", "sideways"}}), std::runtime_error);
  EXPECT_NO_THROW(make({{"mppi.fov", 1.57}})) << "deprecated, no effect";
}

TEST_F(MppiModesTest, TheWeightsAreTheSameForBothMotionModelsUnlessGiven)
{
  const easynav::MPPIParams defaults;
  for (const char * model : {"diff", "omni"}) {
    const auto mppi = make({{"mppi.motion_model", model}});
    EXPECT_DOUBLE_EQ(mppi->params_.path_weight, defaults.path_weight) << model;
    EXPECT_DOUBLE_EQ(mppi->params_.heading_weight, defaults.heading_weight) << model;
    EXPECT_DOUBLE_EQ(
      make({{"mppi.motion_model", model}, {"mppi.path_weight", 3.0}})->params_.path_weight, 3.0)
      << model;
  }
}

TEST_F(MppiModesTest, ADifferentialRobotNeverCommandsSideways)
{
  // Even if told to keep a heading: it faces along the path.
  auto mppi = make({{"mppi.heading_mode", "fixed"}});
  const auto cmd = drive(*mppi, straight_path(0.0, 2.0, M_PI / 2.0), 0.0);
  EXPECT_EQ(cmd.linear.y, 0.0);
  EXPECT_GT(cmd.angular.z, 0.1) << "turning to the path, on its left";
  EXPECT_EQ(
    nav_state_.get<geometry_msgs::msg::TwistStamped>("cmd_vel").header.frame_id,
    easynav::RTTFBuffer::getInstance()->get_tf_info().robot_frame);
}

TEST_F(MppiModesTest, AnOmniRobotKeepingItsHeadingMovesSideways)
{
  auto mppi = make({{"mppi.motion_model", "omni"}, {"mppi.heading_mode", "fixed"}});
  const auto cmd = drive(*mppi, straight_path(0.0, 2.0, 0.0), 0.0);
  EXPECT_GT(cmd.linear.y, 0.1);
  EXPECT_LT(std::abs(cmd.angular.z), 0.1);
}

TEST_F(MppiModesTest, HoldKeepsTheHeadingOfWhenTheGoalArrived)
{
  const auto path = straight_path(3.0, 0.0, 0.0);
  auto hold = make({{"mppi.motion_model", "omni"}, {"mppi.heading_mode", "hold"}});
  EXPECT_LT(std::abs(drive(*hold, path, 0.8).angular.z), 0.1);
  auto along = make({{"mppi.motion_model", "omni"}, {"mppi.heading_mode", "path"}});
  EXPECT_LT(drive(*along, path, 0.8).angular.z, -0.1) << "path mode turns to it";
}

TEST_F(MppiModesTest, HeadingReferenceOverridesTheMode)
{
  const auto path = straight_path(3.0, 0.0, 0.0);
  auto omni = make({{"mppi.motion_model", "omni"}, {"mppi.heading_mode", "path"}});
  nav_state_.set(easynav::kHeadingReferenceKey, M_PI / 2.0);
  EXPECT_GT(drive(*omni, path, 0.0).angular.z, 0.1);

  // A differential robot cannot: it ignores it.
  easynav::NavState fresh;
  fresh.set(easynav::kHeadingReferenceKey, M_PI / 2.0);
  auto diff = make({});
  EXPECT_LT(std::abs(drive(*diff, path, 0.0, fresh).angular.z), 0.1);
}

TEST_F(MppiModesTest, IdleStops)
{
  auto mppi = make({});
  drive(*mppi, straight_path(3.0, 0.0, 0.0), 0.0);
  EXPECT_GT(nav_state_.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x, 0.0);
  nav_state_.set("navigation_state", easynav::GoalManager::State::IDLE);
  mppi->update_rt(nav_state_);
  const auto cmd = nav_state_.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist;
  EXPECT_EQ(cmd.linear.x, 0.0);
  EXPECT_EQ(cmd.linear.y, 0.0);
  EXPECT_EQ(cmd.angular.z, 0.0);
}

TEST_F(MppiModesTest, TheFootprintIsTheRobotsShape)
{
  easynav::RobotGeometry geometry;
  geometry.radius = 0.9;            // A planning margin
  geometry.inscribed_radius = 0.6;
  geometry.footprint = {{0.4, 0.2}, {-0.4, 0.2}, {-0.4, -0.2}, {0.4, -0.2}};
  easynav::RobotGeometryRegistry::getInstance()->set_geometry(geometry, {"footprint"});
  auto mppi = make({});
  EXPECT_EQ(mppi->params_.footprint.size(), 4u);
  EXPECT_DOUBLE_EQ(mppi->params_.robot_radius, std::hypot(0.4, 0.2));
  EXPECT_DOUBLE_EQ(mppi->params_.inscribed_radius, 0.2);

  // Without one: the circle of robot_geometry.radius.
  easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
  auto round = make({});
  EXPECT_TRUE(round->params_.footprint.empty());
  EXPECT_DOUBLE_EQ(round->params_.robot_radius, easynav::RobotGeometry{}.radius);
}
