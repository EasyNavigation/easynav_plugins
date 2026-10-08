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

/// \file
/// \brief SimplePlanner's robot radius is the robot's ("robot_geometry"); its own
/// "robot_radius" is deprecated but still applies.

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/testing/LogCapture.hpp"
#include "easynav_simple_planner/SimplePlanner.hpp"

namespace
{

class TestSimplePlanner : public easynav::SimplePlanner
{
public:
  double robot_radius() const {return robot_radius_;}
};

}  // namespace

class SimplePlannerGeometryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    registry()->set_geometry(easynav::RobotGeometry{});
  }

  static easynav::RobotGeometryRegistry * registry()
  {
    return easynav::RobotGeometryRegistry::getInstance();
  }

  static double robot_radius(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "planner_node", rclcpp::NodeOptions().parameter_overrides(overrides));
    TestSimplePlanner planner;
    planner.initialize(node, "simple");
    return planner.robot_radius();
  }
};

TEST_F(SimplePlannerGeometryTest, UsesTheRobotRadius)
{
  registry()->set_geometry({0.45, 0.45, 1.0}, {"radius"});
  EXPECT_DOUBLE_EQ(robot_radius(), 0.45);
}

TEST_F(SimplePlannerGeometryTest, DefaultsToTheRobotGeometry)
{
  EXPECT_DOUBLE_EQ(robot_radius(), easynav::RobotGeometry{}.radius);
}

TEST_F(SimplePlannerGeometryTest, DeprecatedRobotRadiusStillApplies)
{
  EXPECT_DOUBLE_EQ(robot_radius({{"simple.robot_radius", 0.2}}), 0.2);
}

TEST_F(SimplePlannerGeometryTest, RobotGeometryTakesPrecedence)
{
  registry()->set_geometry({0.45, 0.45, 1.0}, {"radius"});
  EXPECT_DOUBLE_EQ(robot_radius({{"simple.robot_radius", 0.2}}), 0.45);
}

TEST_F(SimplePlannerGeometryTest, OnlyItsOwnNameIsDeprecated)
{
  EXPECT_DOUBLE_EQ(
    robot_radius({{"robot_radius", 0.9}, {"other.robot_radius", 0.8}}),
    easynav::RobotGeometry{}.radius);
}

TEST_F(SimplePlannerGeometryTest, WarnsAboutTheDeprecatedRobotRadius)
{
  {
    easynav::testing::LogCapture log;
    robot_radius({{"simple.robot_radius", 0.2}});
    EXPECT_EQ(
      log.count(
        {"'simple.robot_radius' is deprecated: configure", "system_node.robot_geometry.radius"}),
      1u);
  }
  {
    registry()->set_geometry({0.45, 0.45, 1.0}, {"radius"});
    easynav::testing::LogCapture log;
    robot_radius({{"simple.robot_radius", 0.2}});
    EXPECT_EQ(log.count({"'simple.robot_radius' is deprecated and ignored"}), 1u);
  }
  {
    easynav::testing::LogCapture log;
    robot_radius();
    EXPECT_EQ(log.count({"deprecated"}), 0u);
  }
}
