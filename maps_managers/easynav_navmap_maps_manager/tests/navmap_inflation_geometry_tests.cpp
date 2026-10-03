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
/// \brief The inflation filter's inscribed radius is the robot's ("robot_geometry"); its own
/// "inscribed_radius" is deprecated but still applies.

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/testing/LogCapture.hpp"
#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"

namespace
{

class TestInflationFilter : public easynav::navmap::InflationFilter
{
public:
  double inscribed_radius() const {return inscribed_radius_;}
};

}  // namespace

class NavmapInflationGeometryTest : public ::testing::Test
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

  static double inscribed_radius(
    const std::vector<rclcpp::Parameter> & overrides = {},
    const rclcpp::NodeOptions & base = rclcpp::NodeOptions())
  {
    auto options = base;
    for (const auto & parameter : overrides) {
      options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    }
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("maps_manager_node", options);
    TestInflationFilter filter;
    filter.initialize(node, "navmap.inflation");
    return filter.inscribed_radius();
  }
};

TEST_F(NavmapInflationGeometryTest, UsesTheRobotInscribedRadius)
{
  registry()->set_geometry({0.5, 0.4, 1.0}, {"radius", "inscribed_radius"});
  EXPECT_DOUBLE_EQ(inscribed_radius(), 0.4);
}

TEST_F(NavmapInflationGeometryTest, DefaultsToTheRobotGeometry)
{
  EXPECT_DOUBLE_EQ(inscribed_radius(), easynav::RobotGeometry{}.inscribed_radius);
}

TEST_F(NavmapInflationGeometryTest, DeprecatedInscribedRadiusStillApplies)
{
  EXPECT_DOUBLE_EQ(inscribed_radius({{"navmap.inflation.inscribed_radius", 0.25}}), 0.25);
}

TEST_F(NavmapInflationGeometryTest, RobotGeometryTakesPrecedence)
{
  registry()->set_geometry({0.5, 0.4, 1.0}, {"radius", "inscribed_radius"});
  EXPECT_DOUBLE_EQ(inscribed_radius({{"navmap.inflation.inscribed_radius", 0.25}}), 0.4);
}

TEST_F(NavmapInflationGeometryTest, OnlyTheRadiusConfiguredKeepsTheDeprecatedOne)
{
  // A robot_geometry.radius alone does not configure the inscribed radius.
  registry()->set_geometry({0.5, 0.5, 1.0}, {"radius"});
  EXPECT_DOUBLE_EQ(inscribed_radius({{"navmap.inflation.inscribed_radius", 0.25}}), 0.25);
  EXPECT_DOUBLE_EQ(inscribed_radius(), 0.5);
}

TEST_F(NavmapInflationGeometryTest, WarnsAboutTheDeprecatedInscribedRadius)
{
  {
    easynav::testing::LogCapture log;
    inscribed_radius({{"navmap.inflation.inscribed_radius", 0.25}});
    EXPECT_EQ(
      log.count(
        {"'navmap.inflation.inscribed_radius' is deprecated: configure",
          "system_node.robot_geometry.inscribed_radius"}), 1u);
  }
  {
    registry()->set_geometry({0.5, 0.4, 1.0}, {"inscribed_radius"});
    easynav::testing::LogCapture log;
    inscribed_radius({{"navmap.inflation.inscribed_radius", 0.25}});
    EXPECT_EQ(
      log.count(
        {"'navmap.inflation.inscribed_radius' is deprecated and ignored", "takes precedence"}),
      1u);
  }
  {
    easynav::testing::LogCapture log;
    inscribed_radius();
    EXPECT_EQ(log.count({"deprecated"}), 0u);
  }
}
