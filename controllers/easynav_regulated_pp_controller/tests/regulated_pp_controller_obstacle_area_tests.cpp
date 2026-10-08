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
/// \brief The area where obstacles regulate RPP's velocity comes from its own parameters (no
/// longer from the removed "colision_checker.*").

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_regulated_pp_controller/RegulatedPurePursuitController.hpp"

namespace
{

class TestRpp : public easynav::RegulatedPurePursuitController
{
public:
  using easynav::RegulatedPurePursuitController::computeMinObstacleDistance;
};

}  // namespace

class RppObstacleAreaTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Distance from the robot's edge to the nearest obstacle among \p points (robot frame).
  static double distance(
    const std::vector<pcl::PointXYZ> & points, const std::vector<rclcpp::Parameter> & overrides)
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "controller_node", rclcpp::NodeOptions().parameter_overrides(overrides));
    TestRpp rpp;
    rpp.initialize(node, "rpp");

    easynav::NavState nav_state;
    easynav::PointPerception perception;
    for (const auto & p : points) {
      perception.data.push_back(p);
    }
    perception.frame_id = "base_link";
    perception.stamp = node->now();
    perception.valid = true;
    nav_state.set("scan", perception);
    return rpp.computeMinObstacleDistance(nav_state, 3.0);
  }

  static constexpr double kInf = std::numeric_limits<double>::infinity();
};

TEST_F(RppObstacleAreaTest, Defaults)
{
  EXPECT_NEAR(distance({{1.0, 0.0, 0.2}}, {}), 0.65, 1e-6) << "robot_radius 0.35";
  EXPECT_EQ(distance({{1.0, 0.5, 0.2}}, {}), kInf) << "outside 0.35 + 0.1";
  EXPECT_EQ(distance({{1.0, 0.0, 0.8}}, {}), kInf) << "above robot_height 0.5";
}

TEST_F(RppObstacleAreaTest, ItsOwnParameters)
{
  const std::vector<rclcpp::Parameter> params{
    {"rpp.robot_radius", 0.3}, {"rpp.safety_margin", 0.3},
    {"rpp.z_min_filter", 0.1}, {"rpp.robot_height", 1.0}};
  EXPECT_NEAR(distance({{1.0, 0.0, 0.2}}, params), 0.7, 1e-6);
  EXPECT_NEAR(distance({{1.0, 0.5, 0.2}}, params), std::hypot(1.0, 0.5) - 0.3, 1e-6)
    << "within 0.3 + 0.3";
  EXPECT_NEAR(distance({{1.0, 0.0, 0.8}}, params), 0.7, 1e-6) << "below robot_height 1.0";
  EXPECT_EQ(distance({{1.0, 0.0, 0.05}}, params), kInf) << "below z_min_filter";
}

TEST_F(RppObstacleAreaTest, TheRemovedCollisionCheckerParametersDoNotApply)
{
  const std::vector<rclcpp::Parameter> params{
    {"colision_checker.robot_radius", 0.1}, {"colision_checker.robot_height", 2.0}};
  EXPECT_NEAR(distance({{1.0, 0.0, 0.2}}, params), 0.65, 1e-6);
  EXPECT_EQ(distance({{1.0, 0.0, 0.8}}, params), kInf);
}

TEST_F(RppObstacleAreaTest, NoPerceptionsNoObstacle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_node");
  TestRpp rpp;
  rpp.initialize(node, "rpp");
  easynav::NavState nav_state;
  EXPECT_EQ(rpp.computeMinObstacleDistance(nav_state, 3.0), kInf);
}
