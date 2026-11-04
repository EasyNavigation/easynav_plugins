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
/// \brief ObstacleFilter: "min_height" (points below it are floor hits and are not marked) and
/// "update_distance" (only the square around the robot updates the map).

#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_maps_manager/filters/ObstacleFilter.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "nav_msgs/msg/odometry.hpp"

class CostmapObstacleFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Runs the filter on a free 2 x 2 m map (0.1 m cells) with one point per given height,
  // each at its own (x, y): (0.25 + 0.5 i, 0.25 + 0.5 i). Returns whether each was marked.
  static std::vector<bool> marked(
    const std::vector<float> & heights, const std::vector<rclcpp::Parameter> & overrides = {},
    double * min_height = nullptr)
  {
    static int count = 0;
    rclcpp::NodeOptions options;
    for (const auto & parameter : overrides) {
      options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    }
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "maps_manager_node_" + std::to_string(count++), options);

    easynav::ObstacleFilter filter;
    filter.initialize(node, "costmap.obstacles");
    if (min_height) {
      *min_height = node->get_parameter("costmap.obstacles.min_height").as_double();
    }

    easynav::NavState nav_state;
    nav_state.set("map", easynav::Costmap2D(20, 20, 0.1, 0.0, 0.0, easynav::FREE_SPACE));

    easynav::PointPerception perception;
    perception.frame_id = "map";
    perception.stamp = node->now();
    perception.valid = true;
    for (std::size_t i = 0; i < heights.size(); ++i) {
      const float xy = 0.25f + 0.5f * static_cast<float>(i);
      perception.data.push_back(pcl::PointXYZ(xy, xy, heights[i]));
    }
    nav_state.set("scan", perception);

    filter.update(nav_state);

    const auto & map = nav_state.get<easynav::Costmap2D>("map");
    std::vector<bool> result;
    for (std::size_t i = 0; i < heights.size(); ++i) {
      const double xy = 0.25 + 0.5 * static_cast<double>(i);
      unsigned int mx, my;
      EXPECT_TRUE(map.worldToMap(xy, xy, mx, my));
      result.push_back(map.getCost(mx, my) == easynav::LETHAL_OBSTACLE);
    }
    return result;
  }

  // Runs the filter on a free 40 x 40 m map (0.1 m cells, origin at -20, -20) with the given
  // points, and the robot at `robot` (no robot_pose if unset). Returns whether each was marked.
  static std::vector<bool> marked_around(
    const std::vector<pcl::PointXYZ> & points,
    std::optional<std::pair<double, double>> robot,
    const std::vector<rclcpp::Parameter> & overrides = {}, double * update_distance = nullptr)
  {
    static int count = 0;
    rclcpp::NodeOptions options;
    for (const auto & parameter : overrides) {
      options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    }
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "maps_manager_around_node_" + std::to_string(count++), options);

    easynav::ObstacleFilter filter;
    filter.initialize(node, "costmap.obstacles");
    if (update_distance) {
      *update_distance = node->get_parameter("costmap.obstacles.update_distance").as_double();
    }

    easynav::NavState nav_state;
    nav_state.set("map", easynav::Costmap2D(400, 400, 0.1, -20.0, -20.0, easynav::FREE_SPACE));
    if (robot) {
      nav_msgs::msg::Odometry pose;
      pose.header.frame_id = "map";
      pose.pose.pose.position.x = robot->first;
      pose.pose.pose.position.y = robot->second;
      pose.pose.pose.orientation.w = 1.0;
      nav_state.set("robot_pose", pose);
    }

    easynav::PointPerception perception;
    perception.frame_id = "map";
    perception.stamp = node->now();
    perception.valid = true;
    for (const auto & point : points) {
      perception.data.push_back(point);
    }
    nav_state.set("scan", perception);

    filter.update(nav_state);

    const auto & map = nav_state.get<easynav::Costmap2D>("map");
    std::vector<bool> result;
    for (const auto & point : points) {
      unsigned int mx, my;
      EXPECT_TRUE(map.worldToMap(point.x, point.y, mx, my));
      result.push_back(map.getCost(mx, my) == easynav::LETHAL_OBSTACLE);
    }
    return result;
  }
};

TEST_F(CostmapObstacleFilterTest, DefaultIgnoresPointsBelowTenCentimeters)
{
  double min_height = 0.0;
  const auto result = marked({0.05f, 0.08f, 0.20f}, {}, &min_height);
  EXPECT_DOUBLE_EQ(min_height, 0.1);
  EXPECT_EQ(result, (std::vector<bool>{false, false, true}));
}

TEST_F(CostmapObstacleFilterTest, LowerMinHeightKeepsALowLaser)
{
  // A laser 8 cm above the floor (e.g. TIAGo's base laser): kept with min_height 0.07
  double min_height = 0.0;
  const auto result = marked(
    {0.05f, 0.08f, 0.20f}, {rclcpp::Parameter("costmap.obstacles.min_height", 0.07)},
    &min_height);
  EXPECT_DOUBLE_EQ(min_height, 0.07);
  EXPECT_EQ(result, (std::vector<bool>{false, true, true}));
}

TEST_F(CostmapObstacleFilterTest, HigherMinHeightIgnoresMore)
{
  const auto result = marked(
    {0.05f, 0.08f, 0.20f}, {rclcpp::Parameter("costmap.obstacles.min_height", 0.3)});
  EXPECT_EQ(result, (std::vector<bool>{false, false, false}));
}

TEST_F(CostmapObstacleFilterTest, NegativeMinHeightKeepsEverything)
{
  // Points below the map plane (e.g. a ramp down): kept only when allowed explicitly
  const auto result = marked(
    {-0.2f, 0.0f, 0.08f}, {rclcpp::Parameter("costmap.obstacles.min_height", -0.5)});
  EXPECT_EQ(result, (std::vector<bool>{true, true, true}));
}


namespace
{

/// A point 0.3 m high, an obstacle for the default min_height.
pcl::PointXYZ P(float x, float y)
{
  return pcl::PointXYZ(x, y, 0.3f);
}

}  // namespace

TEST_F(CostmapObstacleFilterTest, ByDefaultOnlyTheFiveMetersAroundTheRobotUpdate)
{
  double update_distance = 0.0;
  // Robot at (2, 3): along x, along y, on the diagonal of the square, and outside it.
  const auto result = marked_around(
    {P(3.0f, 3.0f), P(6.9f, 3.0f), P(7.2f, 3.0f), P(2.0f, -1.9f), P(2.0f, -2.2f),
      P(6.5f, 7.5f), P(-2.5f, -1.5f), P(15.0f, 15.0f)},
    std::make_pair(2.0, 3.0), {}, &update_distance);
  EXPECT_DOUBLE_EQ(update_distance, 5.0);
  EXPECT_EQ(result, (std::vector<bool>{true, true, false, true, false, true, true, false}));
}

TEST_F(CostmapObstacleFilterTest, UpdateDistanceIsConfigurable)
{
  const std::vector<pcl::PointXYZ> points{P(0.5f, 0.0f), P(1.4f, 0.0f), P(1.6f, 0.0f),
    P(0.0f, -1.6f), P(12.0f, 0.0f)};
  EXPECT_EQ(
    marked_around(
      points, std::make_pair(0.0, 0.0),
      {rclcpp::Parameter("costmap.obstacles.update_distance", 1.5)}),
    (std::vector<bool>{true, true, false, false, false}));
  EXPECT_EQ(
    marked_around(
      points, std::make_pair(0.0, 0.0),
      {rclcpp::Parameter("costmap.obstacles.update_distance", 15.0)}),
    (std::vector<bool>{true, true, true, true, true}));
}

TEST_F(CostmapObstacleFilterTest, ZeroOrNegativeUpdateDistanceKeepsEveryPoint)
{
  const std::vector<pcl::PointXYZ> points{P(0.5f, 0.0f), P(12.0f, -15.0f)};
  for (const double distance : {0.0, -1.0}) {
    EXPECT_EQ(
      marked_around(
        points, std::make_pair(0.0, 0.0),
        {rclcpp::Parameter("costmap.obstacles.update_distance", distance)}),
      (std::vector<bool>{true, true})) << "update_distance " << distance;
  }
}

TEST_F(CostmapObstacleFilterTest, WithoutRobotPoseEveryPointUpdates)
{
  // No localization yet: nothing to center the square on.
  EXPECT_EQ(
    marked_around({P(0.5f, 0.0f), P(12.0f, -15.0f)}, std::nullopt),
    (std::vector<bool>{true, true}));
}

TEST_F(CostmapObstacleFilterTest, TheSquareMovesWithTheRobot)
{
  const std::vector<pcl::PointXYZ> points{P(-10.0f, -10.0f), P(10.0f, 10.0f)};
  EXPECT_EQ(
    marked_around(points, std::make_pair(-9.0, -11.0)), (std::vector<bool>{true, false}));
  EXPECT_EQ(
    marked_around(points, std::make_pair(12.0, 8.5)), (std::vector<bool>{false, true}));
}

TEST_F(CostmapObstacleFilterTest, MinHeightStillAppliesInsideTheSquare)
{
  // A floor hit next to the robot is not an obstacle.
  EXPECT_EQ(
    marked_around(
      {pcl::PointXYZ(1.0f, 0.0f, 0.05f), pcl::PointXYZ(-1.0f, 0.0f, 0.20f)},
      std::make_pair(0.0, 0.0)),
    (std::vector<bool>{false, true}));
}
