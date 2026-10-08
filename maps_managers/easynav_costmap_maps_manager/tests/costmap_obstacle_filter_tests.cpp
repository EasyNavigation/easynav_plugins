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
/// \brief ObstacleFilter "min_height": points below it are floor hits and are not marked.

#include <memory>
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
