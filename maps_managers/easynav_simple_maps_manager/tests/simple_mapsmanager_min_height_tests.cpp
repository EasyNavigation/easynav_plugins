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
/// \brief SimpleMapsManager "min_height": points below it are floor hits and are not marked.

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_simple_maps_manager/SimpleMapsManager.hpp"

class SimpleMapsManagerMinHeightTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::RTTFBuffer::getInstance()->set_tf_info(easynav::TFInfo());
  }

  // Updates a free 3 x 3 m map (0.1 m cells) with one point per given height, each at its own
  // (x, y): (-1.25 + 0.5 i, -1.25 + 0.5 i). Returns whether each cell was marked.
  static std::vector<bool> marked(
    const std::vector<float> & heights, double min_height_param = -1.0,
    double * min_height = nullptr)
  {
    static int count = 0;
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "maps_manager_node_" + std::to_string(count++));
    if (min_height_param >= 0.0) {
      node->declare_parameter("simple.min_height", min_height_param);
    }
    auto manager = std::make_shared<easynav::SimpleMapsManager>();
    manager->initialize(node, "simple");
    if (min_height) {
      *min_height = node->get_parameter("simple.min_height").as_double();
    }

    easynav::SimpleMap static_map;
    static_map.initialize(30, 30, 0.1, -1.5, -1.5, false);
    manager->set_static_map(static_map);

    easynav::PointPerception perception;
    perception.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame;
    perception.stamp = node->now();
    perception.valid = true;
    for (std::size_t i = 0; i < heights.size(); ++i) {
      const float xy = -1.25f + 0.5f * static_cast<float>(i);
      perception.data.push_back(pcl::PointXYZ(xy, xy, heights[i]));
    }

    easynav::NavState nav_state;
    nav_state.set("scan", perception);
    manager->update(nav_state);

    const auto & map = nav_state.get<easynav::SimpleMap>("map");
    std::vector<bool> result;
    for (std::size_t i = 0; i < heights.size(); ++i) {
      const double xy = -1.25 + 0.5 * static_cast<double>(i);
      const auto cell = map.metric_to_cell(xy, xy);
      result.push_back(map.at(cell.first, cell.second));
    }
    return result;
  }
};

TEST_F(SimpleMapsManagerMinHeightTest, DefaultIgnoresPointsBelowTenCentimeters)
{
  double min_height = 0.0;
  EXPECT_EQ(
    marked({0.05f, 0.08f, 0.20f}, -1.0, &min_height),
    (std::vector<bool>{false, false, true}));
  EXPECT_DOUBLE_EQ(min_height, 0.1);
}

TEST_F(SimpleMapsManagerMinHeightTest, LowerMinHeightKeepsALowLaser)
{
  double min_height = 0.0;
  EXPECT_EQ(
    marked({0.05f, 0.08f, 0.20f}, 0.07, &min_height),
    (std::vector<bool>{false, true, true}));
  EXPECT_DOUBLE_EQ(min_height, 0.07);
}

TEST_F(SimpleMapsManagerMinHeightTest, HigherMinHeightIgnoresMore)
{
  EXPECT_EQ(marked({0.05f, 0.08f, 0.20f}, 0.3), (std::vector<bool>{false, false, false}));
}
