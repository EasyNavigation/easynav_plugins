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
/// \brief AMCLLocalizer "min_height": points below it are floor hits, not used to correct.

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/testing/LogCapture.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_costmap_localizer/AMCLLocalizer.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"

namespace
{

class TestAMCLLocalizer : public easynav::AMCLLocalizer
{
public:
  using easynav::AMCLLocalizer::correct;
  double min_height() const {return min_height_;}
};

}  // namespace

class CostmapAMCLMinHeightTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Corrects once with points at the given heights (robot footprint frame, 1 m ahead) and
  // returns whether the localizer found no points to correct with. min_height < 0: default.
  static bool no_points_to_correct(
    const std::vector<float> & heights, double min_height_param, double * min_height = nullptr)
  {
    auto node = std::make_shared<easynav::LocalizerNode>();
    if (min_height_param >= 0.0) {
      node->declare_parameter("test_localizer.min_height", min_height_param);
    }
    auto localizer = std::make_shared<TestAMCLLocalizer>();
    localizer->initialize(node, "test_localizer");
    if (min_height) {
      *min_height = localizer->min_height();
    }

    easynav::NavState nav_state;
    easynav::Costmap2D map(80, 80, 0.05, -2.0, -2.0);
    nav_state.set("map.base", map);

    easynav::PointPerception perception;
    perception.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    perception.stamp = node->now();
    perception.valid = true;
    for (std::size_t i = 0; i < heights.size(); ++i) {
      perception.data.push_back(pcl::PointXYZ(1.0f, 0.2f * static_cast<float>(i), heights[i]));
    }
    nav_state.set("scan", perception);

    easynav::testing::LogCapture log;
    localizer->correct(nav_state);
    return log.count({"No points to correct"}) > 0;
  }
};

TEST_F(CostmapAMCLMinHeightTest, DefaultIgnoresPointsBelowTenCentimeters)
{
  double min_height = 0.0;
  EXPECT_TRUE(no_points_to_correct({0.08f}, -1.0, &min_height));
  EXPECT_DOUBLE_EQ(min_height, 0.1);
  EXPECT_FALSE(no_points_to_correct({0.08f, 0.2f}, -1.0));
}

TEST_F(CostmapAMCLMinHeightTest, LowerMinHeightUsesALowLaser)
{
  // A laser 8 cm above the floor (e.g. TIAGo's base laser) corrects with min_height 0.07
  double min_height = 0.0;
  EXPECT_FALSE(no_points_to_correct({0.08f}, 0.07, &min_height));
  EXPECT_DOUBLE_EQ(min_height, 0.07);
}

TEST_F(CostmapAMCLMinHeightTest, PointsBelowMinHeightAreStillIgnored)
{
  EXPECT_TRUE(no_points_to_correct({0.05f}, 0.07));
  EXPECT_TRUE(no_points_to_correct({0.2f}, 0.3));
}
