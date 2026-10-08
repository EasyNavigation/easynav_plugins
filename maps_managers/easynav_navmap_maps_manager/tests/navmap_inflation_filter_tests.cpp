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

#include <cstdint>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"

using navmap_ros::FREE_SPACE;
using navmap_ros::INSCRIBED_INFLATED_OBSTACLE;
using navmap_ros::LETHAL_OBSTACLE;
using navmap_ros::NO_INFORMATION;

class NavmapInflationFilterTest : public ::testing::Test
{
protected:
  static constexpr int kCells = 60;          // 6 x 6 m at 0.1 m
  static constexpr float kRadius = 1.0f;
  static constexpr float kScaling = 3.0f;
  static constexpr float kInscribed = 0.3f;

  void SetUp() override
  {
    grid_.header.frame_id = "map";
    grid_.info.resolution = 0.1;
    grid_.info.width = kCells;
    grid_.info.height = kCells;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(kCells * kCells, 0);
  }

  // Sets column \p col (rows 10..49) to \p value.
  void column(int col, int8_t value)
  {
    for (int row = 10; row < 50; ++row) {
      grid_.data[row * kCells + col] = value;
    }
  }

  // Builds the NavMap, copies "occupancy" to "obstacles" and inflates it.
  void inflate()
  {
    nm_ = navmap_ros::from_occupancy_grid(grid_);
    for (std::size_t c = 0; c < nm_.navcels.size(); ++c) {
      const auto cid = static_cast<::navmap::NavCelId>(c);
      nm_.layer_set<std::uint8_t>(
        "obstacles", cid, nm_.layer_get<std::uint8_t>("occupancy", cid, FREE_SPACE));
    }
    ASSERT_TRUE(filter_.inflate_layer_u8(
      nm_, "obstacles", "inflated_obstacles", kRadius, kScaling, kInscribed));
  }

  std::uint8_t cost_at(float x, float y)
  {
    std::size_t sidx = 0;
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;
    EXPECT_TRUE(nm_.locate_navcel(Eigen::Vector3f(x, y, 0.0f), sidx, cid, bary, &hit));
    return nm_.layer_get<std::uint8_t>("inflated_obstacles", cid, 123);
  }

  nav_msgs::msg::OccupancyGrid grid_;
  ::navmap::NavMap nm_;
  easynav::navmap::InflationFilter filter_;
};

TEST_F(NavmapInflationFilterTest, AWallInflatesTheFreeSpaceAroundIt)
{
  column(30, 100);   // wall at x 3.0-3.1
  inflate();
  EXPECT_EQ(cost_at(3.05f, 3.0f), LETHAL_OBSTACLE);
  EXPECT_EQ(cost_at(2.85f, 3.0f), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(cost_at(3.25f, 3.0f), INSCRIBED_INFLATED_OBSTACLE);
  const auto mid = cost_at(2.45f, 3.0f);
  EXPECT_GT(mid, FREE_SPACE);
  EXPECT_LT(mid, INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(cost_at(1.55f, 3.0f), FREE_SPACE);   // beyond the radius
}

TEST_F(NavmapInflationFilterTest, CostDecreasesWithTheDistanceToTheWall)
{
  column(30, 100);
  inflate();
  std::uint8_t prev = INSCRIBED_INFLATED_OBSTACLE;
  for (float x = 2.65f; x > 1.9f; x -= 0.1f) {
    const auto c = cost_at(x, 3.0f);
    EXPECT_LE(c, prev) << "at x " << x;
    prev = c;
  }
}

// Unknown cells between a wall and the free space (as in maps built from scans) must not
// stop the inflation: the free space next to them is still close to the wall.
TEST_F(NavmapInflationFilterTest, AnUnknownRingAroundAWallDoesNotStopTheInflation)
{
  column(29, -1);
  column(30, 100);
  column(31, -1);
  inflate();
  EXPECT_EQ(cost_at(2.85f, 3.0f), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(cost_at(3.25f, 3.0f), INSCRIBED_INFLATED_OBSTACLE);
  const auto mid = cost_at(2.45f, 3.0f);
  EXPECT_GT(mid, FREE_SPACE);
  EXPECT_LT(mid, INSCRIBED_INFLATED_OBSTACLE);
}

TEST_F(NavmapInflationFilterTest, UnknownCellsStayUnknown)
{
  column(29, -1);
  column(30, 100);
  inflate();
  EXPECT_EQ(cost_at(2.95f, 3.0f), NO_INFORMATION);
  EXPECT_EQ(cost_at(3.05f, 3.0f), LETHAL_OBSTACLE);
}

TEST_F(NavmapInflationFilterTest, UnknownCellsAreNotObstacles)
{
  column(30, -1);
  inflate();
  EXPECT_EQ(cost_at(3.05f, 3.0f), NO_INFORMATION);
  EXPECT_EQ(cost_at(2.85f, 3.0f), FREE_SPACE);
  EXPECT_EQ(cost_at(3.25f, 3.0f), FREE_SPACE);
}

TEST_F(NavmapInflationFilterTest, AWideUnknownAreaStillCarriesTheInflation)
{
  // Unknown band 0.4 m wide between the wall and the free space.
  for (int col = 26; col < 30; ++col) {
    column(col, -1);
  }
  column(30, 100);
  inflate();
  // 0.5 m from the wall: beyond the inscribed radius, inside the inflation one.
  const auto c = cost_at(2.55f, 3.0f);
  EXPECT_GT(c, FREE_SPACE);
  EXPECT_LT(c, INSCRIBED_INFLATED_OBSTACLE);
}

TEST_F(NavmapInflationFilterTest, InflatingTwiceGivesTheSameLayer)
{
  column(29, -1);
  column(30, 100);
  inflate();
  const auto first = cost_at(2.45f, 3.0f);
  ASSERT_TRUE(filter_.inflate_layer_u8(
    nm_, "obstacles", "inflated_obstacles", kRadius, kScaling, kInscribed));
  EXPECT_EQ(cost_at(2.45f, 3.0f), first);
}
