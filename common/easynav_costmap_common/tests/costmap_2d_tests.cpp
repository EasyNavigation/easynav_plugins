// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include <gtest/gtest.h>
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using easynav::Costmap2D;

class Costmap2DTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(Costmap2DTest, BasicInitialization)
{
  Costmap2D map(5, 6, 0.2, -1.0, -2.0, 127);

  EXPECT_EQ(map.getSizeInCellsX(), 5u);
  EXPECT_EQ(map.getSizeInCellsY(), 6u);
  EXPECT_FLOAT_EQ(map.getResolution(), 0.2);
  EXPECT_FLOAT_EQ(map.getOriginX(), -1.0);
  EXPECT_FLOAT_EQ(map.getOriginY(), -2.0);

  for (unsigned int x = 0; x < 5; ++x) {
    for (unsigned int y = 0; y < 6; ++y) {
      EXPECT_EQ(map.getCost(x, y), 127u);
    }
  }
}

TEST_F(Costmap2DTest, SetAndGetCost)
{
  Costmap2D map(3, 3, 1.0, 0.0, 0.0, 0);

  map.setCost(1, 1, 255);
  EXPECT_EQ(map.getCost(1, 1), 255u);

  map.setCost(0, 2, 42);
  EXPECT_EQ(map.getCost(0, 2), 42u);
}

TEST_F(Costmap2DTest, WorldToMapAndMapToWorld)
{
  Costmap2D map(10, 10, 1.0, -5.0, -5.0, 0);

  unsigned int mx, my;
  EXPECT_TRUE(map.worldToMap(-4.6, -4.6, mx, my));
  EXPECT_EQ(mx, 0u);
  EXPECT_EQ(my, 0u);

  double wx, wy;
  map.mapToWorld(0, 0, wx, wy);
  EXPECT_NEAR(wx, -4.5, 1e-6);
  EXPECT_NEAR(wy, -4.5, 1e-6);
}

TEST_F(Costmap2DTest, OutOfBoundsDetection)
{
  Costmap2D map(5, 5, 1.0, 0.0, 0.0, 0);

  EXPECT_FALSE(map.inBounds(5, 0));
  EXPECT_FALSE(map.inBounds(0, 5));
  EXPECT_FALSE(map.inBounds(5, 5));
  EXPECT_TRUE(map.inBounds(4, 4));
}

TEST_F(Costmap2DTest, ClearMap)
{
  Costmap2D map(4, 4, 0.5, 0.0, 0.0, 100);

  map.setCost(2, 2, 250);
  map.resetMap(0, 0, 4, 4);  // clears to default value

  for (unsigned int x = 0; x < 4; ++x) {
    for (unsigned int y = 0; y < 4; ++y) {
      EXPECT_EQ(map.getCost(x, y), 100u);
    }
  }
}

TEST_F(Costmap2DTest, OccupancyGridConversion)
{
  Costmap2D map(4, 3, 0.2, -1.0, -0.6, 0);
  map.setCost(0, 0, 100);
  map.setCost(1, 1, 100);
  map.setCost(3, 2, 100);

  nav_msgs::msg::OccupancyGrid grid;
  map.toOccupancyGridMsg(grid);

  EXPECT_EQ(grid.info.width, 4u);
  EXPECT_EQ(grid.info.height, 3u);
  EXPECT_NEAR(grid.info.resolution, 0.2, 1e-6);
  EXPECT_NEAR(grid.info.origin.position.x, -1.0, 1e-6);
  EXPECT_NEAR(grid.info.origin.position.y, -0.6, 1e-6);

  std::vector<int> expected_indices = {
    0 * 4 + 0,
    1 * 4 + 1,
    2 * 4 + 3
  };

  for (size_t i = 0; i < grid.data.size(); ++i) {
    bool expected = std::find(expected_indices.begin(),
      expected_indices.end(), i) != expected_indices.end();
    EXPECT_EQ(grid.data[i], expected ? 100 : 0);
  }
}
