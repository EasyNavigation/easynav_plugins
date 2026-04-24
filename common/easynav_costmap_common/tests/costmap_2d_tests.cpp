// Copyright 2025 Intelligent Robotics Lab
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

#include <gtest/gtest.h>
#include "easynav_costmap_common/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"

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

TEST_F(Costmap2DTest, TimestampFromOccupancyGridConstructor)
{
  nav_msgs::msg::OccupancyGrid in;
  in.header.stamp.sec = 123;
  in.header.stamp.nanosec = 456u;

  in.info.width = 2u;
  in.info.height = 3u;
  in.info.resolution = 0.5;
  in.info.origin.position.x = 1.0;
  in.info.origin.position.y = -2.0;
  in.data.assign(in.info.width * in.info.height, 0);

  Costmap2D map(in);
  const int64_t expected_ns = 123LL * 1000000000LL + 456LL;
  EXPECT_EQ(map.getLastModifiedStamp().nanoseconds(), expected_ns);

  nav_msgs::msg::OccupancyGrid out;
  map.toOccupancyGridMsg(out);

  EXPECT_EQ(out.header.stamp.sec, in.header.stamp.sec);
  EXPECT_EQ(out.header.stamp.nanosec, in.header.stamp.nanosec);
}

TEST_F(Costmap2DTest, TimestampFromCopyConstructor)
{
  nav_msgs::msg::OccupancyGrid in;
  in.header.stamp.sec = 10;
  in.header.stamp.nanosec = 20u;
  in.info.width = 1u;
  in.info.height = 1u;
  in.info.resolution = 1.0;
  in.info.origin.position.x = 0.0;
  in.info.origin.position.y = 0.0;
  in.data.assign(1u, 0);

  Costmap2D original(in);
  Costmap2D copy(original);

  const int64_t expected_ns = 10LL * 1000000000LL + 20LL;
  EXPECT_EQ(copy.getLastModifiedStamp().nanoseconds(), expected_ns);

  nav_msgs::msg::OccupancyGrid out;
  copy.toOccupancyGridMsg(out);
  EXPECT_EQ(out.header.stamp.sec, in.header.stamp.sec);
  EXPECT_EQ(out.header.stamp.nanosec, in.header.stamp.nanosec);
}

TEST_F(Costmap2DTest, TimestampNotUpdatedByChanges)
{
  nav_msgs::msg::OccupancyGrid in;
  in.header.stamp.sec = 7;
  in.header.stamp.nanosec = 8u;
  in.info.width = 3u;
  in.info.height = 3u;
  in.info.resolution = 1.0;
  in.info.origin.position.x = 0.0;
  in.info.origin.position.y = 0.0;
  in.data.assign(in.info.width * in.info.height, 0);

  Costmap2D map(in);
  const int64_t expected_ns = 7LL * 1000000000LL + 8LL;
  map.setCost(1, 1, 100);
  map.resizeMap(4u, 4u, 1.0, 1.0, 1.0);

  EXPECT_EQ(map.getLastModifiedStamp().nanoseconds(), expected_ns);

  nav_msgs::msg::OccupancyGrid out;
  map.toOccupancyGridMsg(out);
  EXPECT_EQ(out.header.stamp.sec, in.header.stamp.sec);
  EXPECT_EQ(out.header.stamp.nanosec, in.header.stamp.nanosec);
}
