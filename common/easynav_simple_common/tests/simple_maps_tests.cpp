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

#include "easynav_simple_common/SimpleMap.hpp"

#include <memory>

/// \brief Fixture for SimpleMap tests (minimal)
class SimpleMapTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

/// \brief Basic access and fill tests
TEST_F(SimpleMapTest, BasicAccessAndFill)
{
  easynav::SimpleMap map;
  map.initialize(5, 5, 0.5, -1.0, -1.0);

  EXPECT_EQ(map.width(), 5u);
  EXPECT_EQ(map.height(), 5u);
  EXPECT_DOUBLE_EQ(map.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(map.origin_x(), -1.0);
  EXPECT_DOUBLE_EQ(map.origin_y(), -1.0);

  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      EXPECT_FALSE(map.at(x, y));
    }
  }

  map.at(2, 2) = true;
  EXPECT_TRUE(map.at(2, 2));

  map.fill(true);
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      EXPECT_TRUE(map.at(x, y));
    }
  }
}

/// \brief Metric conversion tests
TEST_F(SimpleMapTest, MetricConversion)
{
  easynav::SimpleMap map;
  map.initialize(10, 10, 1.0, -5.0, -5.0);

  auto metric = map.cell_to_metric(0, 0);
  EXPECT_NEAR(metric.first, -4.5, 1e-6);
  EXPECT_NEAR(metric.second, -4.5, 1e-6);

  auto cell = map.metric_to_cell(-4.6, -4.6);
  EXPECT_EQ(cell.first, 0u);
  EXPECT_EQ(cell.second, 0u);

  EXPECT_TRUE(map.check_bounds_metric(-4.6, -4.6));
  EXPECT_FALSE(map.check_bounds_metric(-6.0, -6.0));
}

/// \brief Save and load map tests
TEST_F(SimpleMapTest, SaveLoadMap)
{
  easynav::SimpleMap map;
  map.initialize(4, 4, 0.5, -1.0, -1.0);
  map.at(1, 1) = true;
  map.at(2, 2) = true;

  ASSERT_TRUE(map.save_to_file("test_map.txt"));

  easynav::SimpleMap loaded_map;
  ASSERT_TRUE(loaded_map.load_from_file("test_map.txt"));

  EXPECT_EQ(loaded_map.width(), 4u);
  EXPECT_EQ(loaded_map.height(), 4u);
  EXPECT_DOUBLE_EQ(loaded_map.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(loaded_map.origin_x(), -1.0);
  EXPECT_DOUBLE_EQ(loaded_map.origin_y(), -1.0);

  EXPECT_TRUE(loaded_map.at(1, 1));
  EXPECT_TRUE(loaded_map.at(2, 2));
  EXPECT_FALSE(loaded_map.at(0, 0));
}

/// \brief Conversion to and from OccupancyGrid test
TEST_F(SimpleMapTest, OccupancyGridConversion)
{
  easynav::SimpleMap original_map;
  original_map.initialize(4, 3, 0.2, -1.0, -0.6);
  original_map.at(0, 0) = true;
  original_map.at(1, 1) = true;
  original_map.at(3, 2) = true;

  nav_msgs::msg::OccupancyGrid grid_msg;
  original_map.to_occupancy_grid(grid_msg);

  EXPECT_EQ(grid_msg.info.width, 4u);
  EXPECT_EQ(grid_msg.info.height, 3u);
  EXPECT_NEAR(grid_msg.info.resolution, 0.2, 1e-6);
  EXPECT_NEAR(grid_msg.info.origin.position.x, -1.0, 1e-6);
  EXPECT_NEAR(grid_msg.info.origin.position.y, -0.6, 1e-6);

  std::vector<int> expected_indices = {
    0 * 4 + 0,  // (0,0)
    1 * 4 + 1,  // (1,1)
    2 * 4 + 3   // (3,2)
  };

  for (std::size_t i = 0; i < grid_msg.data.size(); ++i) {
    bool is_occupied = std::find(expected_indices.begin(),
      expected_indices.end(), i) != expected_indices.end();
    EXPECT_EQ(grid_msg.data[i], is_occupied ? 100 : 0);
  }

  easynav::SimpleMap recovered_map;
  recovered_map.from_occupancy_grid(grid_msg);

  EXPECT_EQ(recovered_map.width(), 4u);
  EXPECT_EQ(recovered_map.height(), 3u);
  EXPECT_NEAR(recovered_map.resolution(), 0.2, 1e-6);
  EXPECT_NEAR(recovered_map.origin_x(), -1.0, 1e-6);
  EXPECT_NEAR(recovered_map.origin_y(), -0.6, 1e-6);

  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 4; ++x) {
      EXPECT_EQ(
        recovered_map.at(x, y),
        original_map.at(x, y)
      ) << "Mismatch at (" << x << "," << y << ")";
    }
  }
}

/// \brief Downsampling with integer factor
TEST_F(SimpleMapTest, DownsampleIntegerFactor)
{
  easynav::SimpleMap map;
  map.initialize(4, 4, 0.5, 0.0, 0.0);

  map.at(0, 0) = true;
  map.at(1, 0) = true;
  map.at(1, 1) = true;
  map.at(2, 2) = true;

  auto downsampled = map.downsample_factor(2);
  ASSERT_NE(downsampled, nullptr);

  EXPECT_EQ(downsampled->width(), 2u);
  EXPECT_EQ(downsampled->height(), 2u);
  EXPECT_DOUBLE_EQ(downsampled->resolution(), 1.0);
  EXPECT_DOUBLE_EQ(downsampled->origin_x(), 0.0);
  EXPECT_DOUBLE_EQ(downsampled->origin_y(), 0.0);

  EXPECT_TRUE(downsampled->at(0, 0));
  EXPECT_FALSE(downsampled->at(1, 0));
  EXPECT_FALSE(downsampled->at(0, 1));
  EXPECT_FALSE(downsampled->at(1, 1));
}

/// \brief Downsampling to new resolution
TEST_F(SimpleMapTest, DownsampleToResolution)
{
  easynav::SimpleMap map;
  map.initialize(6, 6, 0.2, -1.0, -1.0);

  map.at(1, 1) = true;
  map.at(2, 2) = true;
  map.at(4, 4) = true;
  map.at(5, 5) = true;

  auto downsampled = map.downsample(0.4);
  ASSERT_NE(downsampled, nullptr);

  EXPECT_EQ(downsampled->width(), 3u);
  EXPECT_EQ(downsampled->height(), 3u);
  EXPECT_DOUBLE_EQ(downsampled->resolution(), 0.4);
  EXPECT_DOUBLE_EQ(downsampled->origin_x(), -1.0);
  EXPECT_DOUBLE_EQ(downsampled->origin_y(), -1.0);

  EXPECT_FALSE(downsampled->at(0, 0));
  EXPECT_FALSE(downsampled->at(1, 1));
  EXPECT_FALSE(downsampled->at(2, 2));
}

/// \brief Downsample that results in cropped edges
TEST_F(SimpleMapTest, DownsampleWithCropping)
{
  easynav::SimpleMap map;
  map.initialize(7, 7, 0.2, 0.0, 0.0);

  map.at(6, 6) = true;

  auto downsampled = map.downsample(0.7);

  EXPECT_EQ(downsampled->width(), 2u);
  EXPECT_EQ(downsampled->height(), 2u);
}
