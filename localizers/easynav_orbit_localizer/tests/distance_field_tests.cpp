// Copyright 2026 Intelligent Robotics Lab
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

/// \brief The distance fields: exact distances, gradients towards free space, ray casts.

#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "easynav_orbit_localizer/DistanceField.hpp"
#include "test_maps.hpp"

using namespace easynav::orbit;  // NOLINT

TEST(GridDistanceField2D, DistanceToTheNearestWallAndItsGradient)
{
  const auto map = test_maps::emptyRoom(4.0, 4.0);
  GridDistanceField2D field(map);
  EXPECT_DOUBLE_EQ(field.resolution(), 0.05);
  // 1 m from the left wall (whose cells are at x in [0, 0.05)); cell centres: 0.025 -> 0.975
  Eigen::Vector3d g;
  const double d = field.distance(Eigen::Vector3d(1.0, 2.0, 0.0), &g);
  EXPECT_NEAR(d, 0.975, 0.03);
  EXPECT_NEAR(g.x(), 1.0, 1e-6);
  EXPECT_NEAR(g.y(), 0.0, 1e-6);
  // The middle is 2 m from every wall; near the top wall the gradient points down
  EXPECT_NEAR(field.distance(Eigen::Vector3d(2.0, 2.0, 0.0), nullptr), 1.975, 0.03);
  field.distance(Eigen::Vector3d(2.0, 3.7, 0.0), &g);
  EXPECT_NEAR(g.y(), -1.0, 1e-6);
  // Walls are at distance 0, outside is unknown
  EXPECT_NEAR(field.distance(Eigen::Vector3d(0.025, 2.0, 0.0), nullptr), 0.0, 1e-6);
  EXPECT_FALSE(std::isfinite(field.distance(Eigen::Vector3d(-1.0, 2.0, 0.0), nullptr)));
  // Just outside the edge wall: as the wall itself
  EXPECT_NEAR(field.distance(Eigen::Vector3d(-0.03, 2.0, 0.0), nullptr), 0.0, 1e-6);
  EXPECT_FALSE(field.inside(Eigen::Vector3d(5.0, 1.0, 0.0)));
  EXPECT_TRUE(field.inside(Eigen::Vector3d(1.0, 1.0, 0.0)));
}

TEST(GridDistanceField2D, DistanceIsExactAtAnObstacleInTheMiddle)
{
  auto map = test_maps::emptyRoom(6.0, 6.0);
  test_maps::fill(map, 3.0, 3.0, 3.0, 3.0, easynav::LETHAL_OBSTACLE);  // one cell at (3.025, 3.025)
  GridDistanceField2D field(map);
  const double d = field.distance(Eigen::Vector3d(3.025 + 0.6, 3.025 + 0.8, 0.0), nullptr);
  EXPECT_NEAR(d, 1.0, 0.03);
}

TEST(GridDistanceField2D, DistanceMatchesBruteForceEverywhere)
{
  auto map = test_maps::emptyRoom(3.0, 2.0);
  test_maps::fill(map, 1.5, 1.0, 1.5, 1.0, easynav::LETHAL_OBSTACLE);
  test_maps::fill(map, 0.5, 1.25, 0.5, 1.25, easynav::LETHAL_OBSTACLE);
  test_maps::fill(map, 2.2, 0.3, 2.6, 0.4, easynav::LETHAL_OBSTACLE);
  GridDistanceField2D field(map);
  const double res = map.getResolution();
  for (unsigned int i = 1; i + 1 < map.getSizeInCellsX(); ++i) {
    for (unsigned int j = 1; j + 1 < map.getSizeInCellsY(); ++j) {
      double best = 1e9;
      for (unsigned int a = 0; a < map.getSizeInCellsX(); ++a) {
        for (unsigned int b = 0; b < map.getSizeInCellsY(); ++b) {
          if (map.getCost(a, b) == easynav::LETHAL_OBSTACLE) {
            best = std::min(
              best, std::hypot(static_cast<double>(a) - i, static_cast<double>(b) - j) * res);
          }
        }
      }
      const double d = field.distance(Eigen::Vector3d((i + 0.5) * res, (j + 0.5) * res, 0.0),
        nullptr);
      ASSERT_NEAR(d, best, 1e-4) << "cell " << i << " " << j;
    }
  }
}

TEST(GridDistanceField2D, FreeCellsAndRayCasts)
{
  const auto map = test_maps::twinRooms();
  GridDistanceField2D field(map);
  EXPECT_TRUE(field.isFree(2.0, 3.0));
  EXPECT_FALSE(field.isFree(4.5, 2.0));     // the wall between the rooms
  EXPECT_TRUE(field.isFree(4.5, 4.5));      // the corridor
  EXPECT_FALSE(field.isFree(-1.0, 1.0));    // outside
  // From the middle of the left room, heading +x: the wall at x 4
  EXPECT_NEAR(field.raycast(2.0, 3.0, 0.0, 12.0), 2.0, 0.06);
  // Heading -y: the bottom wall at y 0
  EXPECT_NEAR(field.raycast(2.0, 3.0, -M_PI / 2.0, 12.0), 3.0, 0.06);
  // Along the corridor: the east wall at x 9
  EXPECT_NEAR(field.raycast(2.0, 4.5, 0.0, 12.0), 7.0, 0.06);
  // A short range returns the maximum
  EXPECT_DOUBLE_EQ(field.raycast(2.0, 3.0, 0.0, 1.0), 1.0);
}

TEST(CloudDistanceField3D, NearestPointDistanceAndSaturation)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      if (i >= 4 && i <= 10 && j >= 4 && j <= 10) {continue;}   // a hole in the floor
      cloud->points.emplace_back(0.5f * i, 0.5f * j, 0.0f);
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  CloudDistanceField3D field(cloud, 0.5, 1.0);
  Eigen::Vector3d g;
  EXPECT_NEAR(field.distance(Eigen::Vector3d(1.0, 1.0, 0.5), &g), 0.5, 1e-6);
  EXPECT_NEAR(g.z(), 1.0, 1e-6);
  // Over the hole's centre the nearest point is 2 m away: saturated, no gradient
  EXPECT_NEAR(field.distance(Eigen::Vector3d(3.5, 3.5, 0.0), &g), 1.0, 1e-6);
  EXPECT_NEAR(g.norm(), 0.0, 1e-12);
  EXPECT_FALSE(std::isfinite(field.distance(Eigen::Vector3d(2.0, 2.0, 10.0), nullptr)));
  EXPECT_FALSE(std::isfinite(field.distance(Eigen::Vector3d(50.0, 0.0, 0.0), nullptr)));
  EXPECT_EQ(field.pointsWithin(Eigen::Vector3d(1.0, 1.0, 0.0), 0.6).size(), 5u);
}
