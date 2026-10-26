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

/// \brief NavMapTerrain places a planar estimate on a NavMap's surface.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "easynav_fusion_localizer/NavMapTerrain.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

namespace
{

using easynav::NavMapTerrain;

/// A square [x0, x1] x [y0, y1] with z = z0 + slope * x, as two NavCels.
void add_square(
  ::navmap::NavMap & map, float x0, float x1, float y0, float y1, float z0, float slope = 0.0f)
{
  const auto s = map.create_surface("map");
  auto z = [&](float x) {return z0 + slope * x;};
  const auto a = map.add_vertex({x0, y0, z(x0)});
  const auto b = map.add_vertex({x1, y0, z(x1)});
  const auto c = map.add_vertex({x1, y1, z(x1)});
  const auto d = map.add_vertex({x0, y1, z(x0)});
  map.add_navcel_to_surface(s, map.add_navcel(a, b, c));
  map.add_navcel_to_surface(s, map.add_navcel(a, c, d));
}

void rpy(const tf2::Quaternion & q, double & r, double & p, double & y)
{
  tf2::Matrix3x3(q).getRPY(r, p, y);
}

tf2::Quaternion quat(double r, double p, double y)
{
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  return q;
}

TEST(NavMapTerrain, AnEmptyMapPlacesNothing)
{
  ::navmap::NavMap map;
  NavMapTerrain terrain;
  EXPECT_FALSE(terrain.project(map, 0.0, 0.0, 0.0, std::nullopt));
}

TEST(NavMapTerrain, AFlatSurfaceGivesItsHeightAndTheImuAttitude)
{
  ::navmap::NavMap map;
  add_square(map, -10, 10, -10, 10, 2.0f);
  map.rebuild_geometry_accels();
  NavMapTerrain terrain;
  const auto placed = terrain.project(map, 1.0, -3.0, 0.7, quat(0.1, -0.05, 2.5));
  ASSERT_TRUE(placed);
  EXPECT_TRUE(placed->on_map);
  EXPECT_NEAR(placed->z, 2.0, 1e-4);
  double r, p, y;
  rpy(placed->orientation, r, p, y);
  EXPECT_NEAR(r, 0.1, 1e-6);
  EXPECT_NEAR(p, -0.05, 1e-6);
  EXPECT_NEAR(y, 0.7, 1e-6) << "the filter's yaw, not the IMU's";
}

TEST(NavMapTerrain, OnARampWithoutImuTheAttitudeIsTheSurfaces)
{
  ::navmap::NavMap map;
  const float slope = 0.2f;  // about 11.3 degrees, rising along +x
  add_square(map, 0, 20, -5, 5, 0.0f, slope);
  map.rebuild_geometry_accels();
  NavMapTerrain terrain;

  // Heading up the ramp: nose up (negative pitch in ROS), no roll.
  auto placed = terrain.project(map, 10.0, 0.0, 0.0, std::nullopt);
  ASSERT_TRUE(placed);
  EXPECT_NEAR(placed->z, 2.0, 1e-3);
  double r, p, y;
  rpy(placed->orientation, r, p, y);
  EXPECT_NEAR(p, -std::atan(slope), 1e-3);
  EXPECT_NEAR(r, 0.0, 1e-3);
  EXPECT_NEAR(y, 0.0, 1e-3);

  // Across the ramp (facing +y): the slope becomes roll.
  placed = terrain.project(map, 10.0, 0.0, M_PI / 2, std::nullopt);
  ASSERT_TRUE(placed);
  rpy(placed->orientation, r, p, y);
  EXPECT_NEAR(std::abs(r), std::atan(slope), 1e-3);
  EXPECT_NEAR(p, 0.0, 1e-3);
  EXPECT_NEAR(y, M_PI / 2, 1e-3);
}

TEST(NavMapTerrain, FollowsTheHeightAlongTheRamp)
{
  ::navmap::NavMap map;
  add_square(map, 0, 20, -5, 5, 1.0f, 0.1f);
  map.rebuild_geometry_accels();
  NavMapTerrain terrain;
  for (double x = 0.5; x < 19.5; x += 0.5) {
    const auto placed = terrain.project(map, x, 0.3, 0.0, std::nullopt);
    ASSERT_TRUE(placed) << x;
    EXPECT_NEAR(placed->z, 1.0 + 0.1 * x, 1e-3) << x;
  }
}

TEST(NavMapTerrain, OnTwoLevelsItKeepsTheOneItIsOn)
{
  ::navmap::NavMap map;
  add_square(map, -10, 10, -10, 10, 0.0f);   // ground floor
  add_square(map, -10, 10, -10, 10, 3.0f);   // first floor, right above
  map.rebuild_geometry_accels();

  NavMapTerrain ground;
  EXPECT_NEAR(ground.project(map, 0.0, 0.0, 0.0, std::nullopt)->z, 0.0, 1e-4)
    << "the level nearest to the default hint (0)";
  for (double x = -5.0; x < 5.0; x += 1.0) {
    EXPECT_NEAR(ground.project(map, x, 1.0, 0.0, std::nullopt)->z, 0.0, 1e-4) << x;
  }

  NavMapTerrain upstairs;
  upstairs.set_height_hint(3.2);  // e.g. an initial pose on the first floor
  for (double x = -5.0; x < 5.0; x += 1.0) {
    EXPECT_NEAR(upstairs.project(map, x, 1.0, 0.0, std::nullopt)->z, 3.0, 1e-4) << x;
  }
}

TEST(NavMapTerrain, OffTheMapItKeepsTheLastHeight)
{
  ::navmap::NavMap map;
  add_square(map, 0, 10, 0, 10, 5.0f);
  map.rebuild_geometry_accels();
  NavMapTerrain terrain;
  EXPECT_FALSE(terrain.project(map, -50.0, -50.0, 0.0, std::nullopt))
    << "never on the map: nothing to place";
  ASSERT_TRUE(terrain.project(map, 5.0, 5.0, 0.0, std::nullopt));
  const auto off = terrain.project(map, -50.0, -50.0, 0.3, quat(0.02, 0.03, 0.0));
  ASSERT_TRUE(off);
  EXPECT_FALSE(off->on_map);
  EXPECT_NEAR(off->z, 5.0, 1e-4);
  double r, p, y;
  rpy(off->orientation, r, p, y);
  EXPECT_NEAR(r, 0.02, 1e-6);
  EXPECT_NEAR(p, 0.03, 1e-6);
  EXPECT_NEAR(y, 0.3, 1e-6);
  // Back on it.
  EXPECT_TRUE(terrain.project(map, 2.0, 2.0, 0.0, std::nullopt)->on_map);
}

TEST(NavMapTerrain, ResetForgetsTheLevel)
{
  ::navmap::NavMap map;
  add_square(map, -10, 10, -10, 10, 0.0f);
  add_square(map, -10, 10, -10, 10, 3.0f);
  map.rebuild_geometry_accels();
  NavMapTerrain terrain;
  terrain.set_height_hint(3.0);
  EXPECT_NEAR(terrain.project(map, 0.0, 0.0, 0.0, std::nullopt)->z, 3.0, 1e-4);
  terrain.reset();  // keeps the hint
  EXPECT_NEAR(terrain.project(map, 0.0, 0.0, 0.0, std::nullopt)->z, 3.0, 1e-4);
  terrain.set_height_hint(0.0);
  EXPECT_NEAR(terrain.project(map, 0.0, 0.0, 0.0, std::nullopt)->z, 0.0, 1e-4);
}

}  // namespace
