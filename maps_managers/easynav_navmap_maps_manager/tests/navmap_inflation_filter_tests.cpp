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

#include <cmath>
#include <cstdint>
#include <functional>

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
    ASSERT_TRUE(
      filter_.inflate_layer_u8(
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
  ASSERT_TRUE(
    filter_.inflate_layer_u8(
      nm_, "obstacles", "inflated_obstacles", kRadius, kScaling, kInscribed));
  EXPECT_EQ(cost_at(2.45f, 3.0f), first);
}

namespace
{

// nx x ny cells of `res` m at z = f(x, y) + z0, as a new surface, with an empty "obstacles" layer.
void add_grid(
  ::navmap::NavMap & nm, int nx, int ny, float res, float x0, float y0,
  const std::function<float(float, float)> & f)
{
  const std::size_t s = nm.create_surface("map");
  const auto base = static_cast<uint32_t>(nm.positions.size());
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      const float x = x0 + i * res, y = y0 + j * res;
      nm.add_vertex({x, y, f(x, y)});
    }
  }
  auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (nx + 1) + i);};
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
}

void finish(::navmap::NavMap & nm)
{
  nm.rebuild_geometry_accels();
  nm.add_layer<std::uint8_t>("obstacles", "", "", FREE_SPACE);
}

std::uint8_t cost_in(const ::navmap::NavMap & nm, float x, float y, float z)
{
  std::size_t sidx;
  ::navmap::NavCelId cid;
  Eigen::Vector3f bary, hit;
  EXPECT_TRUE(nm.locate_navcel({x, y, z}, sidx, cid, bary, &hit));
  return nm.layer_get<std::uint8_t>("inflated_obstacles", cid, 123);
}

::navmap::NavCelId cell_in(const ::navmap::NavMap & nm, float x, float y, float z)
{
  std::size_t sidx;
  ::navmap::NavCelId cid;
  Eigen::Vector3f bary, hit;
  EXPECT_TRUE(nm.locate_navcel({x, y, z}, sidx, cid, bary, &hit));
  return cid;
}

// Same cost formula as the filter, for a given distance.
std::uint8_t expected_cost(float d, float r_ins, float radius, float scaling)
{
  if (d <= r_ins) {return INSCRIBED_INFLATED_OBSTACLE;}
  if (d > radius) {return FREE_SPACE;}
  const double base = INSCRIBED_INFLATED_OBSTACLE - 1;
  return static_cast<std::uint8_t>(std::lround(std::exp(-scaling * (d - r_ins)) * base));
}

}  // namespace

TEST(NavmapInflationBorders, BorderCellsAreNotLethalAndCostByTheirDistance)
{
  ::navmap::NavMap nm;
  add_grid(nm, 40, 40, 0.1f, 0.0f, 0.0f, [](float, float) {return 0.0f;});
  finish(nm);
  easynav::navmap::InflationFilter f;
  ASSERT_TRUE(f.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", 1.0f, 3.0f, 0.3f));
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_NE(nm.layer_get<std::uint8_t>("inflated_obstacles", c), LETHAL_OBSTACLE);
  }
  // Cell centroid ~0.5 m from the x = 0 border: between inscribed and free.
  const auto cid = cell_in(nm, 0.53f, 2.03f, 0.0f);
  const float d = nm.navcel_centroid(cid).x();
  EXPECT_NEAR(nm.layer_get<std::uint8_t>("inflated_obstacles", cid),
    expected_cost(d, 0.3f, 1.0f, 3.0f), 1);
  EXPECT_EQ(cost_in(nm, 2.0f, 2.0f, 0.0f), FREE_SPACE);  // 2 m from every border
}

TEST(NavmapInflationBorders, NarrowCorridorKeepsItsTrueWidth)
{
  // A 1.2 m wide corridor (ramp between two side walls). Robot inscribed radius 0.5 m:
  // the centre line, 0.6 m from both edges, must stay passable.
  ::navmap::NavMap nm;
  add_grid(nm, 100, 3, 0.4f, 0.0f, 0.0f, [](float, float) {return 0.0f;});
  finish(nm);
  easynav::navmap::InflationFilter f;
  ASSERT_TRUE(f.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", 2.0f, 1.5f, 0.5f));
  const auto centre = cell_in(nm, 20.13f, 0.6f, 0.0f);
  EXPECT_LT(nm.layer_get<std::uint8_t>("inflated_obstacles", centre), INSCRIBED_INFLATED_OBSTACLE);
}

TEST(NavmapInflationBorders, DisabledBordersDoNotInflate)
{
  ::navmap::NavMap nm;
  add_grid(nm, 40, 40, 0.1f, 0.0f, 0.0f, [](float, float) {return 0.0f;});
  finish(nm);
  easynav::navmap::InflationFilter f;
  ASSERT_TRUE(
    f.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", 1.0f, 3.0f, 0.3f, false));
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_EQ(nm.layer_get<std::uint8_t>("inflated_obstacles", c), FREE_SPACE);
  }
}

TEST(NavmapInflationBorders, SlopeDistanceIsMeasuredAlongTheSurface)
{
  // Same obstacle and XY offset on a flat floor and on a 35 deg ramp: the ramp is farther.
  const float k = std::tan(35.0f * static_cast<float>(M_PI) / 180.0f);
  for (const float slope : {0.0f, k}) {
    ::navmap::NavMap nm;
    add_grid(nm, 60, 60, 0.1f, 0.0f, 0.0f, [slope](float x, float) {return slope * x;});
    finish(nm);
    nm.layer_set<std::uint8_t>("obstacles", cell_in(nm, 3.03f, 3.03f, slope * 3.03f),
      LETHAL_OBSTACLE);
    easynav::navmap::InflationFilter f;
    ASSERT_TRUE(
      f.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", 1.5f, 3.0f, 0.3f, false));
    const auto probe = cell_in(nm, 3.63f, 3.03f, slope * 3.63f);
    const auto obst = cell_in(nm, 3.03f, 3.03f, slope * 3.03f);
    const float d3 = (nm.navcel_centroid(probe) - nm.navcel_centroid(obst)).norm();
    EXPECT_NEAR(
      nm.layer_get<std::uint8_t>("inflated_obstacles", probe), expected_cost(d3, 0.3f, 1.5f, 3.0f),
      1)
      << "slope " << slope;
  }
}

TEST(NavmapInflationBorders, AnObstacleOnOneFloorDoesNotInflateTheFloorAbove)
{
  ::navmap::NavMap nm;
  add_grid(nm, 40, 40, 0.1f, 0.0f, 0.0f, [](float, float) {return 0.0f;});
  add_grid(nm, 40, 40, 0.1f, 0.0f, 0.0f, [](float, float) {return 2.5f;});
  finish(nm);
  nm.layer_set<std::uint8_t>("obstacles", cell_in(nm, 2.03f, 2.03f, 0.0f), LETHAL_OBSTACLE);
  easynav::navmap::InflationFilter f;
  ASSERT_TRUE(
    f.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", 1.0f, 3.0f, 0.3f, false));
  EXPECT_EQ(cost_in(nm, 2.13f, 2.03f, 0.0f), INSCRIBED_INFLATED_OBSTACLE);  // floor 0
  EXPECT_EQ(cost_in(nm, 2.03f, 2.03f, 2.5f), FREE_SPACE);                     // floor 1 above
  EXPECT_EQ(cost_in(nm, 2.13f, 2.03f, 2.5f), FREE_SPACE);
}
