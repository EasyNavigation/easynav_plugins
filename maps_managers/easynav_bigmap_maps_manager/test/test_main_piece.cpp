// Copyright 2026 José Miguel Guerrero
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


#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "easynav_bigmap_maps_manager/dem_grid.hpp"
#include "easynav_bigmap_maps_manager/narrow_passages.hpp"

using bigmap_maps_manager::keep_main_piece;

namespace
{
constexpr int kSide = 30;

bigmap_maps_manager::DemGrid flat(float z = 0.0f)
{
  bigmap_maps_manager::DemGrid d;
  d.side = kSide;
  d.resolution = 1.0;
  d.z.assign(static_cast<std::size_t>(kSide) * kSide, z);
  return d;
}

std::size_t at(int x, int y) {return static_cast<std::size_t>(y) * kSide + x;}
}  // namespace

TEST(MainPiece, TheRoofOfABuildingIsDroppedAndItsWallsStay)
{
  auto d = flat();
  for (int y = 10; y < 20; ++y) {
    for (int x = 10; x < 20; ++x) {
      d.z[at(x, y)] = 20.0f;
    }                                                        // a 10 x 10 m block, 20 m high
  }
  std::size_t dropped = 0;
  const auto v = keep_main_piece(d, {}, 25.0, &dropped);
  EXPECT_TRUE(v[at(2, 2)]);           // the street
  EXPECT_TRUE(v[at(9, 15)]);          // the foot of the wall
  EXPECT_TRUE(v[at(10, 15)]);         // the wall's top edge: the obstacle itself stays
  EXPECT_FALSE(v[at(15, 15)]);        // the roof: no goal can be picked there
  EXPECT_FALSE(v[at(12, 12)]);
  EXPECT_EQ(dropped, 64u);            // the 8 x 8 inner nodes of the roof
}

TEST(MainPiece, GroundThatIsReachableByARampIsKept)
{
  auto d = flat();
  // a platform 3 m up and a ramp to it (26 deg)
  for (int y = 10; y < 20; ++y) {
    for (int x = 10; x < 20; ++x) {
      d.z[at(x, y)] = 3.0f;
    }
    for (int x = 4; x < 10; ++x) {
      d.z[at(x, y)] = 0.5f * (x - 3);
    }
  }
  std::size_t dropped = 0;
  const auto v = keep_main_piece(d, {}, 35.0, &dropped);
  EXPECT_TRUE(v[at(15, 15)]);         // on the platform: the robot can drive there
  EXPECT_EQ(dropped, 0u);
  // the same platform with no ramp: it is a roof
  for (int y = 10; y < 20; ++y) {
    for (int x = 4; x < 10; ++x) {
      d.z[at(x, y)] = 0.0f;
    }
  }
  EXPECT_FALSE(keep_main_piece(d, {}, 35.0)[at(15, 15)]);
}

TEST(MainPiece, OnlyTheLargestPieceIsKept)
{
  auto d = flat();
  for (int y = 0; y < kSide; ++y) {
    d.z[at(15, y)] = -30.0f;
                                                           }   // a trench splits the map
  const auto v = keep_main_piece(d, {}, 25.0);
  EXPECT_TRUE(v[at(3, 3)]);           // the larger side (x < 14)
  EXPECT_FALSE(v[at(25, 20)]);        // the smaller one
}

TEST(MainPiece, NodesWithNoDataStayWithNoData)
{
  auto d = flat();
  std::vector<std::uint8_t> valid(d.z.size(), 1);
  valid[at(5, 5)] = 0;
  std::size_t dropped = 0;
  const auto v = keep_main_piece(d, valid, 25.0, &dropped);
  EXPECT_FALSE(v[at(5, 5)]);
  EXPECT_EQ(dropped, 0u);             // it had none to lose
  EXPECT_TRUE(v[at(6, 5)]);
}
