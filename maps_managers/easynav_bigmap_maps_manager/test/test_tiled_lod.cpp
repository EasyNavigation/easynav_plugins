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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "easynav_bigmap_maps_manager/tiled_lod.hpp"
#include "navmap_core/NavMap.hpp"

using easynav::bigmap::LodParams;
using easynav::bigmap::TiledLod;
using navmap::NavCelId;
using navmap::NavMap;

namespace
{

// Bumpy ground: the finer levels keep the bumps, the coarser ones flatten them
float ground(float x, float y) {return 0.3f * std::sin(0.6f * x) * std::cos(0.45f * y);}

// A town of side x side m at `res` m: ground everywhere but the buildings (cells where
// building(x, y))
NavMap town(float side, float res, const std::function<bool(float, float)> & building)
{
  NavMap nm;
  const auto s = nm.create_surface("map");
  const int n = static_cast<int>(std::round(side / res));
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.positions.x.push_back(i * res);
      nm.positions.y.push_back(j * res);
      nm.positions.z.push_back(ground(i * res, j * res));
    }
  }
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      if (building((i + 0.5f) * res, (j + 0.5f) * res)) {continue;}
      const uint32_t a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
      nm.add_navcel_to_surface(s, nm.add_navcel(a, b, d));
      nm.add_navcel_to_surface(s, nm.add_navcel(a, d, c));
    }
  }
  nm.rebuild_geometry_accels();
  return nm;
}

// Blocks of 14 x 14 m every 20 m (6 m streets), and a narrow passage (1.5 m) cut through one
bool blocks(float x, float y)
{
  const float bx = std::fmod(x, 20.0f), by = std::fmod(y, 20.0f);
  const bool in_block = bx > 3.0f && bx<17.0f && by>3.0f && by < 17.0f;
  const bool passage = x > 49.25f && x<50.75f && y>40.0f && y < 60.0f;
  return in_block && !passage;
}

// Pieces of the map (NavCels sharing an edge) and length of the open edges
std::pair<int, double> pieces_and_border(const NavMap & nm)
{
  std::map<std::pair<uint32_t, uint32_t>, std::vector<NavCelId>> edges;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    for (int k = 0; k < 3; ++k) {
      const uint32_t a = nm.navcels[c].v[k], b = nm.navcels[c].v[(k + 1) % 3];
      edges[{std::min(a, b), std::max(a, b)}].push_back(c);
    }
  }
  std::vector<int> parent(nm.navcels.size());
  for (std::size_t i = 0; i < parent.size(); ++i) {
    parent[i] = static_cast<int>(i);
  }
  std::function<int(int)> find = [&](int x) {
      return parent[x] == x ? x : parent[x] = find(parent[x]);
    };
  double border = 0.0;
  for (const auto & [e, cels] : edges) {
    if (cels.size() == 1) {
      border += (nm.positions.at(e.first) - nm.positions.at(e.second)).norm();
    }
    for (std::size_t i = 1; i < cels.size(); ++i) {
      parent[find(cels[i])] = find(cels[0]);
    }
  }
  int pieces = 0;
  for (std::size_t i = 0; i < parent.size(); ++i) {
    pieces += find(static_cast<int>(i)) == static_cast<int>(i);
  }
  return {pieces, border};
}

bool on_map(const NavMap & nm, float x, float y)
{
  std::size_t s;
  NavCelId c;
  Eigen::Vector3f bary;
  return nm.locate_navcel(Eigen::Vector3f(x, y, ground(x, y) + 0.5f), s, c, bary, nullptr);
}

LodParams params()
{
  LodParams p;
  p.cell_min = 0.5;
  p.cell_max = 4.0;
  p.min_radius = 10.0;
  p.max_radius = 40.0;
  return p;
}

}  // namespace

TEST(TiledLod, TheLevelsAndTheirRadii)
{
  TiledLod lod;
  lod.build(town(20.0f, 0.5f, [](float, float) {return false;}), params());
  ASSERT_EQ(lod.levels(), 4);   // 0.5, 1, 2, 4 m
  EXPECT_DOUBLE_EQ(lod.cell(0), 0.5);
  EXPECT_DOUBLE_EQ(lod.cell(3), 4.0);
  EXPECT_DOUBLE_EQ(lod.radius(0), 0.0);
  EXPECT_DOUBLE_EQ(lod.radius(1), 10.0);
  EXPECT_DOUBLE_EQ(lod.radius(3), 40.0);
  EXPECT_DOUBLE_EQ(lod.radius(2), 20.0);   // geometric between them
}

TEST(TiledLod, FineNearTheRobotCoarseFarAndNoCracks)
{
  const NavMap map = town(100.0f, 0.5f, blocks);
  const auto [map_pieces, map_border] = pieces_and_border(map);
  ASSERT_EQ(map_pieces, 1);
  TiledLod lod;
  const auto st = lod.build(map, params(), 4);
  ASSERT_EQ(st.navcels.size(), 4u);
  EXPECT_GT(st.tiles, 50u);
  for (std::size_t k = 1; k < st.navcels.size(); ++k) {
    EXPECT_LT(st.navcels[k], st.navcels[k - 1]) << "level " << k;
  }

  const auto tiles = lod.select(1.0, 1.0);
  int fine = 0, coarse = 0;
  for (const auto t : tiles) {
    fine += lod.level(t) == 0;
    coarse += lod.level(t) == lod.levels() - 1;
  }
  EXPECT_GT(fine, 0);
  EXPECT_GT(coarse, 0);
  const NavMap nm = lod.assemble(tiles);
  EXPECT_LT(nm.navcels.size(), st.navcels[0] * 6 / 10);
  EXPECT_LT(nm.navcels.size(), map.navcels.size() / 10);

  // One piece, and about the same border as the map: tiles of different levels meet whole
  const auto [pieces, border] = pieces_and_border(nm);
  EXPECT_EQ(pieces, 1);
  EXPECT_NEAR(border, map_border, 0.05 * map_border);

  // The buildings are still holes, the streets ground, the passage open, also far away
  EXPECT_FALSE(on_map(nm, 10.0f, 10.0f));
  EXPECT_FALSE(on_map(nm, 90.0f, 90.0f));
  EXPECT_TRUE(on_map(nm, 1.5f, 50.0f));
  EXPECT_TRUE(on_map(nm, 99.0f, 99.0f));
  EXPECT_TRUE(on_map(nm, 50.0f, 50.0f));
}

TEST(TiledLod, EveryPlaceOfTheMapIsInExactlyOneTile)
{
  const NavMap map = town(100.0f, 0.5f, blocks);
  TiledLod lod;
  lod.build(map, params(), 4);
  for (const auto & robot : {Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(50.0, 50.0),
      Eigen::Vector2d(99.0, 30.0)})
  {
    const NavMap nm = lod.assemble(lod.select(robot.x(), robot.y()));
    double area = 0.0;
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      area += nm.navcels[c].area;
    }
    double map_area = 0.0;
    for (NavCelId c = 0; c < map.navcels.size(); ++c) {
      map_area += map.navcels[c].area;
    }
    EXPECT_NEAR(area, map_area, 0.03 * map_area) << robot.transpose();
    EXPECT_EQ(pieces_and_border(nm).first, 1) << robot.transpose();
  }
}

TEST(TiledLod, FloorsOverEachOtherStaySeparate)
{
  // Two floors 2.5 m apart over the same 100 x 100 m, with buildings in different places, so
  // their tiles keep different vertices on the same grid lines
  NavMap map = town(100.0f, 0.5f, blocks);
  const NavMap upper = town(100.0f, 0.5f, [](float x, float y) {return blocks(y, x + 7.0f);});
  const auto base = static_cast<uint32_t>(map.positions.size());
  for (std::size_t i = 0; i < upper.positions.size(); ++i) {
    map.positions.x.push_back(upper.positions.x[i]);
    map.positions.y.push_back(upper.positions.y[i]);
    map.positions.z.push_back(upper.positions.z[i] + 2.5f);
  }
  const auto s = map.create_surface("upper");
  for (const auto & c : upper.navcels) {
    map.add_navcel_to_surface(s, map.add_navcel(base + c.v[0], base + c.v[1], base + c.v[2]));
  }
  map.rebuild_geometry_accels();
  const auto [map_pieces, map_border] = pieces_and_border(map);
  ASSERT_EQ(map_pieces, 2);

  TiledLod lod;
  lod.build(map, params(), 4);
  for (const auto & robot : {Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(50.0, 50.0)}) {
    const auto tiles = lod.select(robot.x(), robot.y());
    NavMap nm = lod.assemble(tiles);
    lod.detail(nm, robot.x(), robot.y(), {}, 0.6);
    // No NavCel joins the floors, and each floor is whole (no cracks between its tiles)
    double span = 0.0;
    for (const auto & c : nm.navcels) {
      const float z0 = nm.positions.z[c.v[0]], z1 = nm.positions.z[c.v[1]],
        z2 = nm.positions.z[c.v[2]];
      span = std::max(span, static_cast<double>(std::max({z0, z1, z2}) - std::min({z0, z1, z2})));
    }
    EXPECT_LT(span, 1.0) << robot.transpose();
    const auto [pieces, border] = pieces_and_border(nm);
    EXPECT_EQ(pieces, 2) << robot.transpose();
    EXPECT_NEAR(border, map_border, 0.05 * map_border) << robot.transpose();
  }
}

TEST(TiledLod, TheLayersOfTheMapAreKept)
{
  NavMap map = town(40.0f, 0.5f, [](float, float) {return false;});
  auto layer = map.add_layer<uint8_t>("obstacles");
  for (NavCelId c = 0; c < map.navcels.size(); ++c) {
    const auto q = map.navcel_centroid(c);
    layer->data()[c] = (q.x() > 30.0f && q.x() < 32.0f && q.y() > 5.0f && q.y() < 35.0f) ? 254 : 0;
  }
  TiledLod lod;
  lod.build(map, params());
  const NavMap nm = lod.assemble(lod.select(0.0, 0.0));
  ASSERT_TRUE(nm.has_layer("obstacles"));
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto q = nm.navcel_centroid(c);
    const bool inside = q.x() > 30.1f && q.x() < 31.9f && q.y() > 5.1f && q.y() < 34.9f;
    const bool outside = q.x() < 29.9f || q.x() > 32.1f || q.y() < 4.9f || q.y() > 35.1f;
    if (inside) {EXPECT_EQ(nm.layer_get<uint8_t>("obstacles", c), 254);}
    if (outside) {EXPECT_EQ(nm.layer_get<uint8_t>("obstacles", c), 0);}
  }
}
