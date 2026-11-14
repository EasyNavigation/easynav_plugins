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
#include <functional>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "easynav_navmap_maps_manager/filters/rtin_lod.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

using easynav::navmap::RtinLod;
using LayerU8 = navmap::LayerView<std::uint8_t>;

namespace
{

constexpr double kRes = 0.4;
constexpr double kOx = -3.0, kOy = 2.0;   // not at the origin, on purpose

// A grid NavMap like pointcloud_to_navmap's: two right triangles per cell.
struct Grid
{
  int nx{4}, ny{4};
  // diagonal of cell (i, j): true = main ((0,0)-(1,1))
  std::function<bool(int, int)> main_diag = [](int, int) {return true;};
  // which halves exist (half 0: right angle in the lower row of the cell)
  std::function<bool(int, int, int)> exists = [](int, int, int) {return true;};
  std::function<float(int, int)> z = [](int, int) {return 0.0f;};
  std::function<std::uint8_t(int, int, int)> value = [](int, int, int) {return 0;};
};

// half h of cell (i, j): its three corners (cell units), right angle first
std::vector<std::pair<int, int>> half_corners(bool main, int h)
{
  if (main) {   // right angles at (1,0) and (0,1)
    return h == 0 ? std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 0}} :
           std::vector<std::pair<int, int>>{{0, 1}, {0, 0}, {1, 1}};
  }
  return h == 0 ? std::vector<std::pair<int, int>>{{0, 0}, {1, 0}, {0, 1}} :   // anti: (0,0), (1,1)
         std::vector<std::pair<int, int>>{{1, 1}, {0, 1}, {1, 0}};
}

struct Built
{
  navmap::NavMap map;
  std::map<std::tuple<int, int, int>, navmap::NavCelId> cid;   // (i, j, h) -> NavCel
};

Built make_map(const Grid & g)
{
  Built b;
  auto & m = b.map;
  m.create_surface("map");
  std::map<std::pair<int, int>, std::uint32_t> vid;
  auto vertex = [&](int x, int y) {
      auto it = vid.find({x, y});
      if (it != vid.end()) {return it->second;}
      const auto id = m.add_vertex(
        Eigen::Vector3f(
          static_cast<float>(kOx + x * kRes), static_cast<float>(kOy + y * kRes), g.z(x, y)));
      vid[{x, y}] = id;
      return id;
    };
  std::vector<std::uint8_t> vals;
  for (int j = 0; j < g.ny; ++j) {
    for (int i = 0; i < g.nx; ++i) {
      for (int h = 0; h < 2; ++h) {
        if (!g.exists(i, j, h)) {continue;}
        const auto c = half_corners(g.main_diag(i, j), h);
        const auto id = m.add_navcel(
          vertex(i + c[0].first, j + c[0].second), vertex(i + c[1].first, j + c[1].second),
          vertex(i + c[2].first, j + c[2].second));
        m.add_navcel_to_surface(0, id);
        b.cid[{i, j, h}] = id;
        vals.push_back(g.value(i, j, h));
      }
    }
  }
  auto l = m.add_layer<std::uint8_t>("cost", "", "", 0);
  l->data() = vals;
  m.rebuild_geometry_accels();
  return b;
}

void set_value(Built & b, int i, int j, int h, std::uint8_t v)
{
  auto l = std::dynamic_pointer_cast<LayerU8>(b.map.layers.get("cost"));
  l->data()[b.cid.at({i, j, h})] = v;
}

// Grid coordinates of an output vertex
std::pair<int, int> gxy(const navmap::NavMap & m, std::uint32_t v)
{
  const auto p = m.positions.at(v);
  return {static_cast<int>(std::lround((p.x() - kOx) / kRes)),
    static_cast<int>(std::lround((p.y() - kOy) / kRes))};
}

double area2(const navmap::NavMap & m, const navmap::NavCel & c)
{
  const auto a = gxy(m, c.v[0]), b = gxy(m, c.v[1]), d = gxy(m, c.v[2]);
  return static_cast<double>((b.first - a.first) * (d.second - a.second) -
         (b.second - a.second) * (d.first - a.first));
}

bool inside(const navmap::NavMap & m, const navmap::NavCel & c, double x, double y)
{
  if (area2(m, c) == 0.0) {return false;}   // a dead NavCel holds nothing
  double s[3];
  for (int e = 0; e < 3; ++e) {
    const auto a = gxy(m, c.v[e]), b = gxy(m, c.v[(e + 1) % 3]);
    s[e] = (b.first - a.first) * (y - a.second) - (b.second - a.second) * (x - a.first);
  }
  return (s[0] >= 0 && s[1] >= 0 && s[2] >= 0) || (s[0] <= 0 && s[1] <= 0 && s[2] <= 0);
}

// The live NavCels of extract()'s map (the dead ones are degenerate, all on vertex 0)
bool live(const navmap::NavCel & c) {return !(c.v[0] == c.v[1] && c.v[1] == c.v[2]);}

template<class L>
navmap::NavMap ex(L && lod, RtinLod::Stats * st = nullptr)
{
  return *lod.extract(st);
}

// Every check that does not depend on the scenario
void check_invariants(const Built & src, const navmap::NavMap & out, const Grid & g)
{
  const auto lv = std::dynamic_pointer_cast<LayerU8>(out.layers.get("cost"));
  const auto ll = std::dynamic_pointer_cast<LayerU8>(out.layers.get("lod_level"));
  const auto lm = std::dynamic_pointer_cast<LayerU8>(out.layers.get("modified"));
  ASSERT_TRUE(lv && ll && lm);
  ASSERT_EQ(lv->size(), out.navcels.size());

  // the vertices of the live NavCels (the map keeps those of NavCels gone: they are unused)
  std::set<std::pair<int, int>> verts;
  for (const auto & c : out.navcels) {
    if (!live(c)) {continue;}
    for (int e = 0; e < 3; ++e) {
      verts.insert(gxy(out, c.v[e]));
    }
  }

  // Counter-clockwise, no T-junction: no vertex strictly inside an edge
  std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> edges;
  std::size_t nlive = 0;
  for (const auto & c : out.navcels) {
    if (!live(c)) {
      for (int e = 0; e < 3; ++e) {
        EXPECT_EQ(c.neighbor[e], navmap::NavCel{}.neighbor[e]);
      }
      continue;
    }
    ++nlive;
    EXPECT_GT(area2(out, c), 0.0);
    for (int e = 0; e < 3; ++e) {
      auto a = gxy(out, c.v[e]), b = gxy(out, c.v[(e + 1) % 3]);
      const int dx = b.first - a.first, dy = b.second - a.second;
      const int steps = std::gcd(std::abs(dx), std::abs(dy));
      for (int s = 1; s < steps; ++s) {
        EXPECT_FALSE(verts.count({a.first + dx / steps * s, a.second + dy / steps * s}))
          << "T-junction on edge (" << a.first << "," << a.second << ")-(" << b.first << "," <<
          b.second << ")";
      }
      if (b < a) {std::swap(a, b);}
      ++edges[{a, b}];
    }
  }
  for (const auto & [e, n] : edges) {
    EXPECT_LE(n, 2);
  }
  EXPECT_LE(out.navcels.size() - nlive, std::max<std::size_t>(nlive, 65536));   // dead ones
  // a live NavCel's neighbours are live
  for (const auto & c : out.navcels) {
    if (!live(c)) {continue;}
    for (int e = 0; e < 3; ++e) {
      if (c.neighbor[e] != navmap::NavCel{}.neighbor[e]) {
        EXPECT_TRUE(live(out.navcels[c.neighbor[e]]));
      }
    }
  }

  // Every covered source half lies in exactly one output NavCel, whose value is at least its own
  // (merged NavCels: exactly its own); output area = covered area
  double covered = 0.0, out_area = 0.0;
  for (const auto & c : out.navcels) {
    if (live(c)) {
      out_area += area2(out, c) / 2.0;
    }
  }
  for (int j = 0; j < g.ny; ++j) {
    for (int i = 0; i < g.nx; ++i) {
      for (int h = 0; h < 2; ++h) {
        if (!g.exists(i, j, h)) {continue;}
        const auto c = half_corners(g.main_diag(i, j), h);
        // a point strictly inside the half, off the grid lines and off every diagonal
        // (x +- y never an integer): the centroid would lie on the edges of merged NavCels
        const double x = i + c[0].first + 0.25 * (c[1].first - c[0].first) +
          0.2 * (c[2].first - c[0].first);
        const double y = j + c[0].second + 0.25 * (c[1].second - c[0].second) +
          0.2 * (c[2].second - c[0].second);
        int found = -1, count = 0;
        for (std::size_t k = 0; k < out.navcels.size(); ++k) {
          if (live(out.navcels[k]) && inside(out, out.navcels[k], x, y)) {
            found = static_cast<int>(k);
            ++count;
          }
        }
        if (g.exists(i, j, 0) && g.exists(i, j, 1)) {
          EXPECT_EQ(count, 1) << "cell " << i << "," << j << " half " << h << " not covered";
        }
        if (count == 0) {continue;}   // a half of a cell split by the other diagonal, alone
        covered += 0.5;
        EXPECT_EQ(count, 1);
        const auto v = g.value(i, j, h);
        EXPECT_GE(lv->data()[found], v);
        if (ll->data()[found] > 0) {
          // merged: its own value, or (finest NavCel on the other diagonal) the highest of the
          // two halves of its cell
          const auto other = g.exists(i, j, 1 - h) ? std::max(v, g.value(i, j, 1 - h)) : v;
          EXPECT_TRUE(lv->data()[found] == v || lv->data()[found] == other)
            << "cell " << i << "," << j << ": " << int(lv->data()[found]) << " over " << int(v);
        }
        EXPECT_EQ(lm->data()[found], ll->data()[found] > 0 ? 1 : 0);
      }
    }
  }
  // the source halves whose centroids are covered fill the output, up to the halves that lose
  // their cell mate when the tree uses the other diagonal
  EXPECT_LE(out_area, covered + 1e-9);
  (void)src;
}

using Key = std::vector<std::tuple<std::pair<int, int>, std::pair<int, int>, std::pair<int, int>,
    int, int>>;

Key canon(const navmap::NavMap & out)
{
  const auto lv = std::dynamic_pointer_cast<LayerU8>(out.layers.get("cost"));
  const auto ll = std::dynamic_pointer_cast<LayerU8>(out.layers.get("lod_level"));
  Key k;
  for (std::size_t i = 0; i < out.navcels.size(); ++i) {
    if (!live(out.navcels[i])) {continue;}
    std::vector<std::pair<int, int>> v;
    for (int e = 0; e < 3; ++e) {
      v.push_back(gxy(out, out.navcels[i].v[e]));
    }
    std::sort(v.begin(), v.end());
    k.emplace_back(v[0], v[1], v[2], lv->data()[i], ll->data()[i]);
  }
  std::sort(k.begin(), k.end());
  return k;
}

RtinLod built(const Built & b, float tol = 0.05f, float angle = 5.0f, float tile = 50.0f)
{
  RtinLod lod;
  std::string err;
  RtinLod::Params p;
  p.height_tolerance = tol;
  p.max_angle_deg = angle;
  p.tile_size = tile;
  EXPECT_TRUE(lod.build(b.map, "cost", p, &err)) << err;
  return lod;
}

// largest angle (deg) between an output NavCel and the source halves under it
double worst_angle_under(const navmap::NavMap & out, const Built & src, const Grid & g)
{
  double worst = 0.0;
  for (const auto & [key, cid] : src.cid) {
    const auto [i, j, h] = key;
    const auto c = half_corners(g.main_diag(i, j), h);
    const double x = i + c[0].first + 0.25 * (c[1].first - c[0].first) +
      0.2 * (c[2].first - c[0].first);
    const double y = j + c[0].second + 0.25 * (c[1].second - c[0].second) +
      0.2 * (c[2].second - c[0].second);
    for (const auto & oc : out.navcels) {
      if (!inside(out, oc, x, y)) {continue;}
      const double d = std::clamp(
        static_cast<double>(oc.normal.dot(src.map.navcels[cid].normal)), -1.0, 1.0);
      worst = std::max(worst, std::acos(std::abs(d)) * 180.0 / M_PI);
    }
  }
  return worst;
}

}  // namespace

TEST(RtinLod, UniformSquareMergesIntoTwoTriangles)
{
  Grid g;
  g.nx = g.ny = 8;
  auto b = make_map(g);
  auto lod = built(b);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  EXPECT_EQ(st.triangles, 2u);
  EXPECT_EQ(st.finest, 0u);
  EXPECT_EQ(st.max_level, 6);   // 8x8 cells: 128 halves -> 2: 6 merges
  check_invariants(b, out, g);
}

TEST(RtinLod, OneCellMapIsKept)
{
  Grid g;
  g.nx = g.ny = 1;
  auto b = make_map(g);
  auto lod = built(b);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  EXPECT_EQ(st.triangles, 2u);
  EXPECT_EQ(st.finest, 2u);
  check_invariants(b, out, g);
}

TEST(RtinLod, NonPowerOfTwoRectangle)
{
  Grid g;
  g.nx = 7;
  g.ny = 3;
  auto b = make_map(g);
  auto lod = built(b);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  check_invariants(b, out, g);
  double a = 0.0;
  for (const auto & c : out.navcels) {
    a += area2(out, c) / 2.0;
                                                               }
  EXPECT_DOUBLE_EQ(a, 21.0);   // every cell covered (main diagonal everywhere, as the tree's
                               // diagonal or both halves present)
  EXPECT_LT(st.triangles, 42u);
}

TEST(RtinLod, ObstacleKeepsFullResolutionOnlyAroundIt)
{
  Grid g;
  g.nx = g.ny = 32;
  g.value = [](int i, int j, int) {return (i == 10 && j == 20) ? 254 : 0;};
  auto b = make_map(g);
  auto lod = built(b);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  check_invariants(b, out, g);
  EXPECT_GE(st.finest, 2u);
  EXPECT_LT(st.triangles, 120u);   // a few dozen: the obstacle, the forced splits around it
}

TEST(RtinLod, SplitThenMergeBackSequence)
{
  Grid g;
  g.nx = g.ny = 16;
  auto b = make_map(g);
  auto lod = built(b);
  const auto initial = canon(ex(lod));
  ASSERT_EQ(initial.size(), 2u);

  EXPECT_EQ(lod.update(b.map), 0u);   // nothing changed

  set_value(b, 5, 9, 0, 120);
  g.value = [](int i, int j, int h) {return (i == 5 && j == 9 && h == 0) ? 120 : 0;};
  EXPECT_EQ(lod.update(b.map), 1u);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  check_invariants(b, out, g);
  EXPECT_GT(st.triangles, 2u);
  EXPECT_GE(st.finest, 1u);

  // a value change that keeps the obstacle: the mesh is the same, the value follows
  set_value(b, 5, 9, 0, 200);
  g.value = [](int i, int j, int h) {return (i == 5 && j == 9 && h == 0) ? 200 : 0;};
  EXPECT_EQ(lod.update(b.map), 1u);
  out = ex(lod);
  check_invariants(b, out, g);

  // gone: merged back to the two roots
  set_value(b, 5, 9, 0, 0);
  g.value = [](int, int, int) {return 0;};
  EXPECT_EQ(lod.update(b.map), 1u);
  EXPECT_EQ(canon(ex(lod)), initial);
}

TEST(RtinLod, HolesAreNeverCovered)
{
  Grid g;
  g.nx = g.ny = 16;
  g.exists = [](int i, int j, int) {return !(i >= 4 && i < 7 && j >= 4 && j < 6);};
  auto b = make_map(g);
  auto lod = built(b);
  auto out = ex(lod);
  check_invariants(b, out, g);
  for (const auto & c : out.navcels) {
    for (int i = 4; i < 7; ++i) {
      for (int j = 4; j < 6; ++j) {
        EXPECT_FALSE(inside(out, c, i + 0.5, j + 0.5)) << "cell " << i << "," << j;
      }
    }
  }
}

TEST(RtinLod, LoneHalfAcrossTheTreeDiagonalIsDropped)
{
  // anti diagonals and one half missing: where the tree uses the main diagonal, the remaining
  // half cannot be a full leaf and is not covered (never a NavCel over a hole)
  Grid g;
  g.nx = g.ny = 4;
  g.main_diag = [](int, int) {return false;};
  g.exists = [](int i, int j, int h) {return !(i == 1 && j == 1 && h == 1);};
  auto b = make_map(g);
  auto lod = built(b);
  auto out = ex(lod);
  check_invariants(b, out, g);
  for (const auto & c : out.navcels) {
    EXPECT_FALSE(inside(out, c, 1.75, 1.75));   // inside the missing half (corner (1,1))
  }
}

TEST(RtinLod, OtherDiagonalTakesTheHighestOfTheTwoHalves)
{
  Grid g;
  g.nx = g.ny = 2;
  g.main_diag = [](int, int) {return false;};   // the tree alternates: some cells differ
  g.value = [](int i, int j, int h) {return (i == 0 && j == 0 && h == 1) ? 77 : 0;};
  auto b = make_map(g);
  auto lod = built(b);
  auto out = ex(lod);
  check_invariants(b, out, g);
  // cell (0,0) is fully covered; whatever covers its half 0 has at least 0, and its half 1
  // at least 77 (checked in the invariants); a NavCel over both halves has 77
  const auto lv = std::dynamic_pointer_cast<LayerU8>(out.layers.get("cost"));
  for (std::size_t k = 0; k < out.navcels.size(); ++k) {
    if (inside(out, out.navcels[k], 0.2, 0.2) && inside(out, out.navcels[k], 0.8, 0.8)) {
      EXPECT_EQ(lv->data()[k], 77);
    }
  }
}

TEST(RtinLod, PlanarRampMergesWhateverItsHeight)
{
  Grid g;
  g.nx = g.ny = 16;
  g.z = [](int x, int y) {return 0.1f * x + 0.05f * y;};   // ~15 deg, 2.4 m of rise
  auto b = make_map(g);
  auto lod = built(b, 0.01f, 0.5f);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  check_invariants(b, out, g);
  EXPECT_EQ(st.triangles, 2u);
  EXPECT_LT(worst_angle_under(out, b, g), 0.1);   // float32 positions: hundredths of a degree
}

TEST(RtinLod, CreaseBetweenSlopesIsKept)
{
  // a ridge along x = 8: slopes of +-5.7 deg (11.4 between them)
  Grid g;
  g.nx = g.ny = 16;
  g.z = [](int x, int) {return 0.04f * std::abs(x - 8);};
  auto b = make_map(g);
  {
    auto lod = built(b, 1.0f, 5.0f);   // the deviation does not limit: only the angle
    auto out = ex(lod);
    check_invariants(b, out, g);
    EXPECT_LE(worst_angle_under(out, b, g), 5.0 + 1e-3);
    for (const auto & c : out.navcels) {   // no NavCel crosses the ridge
      int lo = 1 << 30, hi = -(1 << 30);
      for (int e = 0; e < 3; ++e) {
        lo = std::min(lo, gxy(out, c.v[e]).first);
        hi = std::max(hi, gxy(out, c.v[e]).first);
      }
      EXPECT_FALSE(lo < 8 && hi > 8);
    }
  }
  {
    auto lod = built(b, 1.0f, 15.0f);   // the ridge angle is allowed: merged across it
    RtinLod::Stats st;
    auto out = ex(lod, &st);
    check_invariants(b, out, g);
    EXPECT_LE(worst_angle_under(out, b, g), 15.0 + 1e-3);
    EXPECT_LT(st.triangles, 20u);
  }
}

TEST(RtinLod, CurvedGroundIsBoundedByTheDeviation)
{
  // a gentle bowl: every cell within a fraction of a degree of its neighbours
  Grid g;
  g.nx = g.ny = 32;
  g.z = [](int x, int y) {return 0.002f * ((x - 16) * (x - 16) + (y - 16) * (y - 16));};
  auto b = make_map(g);
  for (const float tol : {0.01f, 0.1f}) {
    auto lod = built(b, tol, 45.0f);   // only the deviation limits
    auto out = ex(lod);
    check_invariants(b, out, g);
    // every source vertex within tol of the plane of the output NavCel over it
    for (const auto & c : out.navcels) {
      const Eigen::Vector3f p0 = out.positions.at(c.v[0]);
      for (int x = 0; x <= g.nx; ++x) {
        for (int y = 0; y <= g.ny; ++y) {
          if (!inside(out, c, x, y)) {continue;}
          const Eigen::Vector3f q(
            static_cast<float>(kOx + x * kRes), static_cast<float>(kOy + y * kRes), g.z(x, y));
          EXPECT_LE(std::abs(c.normal.dot(q - p0)), tol + 1e-4f);
        }
      }
    }
  }
  RtinLod::Stats fine, coarse;
  ex(built(b, 0.01f, 45.0f), &fine);
  ex(built(b, 0.1f, 45.0f), &coarse);
  EXPECT_GT(fine.triangles, coarse.triangles);
}

TEST(RtinLod, ZeroThresholdsStillMergeFlatGround)
{
  Grid g;
  g.nx = g.ny = 8;
  auto b = make_map(g);
  RtinLod::Stats st;
  ex(built(b, 0.0f, 0.0f), &st);
  EXPECT_EQ(st.triangles, 2u);
}

TEST(RtinLod, ExtremeValues)
{
  Grid g;
  g.nx = g.ny = 8;
  g.value = [](int i, int, int) {return i < 4 ? 255 : 254;};
  auto b = make_map(g);
  auto lod = built(b);
  RtinLod::Stats st;
  auto out = ex(lod, &st);
  check_invariants(b, out, g);
  EXPECT_LT(st.triangles, 64u);   // two uniform halves of the square, merged
}

TEST(RtinLod, IncrementalEqualsRebuildOverRandomSequences)
{
  std::mt19937 rng(42);
  const float tiles[] = {0.6f, 1.5f, 100.0f, 3.0f};   // m: many tiles ... one
  for (int trial = 0; trial < 4; ++trial) {
    const float tile = tiles[trial];
    Grid g;
    g.nx = 20 + trial * 3;
    g.ny = 13 + trial;
    std::uniform_int_distribution<int> coin(0, 9);
    std::vector<int> diag(g.nx * g.ny), hole(g.nx * g.ny);
    for (auto & d : diag) {
      d = coin(rng) < 7;
    }
    for (auto & h : hole) {
      h = coin(rng) == 0;
    }
    const int nx = g.nx;
    g.main_diag = [diag, nx](int i, int j) {return diag[j * nx + i] != 0;};
    g.exists = [hole, nx](int i, int j, int) {return hole[j * nx + i] == 0;};
    std::vector<std::uint8_t> vals(2 * g.nx * g.ny, 0);
    g.value = [&vals, nx](int i, int j, int h) {return vals[2 * (j * nx + i) + h];};
    auto b = make_map(g);
    auto lod = built(b, 0.05f, 5.0f, tile);

    std::uniform_int_distribution<int> ci(0, g.nx - 1), cj(0, g.ny - 1), ch(0, 1);
    const std::uint8_t palette[] = {0, 0, 0, 50, 128, 253, 254, 255};
    std::uniform_int_distribution<int> pv(0, 7), burst(1, 12);
    for (int step = 0; step < 25; ++step) {
      const int n = burst(rng);
      for (int k = 0; k < n; ++k) {
        const int i = ci(rng), j = cj(rng), h = ch(rng);
        if (!g.exists(i, j, h)) {continue;}
        const std::uint8_t v = palette[pv(rng)];
        vals[2 * (j * nx + i) + h] = v;
        set_value(b, i, j, h, v);
      }
      lod.update(b.map);
      const auto inc = ex(lod);
      auto fresh = built(b, 0.05f, 5.0f, tile);
      ASSERT_EQ(canon(inc), canon(ex(fresh))) << "trial " << trial << " step " << step;
      check_invariants(b, inc, g);
    }
  }
}

TEST(RtinLod, WindowNowSweepLater)
{
  Grid g;
  g.nx = g.ny = 32;
  std::vector<std::uint8_t> vals(2 * 32 * 32, 0);
  g.value = [&vals](int i, int j, int h) {return vals[2 * (j * 32 + i) + h];};
  auto b = make_map(g);
  auto lod = built(b);
  const auto initial = canon(ex(lod));

  // one change near the robot (cell 3,3) and one far (cell 28,28)
  vals[2 * (3 * 32 + 3)] = 90;
  set_value(b, 3, 3, 0, 90);
  vals[2 * (28 * 32 + 28)] = 90;
  set_value(b, 28, 28, 0, 90);
  const double rx = kOx + 3.5 * kRes, ry = kOy + 3.5 * kRes;

  // no sweep: only the near one
  EXPECT_EQ(lod.update_window(b.map, rx, ry, 1.0, 0), 1u);
  auto out = ex(lod);
  const auto lv = std::dynamic_pointer_cast<LayerU8>(out.layers.get("cost"));
  for (std::size_t k = 0; k < out.navcels.size(); ++k) {
    if (inside(out, out.navcels[k], 28.75, 28.25)) {EXPECT_EQ(lv->data()[k], 0);}
  }
  // the same window again: nothing new
  EXPECT_EQ(lod.update_window(b.map, rx, ry, 1.0, 0), 0u);
  // a window outside the map does not fail
  EXPECT_EQ(lod.update_window(b.map, -1e6, -1e6, 5.0, 0), 0u);

  // the sweep reaches the far one within source_navcels() / sweep calls
  const std::size_t sweep = 100;
  std::size_t total = 0;
  for (std::size_t c = 0; c <= lod.source_navcels() / sweep; ++c) {
    total += lod.update_window(b.map, rx, ry, 1.0, sweep);
  }
  EXPECT_EQ(total, 1u);
  check_invariants(b, ex(lod), g);
  auto fresh = built(b);
  EXPECT_EQ(canon(ex(lod)), canon(ex(fresh)));

  // both back to 0, seen through windows: the initial mesh again
  vals[2 * (3 * 32 + 3)] = 0;
  set_value(b, 3, 3, 0, 0);
  vals[2 * (28 * 32 + 28)] = 0;
  set_value(b, 28, 28, 0, 0);
  EXPECT_EQ(lod.update_window(b.map, rx, ry, 1.0, 0), 1u);
  EXPECT_EQ(lod.update_window(b.map, kOx + 28.5 * kRes, kOy + 28.5 * kRes, 1.0, 0), 1u);
  EXPECT_EQ(canon(ex(lod)), initial);
}

TEST(RtinLod, NeighboursAndBvhMatchTheGenericBuilders)
{
  std::mt19937 rng(7);
  const float tiles[] = {0.6f, 2.0f, 100.0f};
  for (int trial = 0; trial < 3; ++trial) {
    Grid g;
    g.nx = 24 + 5 * trial;
    g.ny = 17;
    std::uniform_int_distribution<int> coin(0, 9);
    std::vector<int> diag(g.nx * g.ny), hole(g.nx * g.ny);
    std::vector<std::uint8_t> vals(2 * g.nx * g.ny);
    for (auto & d : diag) {
      d = coin(rng) < 6;
    }
    for (auto & h : hole) {
      h = coin(rng) == 0;
    }
    for (auto & v : vals) {
      v = coin(rng) < 8 ? 0 : 100;
    }
    const int nx = g.nx;
    g.main_diag = [diag, nx](int i, int j) {return diag[j * nx + i] != 0;};
    g.exists = [hole, nx](int i, int j, int) {return hole[j * nx + i] == 0;};
    g.value = [vals, nx](int i, int j, int h) {return vals[2 * (j * nx + i) + h];};
    auto b = make_map(g);
    auto lod = built(b, 0.05f, 5.0f, tiles[trial]);
    for (int round = 0; round < 2; ++round) {   // the second one from the tiles' cache
      auto out = ex(lod);
      navmap::NavMap ref = out;
      ref.rebuild_geometry_accels();
      ASSERT_EQ(ref.navcels.size(), out.navcels.size());
      for (std::size_t c = 0; c < out.navcels.size(); ++c) {
        if (!live(out.navcels[c])) {continue;}
        for (int e = 0; e < 3; ++e) {
          EXPECT_EQ(out.navcels[c].neighbor[e], ref.navcels[c].neighbor[e]) << c << " " << e;
        }
        EXPECT_NEAR(out.navcels[c].area, ref.navcels[c].area, 1e-5);
      }
      ASSERT_TRUE(std::any_of(out.surfaces.begin(), out.surfaces.end(),
        [](const auto & s) {return !s.bvh.empty();}));
      std::uniform_real_distribution<double> ux(-0.5, g.nx + 0.5), uy(-0.5, g.ny + 0.5);
      for (int q = 0; q < 300; ++q) {
        const Eigen::Vector3f p(
          static_cast<float>(kOx + ux(rng) * kRes), static_cast<float>(kOy + uy(rng) * kRes),
          0.5f);
        std::size_t s1 = 0, s2 = 0;
        navmap::NavCelId c1 = 0, c2 = 0;
        Eigen::Vector3f b1, b2, h1, h2;
        const bool f1 = out.locate_navcel(p, s1, c1, b1, &h1);
        const bool f2 = ref.locate_navcel(p, s2, c2, b2, &h2);
        ASSERT_EQ(f1, f2);
        if (f1) {
          EXPECT_TRUE(inside(out, out.navcels[c1], (p.x() - kOx) / kRes,
            (p.y() - kOy) / kRes));
        }
        if (f1) {
          EXPECT_TRUE(inside(ref, ref.navcels[c2], (p.x() - kOx) / kRes,
            (p.y() - kOy) / kRes));
        }
      }
    }
  }
}

TEST(RtinLod, OnlyTheChangedTilesAreWalkedAgain)
{
  Grid g;
  g.nx = g.ny = 64;   // 25.6 m
  std::vector<std::uint8_t> vals(2 * 64 * 64, 0);
  for (int j = 0; j < 64; j += 7) {   // some structure: rows of obstacles
    for (int i = 0; i < 64; ++i) {
      vals[2 * (j * 64 + i)] = (i % 9) ? 200 : 0;
    }
  }
  g.value = [&vals](int i, int j, int h) {return vals[2 * (j * 64 + i) + h];};
  auto b = make_map(g);
  auto lod = built(b, 0.05f, 5.0f, 2.0f);
  RtinLod::Stats st;
  ex(lod, &st);
  const std::size_t tiles = st.tiles;
  EXPECT_GT(tiles, 50u);
  EXPECT_GE(st.tiles_rebuilt, tiles);   // all of them at first (and the empty ones)

  ex(lod, &st);   // nothing changed: nothing walked
  EXPECT_EQ(st.tiles_rebuilt, 0u);

  vals[2 * (30 * 64 + 30)] = 99;   // one change in the middle of a free area
  set_value(b, 30, 30, 0, 99);
  ASSERT_EQ(lod.update(b.map), 1u);
  auto out = ex(lod, &st);
  EXPECT_GE(st.tiles_rebuilt, 1u);
  EXPECT_LE(st.tiles_rebuilt, 6u);   // its tile, the top, and the ones a forced split reaches
  check_invariants(b, out, g);
  EXPECT_EQ(canon(out), canon(ex(built(b, 0.05f, 5.0f, 2.0f))));
}

TEST(RtinLod, RejectsWhatItCannotHandle)
{
  RtinLod lod;
  std::string err;
  {
    navmap::NavMap m;   // no layer
    Grid g;
    auto b = make_map(g);
    EXPECT_FALSE(lod.build(b.map, "missing", RtinLod::Params{}, &err));
    EXPECT_NE(err.find("missing"), std::string::npos);
  }
  {
    // a NavCel off the grid
    Grid g;
    auto b = make_map(g);
    b.map.positions.x[0] += 0.13f;
    EXPECT_FALSE(lod.build(b.map, "cost", RtinLod::Params{}, &err));
    EXPECT_NE(err.find("grid"), std::string::npos);
  }
  {
    // two floors: the same cell twice at different heights
    Grid g;
    g.nx = g.ny = 2;
    auto b = make_map(g);
    auto & m = b.map;
    const auto v0 = m.add_vertex(Eigen::Vector3f(kOx + kRes, kOy, 3.0f));
    const auto v1 = m.add_vertex(Eigen::Vector3f(kOx + 2 * kRes, kOy, 3.0f));
    const auto v2 = m.add_vertex(Eigen::Vector3f(kOx + 2 * kRes, kOy + kRes, 3.0f));
    m.add_navcel_to_surface(0, m.add_navcel(v1, v2, v0));
    auto l = std::dynamic_pointer_cast<LayerU8>(m.layers.get("cost"));
    l->data().push_back(0);
    EXPECT_FALSE(lod.build(m, "cost", RtinLod::Params{}, &err));
    EXPECT_NE(err.find("levels"), std::string::npos);
  }
  EXPECT_FALSE(lod.ready());
  EXPECT_EQ(lod.update(navmap::NavMap{}), 0u);
}

// ─── Changes since the last take (what a receiver needs) ─────────────────────────────────────

namespace
{

// What a receiver kept must equal the full conversion of the output.
void expect_same_msg(
  const navmap_ros_interfaces::msg::NavMap & a, const navmap_ros_interfaces::msg::NavMap & b)
{
  EXPECT_EQ(a.geometry_id, b.geometry_id);
  EXPECT_EQ(a.positions_x, b.positions_x);
  EXPECT_EQ(a.positions_y, b.positions_y);
  EXPECT_EQ(a.positions_z, b.positions_z);
  EXPECT_EQ(a.navcels_v0, b.navcels_v0);
  EXPECT_EQ(a.navcels_v1, b.navcels_v1);
  EXPECT_EQ(a.navcels_v2, b.navcels_v2);
  ASSERT_EQ(a.surfaces.size(), b.surfaces.size());
  for (std::size_t i = 0; i < a.surfaces.size(); ++i) {
    EXPECT_EQ(a.surfaces[i].navcels, b.surfaces[i].navcels) << "surface " << i;
  }
  ASSERT_EQ(a.layers.size(), b.layers.size());
  for (std::size_t i = 0; i < a.layers.size(); ++i) {
    EXPECT_EQ(a.layers[i].name, b.layers[i].name);
    EXPECT_EQ(a.layers[i].data_u8, b.layers[i].data_u8) << a.layers[i].name;
  }
}

}  // namespace

TEST(RtinLod, TheFirstChangesAreTheWholeMap)
{
  Grid g;
  g.nx = g.ny = 8;
  auto b = make_map(g);
  auto lod = built(b);
  lod.extract();
  EXPECT_TRUE(lod.take_changes().full);
  const auto again = lod.take_changes();
  EXPECT_FALSE(again.full);
  EXPECT_TRUE(again.navcels.empty()) << "nothing changed since";
  EXPECT_TRUE(again.surfaces.empty());
}

TEST(RtinLod, PatchesFollowTheOutputExactly)
{
  // Many small tiles, random costs changing in places, sometimes growing a tile past its slack:
  // after every extraction the patch brings the receiver's copy to the output, exactly.
  Grid g;
  g.nx = g.ny = 32;
  auto b = make_map(g);
  auto lod = built(b, 0.05f, 5.0f, 2.0f);
  auto out = lod.extract();
  auto kept = navmap_ros::to_msg(*out);
  lod.take_changes();
  std::mt19937 rng(11);
  std::uniform_int_distribution<int> cell(0, 31), half(0, 1), burst(1, 40);
  int patches = 0, fulls = 0;
  for (int round = 0; round < 60; ++round) {
    const int n = burst(rng);
    const std::uint8_t v = (round % 3 == 0) ? 0 : static_cast<std::uint8_t>(10 + round);
    for (int k = 0; k < n; ++k) {
      set_value(b, cell(rng), cell(rng), half(rng), v);
    }
    lod.update(b.map);
    const auto base = kept.geometry_id;
    out = lod.extract();
    const auto ch = lod.take_changes();
    if (ch.full) {
      kept = navmap_ros::to_msg(*out);
      ++fulls;
    } else {
      const auto p = navmap_ros::to_patch_msg(*out, base, ch.first_vertex, ch.navcels, ch.surfaces);
      ASSERT_TRUE(navmap_ros::apply_patch(p, kept)) << "round " << round;
      ++patches;
      EXPECT_LT(p.navcel_indices.size(), out->navcels.size()) << "a patch, not the whole map";
    }
    expect_same_msg(kept, navmap_ros::to_msg(*out));
    if (HasFatalFailure()) {return;}
  }
  EXPECT_GT(patches, 40);
}

TEST(RtinLod, ChangesAccumulateOverSeveralExtractions)
{
  Grid g;
  g.nx = g.ny = 16;
  auto b = make_map(g);
  auto lod = built(b, 0.05f, 5.0f, 2.0f);
  auto out = lod.extract();
  auto kept = navmap_ros::to_msg(*out);
  lod.take_changes();
  const auto base = kept.geometry_id;
  // Three extractions, one take: the patch carries the three
  for (int k = 0; k < 3; ++k) {
    set_value(b, 2 + 5 * k, 3 + 4 * k, 0, static_cast<std::uint8_t>(100 + k));
    lod.update(b.map);
    out = lod.extract();
  }
  const auto ch = lod.take_changes();
  ASSERT_FALSE(ch.full);
  ASSERT_TRUE(
    navmap_ros::apply_patch(
      navmap_ros::to_patch_msg(*out, base, ch.first_vertex, ch.navcels, ch.surfaces), kept));
  expect_same_msg(kept, navmap_ros::to_msg(*out));
}

TEST(RtinLod, ANewTreeIsAFullChange)
{
  Grid g;
  g.nx = g.ny = 8;
  auto b = make_map(g);
  auto lod = built(b);
  lod.extract();
  lod.take_changes();
  std::string err;
  ASSERT_TRUE(lod.build(b.map, "cost", RtinLod::Params{}, &err)) << err;
  lod.extract();
  EXPECT_TRUE(lod.take_changes().full);
}
