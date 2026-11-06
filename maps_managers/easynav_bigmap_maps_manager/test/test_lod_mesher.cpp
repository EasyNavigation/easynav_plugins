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

#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <utility>

#include "easynav_bigmap_maps_manager/known_map.hpp"
#include "easynav_bigmap_maps_manager/profiler.hpp"
#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/narrow_passages.hpp"

using bigmap_maps_manager::LodMesh;
using bigmap_maps_manager::LodMesher;
using bigmap_maps_manager::LodParams;

namespace
{
// 65x65 nodes (= 2^6 + 1, no padding), 2 m apart: rough terrain, so errors are non-trivial
bigmap_maps_manager::DemGrid make_dem()
{
  bigmap_maps_manager::DemGrid g;
  g.side = 65;
  g.resolution = 2.0;
  g.radius = 64.0;
  g.z.resize(65 * 65);
  for (int j = 0; j < 65; ++j) {
    for (int i = 0; i < 65; ++i) {
      const double x = g.x(i), y = g.y(j);
      g.z[j * 65 + i] = static_cast<float>(
        8.0 * std::sin(x / 17.0) * std::cos(y / 13.0) + 0.6 * std::sin(x / 3.1 + y / 2.3));
    }
  }
  return g;
}

LodParams params()
{
  LodParams p;
  p.high_res_radius = 20.0;
  p.min_size = 2.0;
  p.max_size = 32.0;
  p.base_error = 0.05;
  p.error_growth = 0.1;
  p.low_res_distance = 100.0;
  return p;
}

using Edge = std::pair<uint32_t, uint32_t>;
std::map<Edge, int> edge_use(const LodMesh & m)
{
  std::map<Edge, int> use;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    for (int k = 0; k < 3; ++k) {
      uint32_t a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
      ++use[{std::min(a, b), std::max(a, b)}];
    }
  }
  return use;
}
}  // namespace

TEST(LodMesher, CrackFreeAndManifoldForManyRobotPositions)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  const double half = 64.0;
  for (double rx : {-60.0, -17.3, 0.0, 33.3, 64.0}) {
    for (double ry : {-64.0, -5.0, 0.0, 41.0}) {
      const auto m = mesher.extract(rx, ry, params());
      for (const auto & [e, n] : edge_use(m)) {
        ASSERT_LE(n, 2) << "non-manifold edge";
        if (n == 1) {  // a boundary edge must lie on the outer border: otherwise it is a crack
          auto on_border = [&](uint32_t v) {
              return std::abs(std::abs(m.x[v]) - half) < 1e-3 ||
                     std::abs(std::abs(m.y[v]) - half) < 1e-3;
            };
          ASSERT_TRUE(on_border(e.first) && on_border(e.second))
            << "crack at robot (" << rx << ", " << ry << ")";
        }
      }
    }
  }
}

TEST(LodMesher, CrackFreeUnderSteepPolicies)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  for (double growth : {0.5, 2.0, 10.0}) {
    for (double radius : {0.0, 6.0, 25.0}) {
      LodParams p = params();
      p.high_res_radius = radius;
      p.error_growth = growth;
      p.low_res_distance = radius + 4.0 / growth;  // from very short to long transitions
      p.base_error = 0.01;
      p.max_size = 64.0;
      for (double rx = -64.0; rx <= 64.0; rx += 16.0) {
        const auto m = mesher.extract(rx, 7.0, p);
        for (const auto & [e, n] : edge_use(m)) {
          ASSERT_LE(n, 2);
          if (n == 1) {
            auto on_border = [&](uint32_t v) {
                return std::abs(std::abs(m.x[v]) - 64.0) < 1e-3 ||
                       std::abs(std::abs(m.y[v]) - 64.0) < 1e-3;
              };
            ASSERT_TRUE(on_border(e.first) && on_border(e.second))
              << "crack: growth " << growth << " radius " << radius << " rx " << rx;
          }
        }
      }
    }
  }
}

TEST(Profiler, ReportsTheSectionsSortedByTheTimeTheyTakeAndStartsANewWindow)
{
  bigmap_maps_manager::Profiler prof(3600.0);
  EXPECT_FALSE(prof.due());
  prof.add("cheap", 1.0);
  prof.add("expensive", 30.0);
  prof.add("expensive", 10.0);
  {bigmap_maps_manager::ProfScope s(&prof, "scoped");}
  {bigmap_maps_manager::ProfScope s(nullptr, "never");}    // a null profiler does nothing
  const std::string r = prof.report_and_reset("title");
  EXPECT_NE(r.find("title"), std::string::npos);
  EXPECT_NE(r.find("scoped"), std::string::npos);
  EXPECT_EQ(r.find("never"), std::string::npos);
  EXPECT_LT(r.find("expensive"), r.find("cheap"));   // sorted: the costly one first
  // the mean of "expensive" is 20 ms over 2 calls, the worst 30
  EXPECT_NE(r.find("    2     20.00     30.00"), std::string::npos) << r;
  EXPECT_EQ(prof.report_and_reset("again").find("expensive"), std::string::npos);
}

TEST(KnownMap, DistanceIsTheExactEuclideanDistanceToTheNearestMarkedNode)
{
  bigmap_maps_manager::KnownMap k(65, 2.0);
  EXPECT_TRUE(k.empty());
  k.mark(-30.0, 10.0, 1.0);
  k.mark(20.0, -40.0, 3.0);
  k.update();
  EXPECT_FALSE(k.empty());
  for (int j = 0; j < 65; j += 4) {
    for (int i = 0; i < 65; i += 3) {
      double best = 1e9;
      for (int q = 0; q < 65; ++q) {
        for (int p = 0; p < 65; ++p) {
          if (k.known(p, q)) {best = std::min(best, std::hypot(p - i, q - j) * 2.0);}
        }
      }
      EXPECT_NEAR(k.distance(i, j), best, 1e-3) << i << "," << j;
    }
  }
}

TEST(KnownMap, UpdateWithinIgnoresTheTraversedNodesBeyondTheRadiusButKeepsThemMarked)
{
  bigmap_maps_manager::KnownMap k(65, 2.0);
  k.mark(-40.0, 0.0, 1.0);   // far from (30, 0)
  k.mark(20.0, 0.0, 1.0);    // 10 m from it
  k.update_within(30.0, 0.0, 15.0);
  const int c = 32;
  EXPECT_NEAR(k.distance(c + 10, c), 0.0, 1e-6);          // x = 20: a source
  EXPECT_GT(k.distance(c - 20, c), 50.0);                  // x = -40: not a source: far from 20
  EXPECT_TRUE(k.known(c - 20, c));                         // but still marked
  EXPECT_TRUE(k.near_marked(c - 20, c + 1, 1));
  EXPECT_FALSE(k.near_marked(c - 20, c + 3, 1));
  k.update_within(30.0, 0.0, 1.0);                         // nothing within 1 m
  EXPECT_EQ(k.sources(), 0u);
  EXPECT_TRUE(std::isinf(k.distance(c, c)));
  k.update();                                              // all of them again
  EXPECT_EQ(k.sources(), k.count());
  EXPECT_NEAR(k.distance(c - 20, c), 0.0, 1e-6);
}

TEST(KnownMap, ForgetOutsideUnmarksForGoodUntilTheRobotDrivesOverItAgain)
{
  bigmap_maps_manager::KnownMap k(65, 2.0);
  k.mark(-40.0, 0.0, 1.0);
  k.mark(20.0, 0.0, 1.0);
  const std::size_t before = k.count();
  const int c = 32;
  EXPECT_EQ(k.forget_outside(30.0, 0.0, 15.0), before / 2);  // the one at -40 m (70 m away)
  k.update();
  EXPECT_FALSE(k.known(c - 20, c));
  EXPECT_TRUE(k.known(c + 10, c));
  EXPECT_GT(k.distance(c - 20, c), 50.0);                      // no longer fine there
  // the robot is back near x = -40: nothing returns by itself ...
  k.forget_outside(-40.0, 0.0, 15.0);                          // the other one is the far one now
  k.update();
  EXPECT_FALSE(k.known(c - 20, c));
  EXPECT_EQ(k.count(), 0u);
  // ... until it drives over it again
  k.mark(-40.0, 0.0, 1.0);
  k.update();
  EXPECT_TRUE(k.known(c - 20, c));
  EXPECT_NEAR(k.distance(c - 20, c), 0.0, 1e-6);
}

TEST(LodMesher, GroundTheRobotHasDrivenOverStaysFineWhereverTheRobotIsAndStaysCrackFree)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  bigmap_maps_manager::KnownMap known(dem.side, dem.resolution);
  // a trail along y = -40, x from -60 to -20
  for (double x = -60.0; x <= -20.0; x += 1.0) {
    known.mark(x, -40.0, 1.0);
  }
  known.update();
  LodParams p = params();
  p.known_margin = 2.0;

  const double rx = 50.0, ry = 50.0;  // the robot is far away
  const auto without = mesher.extract(rx, ry, p);
  const auto with = mesher.extract(rx, ry, p, &known);
  auto leaf_at = [](const LodMesh & m, double x, double y) {
      double best = 1e9, leg = -1.0;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        double cx = 0, cy = 0;
        for (int k = 0; k < 3; ++k) {
          cx += m.x[m.tri[3 * t + k]] / 3;
          cy += m.y[m.tri[3 * t + k]] / 3;
        }
        const double d = std::hypot(cx - x, cy - y);
        if (d < best) {best = d; leg = m.leg[t];}
      }
      return leg;
    };
  EXPECT_GT(leaf_at(without, -40.0, -40.0), p.min_size) << "the point is far: it should be coarse";
  EXPECT_DOUBLE_EQ(leaf_at(with, -40.0, -40.0), p.min_size);
  EXPECT_GT(with.triangle_count(), without.triangle_count());

  // no cracks: an edge used once must lie on the border of the domain
  for (const auto & [e, n] : edge_use(with)) {
    ASSERT_LE(n, 2);
    if (n == 1) {
      auto on_border = [&](uint32_t v) {
          return std::abs(std::abs(with.x[v]) - 64.0) < 1e-3 ||
                 std::abs(std::abs(with.y[v]) - 64.0) < 1e-3;
        };
      ASSERT_TRUE(on_border(e.first) && on_border(e.second)) << "T-junction with a trail";
    }
  }
}

TEST(LodMesher, TraversedGroundFadesWithinTheShortFalloffAndOnlyThere)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  bigmap_maps_manager::KnownMap known(dem.side, dem.resolution);
  for (double x = -60.0; x <= -20.0; x += 1.0) {
    known.mark(x, -40.0, 1.0);
  }
  known.update();
  LodParams p = params();
  p.known_margin = 6.0;
  p.known_falloff = (p.low_res_distance - p.high_res_radius) / 4.0;  // fade over 4 m

  const double rx = 50.0, ry = 50.0;
  const auto with = mesher.extract(rx, ry, p, &known);
  const auto without = mesher.extract(rx, ry, p);
  auto leaf_at = [](const LodMesh & m, double x, double y) {
      double best = 1e9, leg = -1.0;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        double cx = 0, cy = 0;
        for (int k = 0; k < 3; ++k) {
          cx += m.x[m.tri[3 * t + k]] / 3;
          cy += m.y[m.tri[3 * t + k]] / 3;
        }
        const double d = std::hypot(cx - x, cy - y);
        if (d < best) {best = d; leg = m.leg[t];}
      }
      return leg;
    };
  EXPECT_DOUBLE_EQ(leaf_at(with, -40.0, -40.0), p.min_size);
  EXPECT_DOUBLE_EQ(leaf_at(with, -40.0, -36.0), p.min_size);        // inside the radius (6 m)
  // beyond the radius the fade is much faster than the robot's own (known_falloff = 1): coarser
  // there and far fewer triangles in all (a crack-free mesh can only grow about one leg per
  // 1.8 legs of distance, so it cannot be a step)
  LodParams slow = p;
  slow.known_falloff = 1.0;
  const auto slow_mesh = mesher.extract(rx, ry, slow, &known);
  EXPECT_GT(leaf_at(with, -40.0, -40.0 + 6.0 + 8.0), leaf_at(slow_mesh, -40.0, -26.0));
  EXPECT_LT(with.triangle_count(), slow_mesh.triangle_count());
  EXPECT_GT(with.triangle_count(), without.triangle_count());
  for (const auto & [e, n] : edge_use(with)) {
    ASSERT_LE(n, 2);
    if (n == 1) {
      auto on_border = [&](uint32_t v) {
          return std::abs(std::abs(with.x[v]) - 64.0) < 1e-3 ||
                 std::abs(std::abs(with.y[v]) - 64.0) < 1e-3;
        };
      ASSERT_TRUE(on_border(e.first) && on_border(e.second)) << "T-junction";
    }
  }
}

TEST(LodMesher, TheTwoCirclesKeepTheTraversedGroundFineOnlyInsideTheLargeOne)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  bigmap_maps_manager::KnownMap known(dem.side, dem.resolution);
  for (double x = -60.0; x <= 40.0; x += 1.0) {
    known.mark(x, -30.0, 1.0);
                                                                         }  // a long trail
  LodParams p = params();
  p.high_res_radius = 10.0;
  p.low_res_distance = 30.0;
  p.known_margin = 2.0;
  p.known_falloff = (p.low_res_distance - p.high_res_radius) / 3.0;
  auto leaf_at = [](const LodMesh & m, double x, double y) {
      double best = 1e9, leg = -1.0;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        double cx = 0, cy = 0;
        for (int k = 0; k < 3; ++k) {
          cx += m.x[m.tri[3 * t + k]] / 3;
          cy += m.y[m.tri[3 * t + k]] / 3;
        }
        const double d = std::hypot(cx - x, cy - y);
        if (d < best) {best = d; leg = m.leg[t];}
      }
      return leg;
    };
  const double rx = 0.0, ry = 30.0;  // the robot is 60 m from the trail's centre

  // mode persistent: the whole trail is fine
  known.update();
  const auto persistent = mesher.extract(rx, ry, p, &known);
  EXPECT_DOUBLE_EQ(leaf_at(persistent, -50.0, -30.0), p.min_size);
  EXPECT_DOUBLE_EQ(leaf_at(persistent, 0.0, -30.0), p.min_size);

  // mode circles, large circle 70 m around the robot: the trail within it is fine, the rest is
  // coarse
  known.update_within(rx, ry, 70.0);
  const auto circles = mesher.extract(rx, ry, p, &known);
  EXPECT_DOUBLE_EQ(leaf_at(circles, 0.0, -30.0), p.min_size);   // 60 m away: inside 70
  EXPECT_GT(leaf_at(circles, -50.0, -30.0), p.min_size);        // 78 m away: forgotten

  known.update_within(rx, ry, 50.0);                            // the large circle is smaller
  const auto tight = mesher.extract(rx, ry, p, &known);
  EXPECT_GT(leaf_at(tight, 0.0, -30.0), p.min_size);            // 60 m away: outside 50
  EXPECT_LT(tight.triangle_count(), circles.triangle_count());
  EXPECT_LT(circles.triangle_count(), persistent.triangle_count());

  // none of them has T-junctions
  for (const auto * m : {&persistent, &circles, &tight}) {
    for (const auto & [e, n] : edge_use(*m)) {
      ASSERT_LE(n, 2);
      if (n == 1) {
        auto on_border = [&](uint32_t v) {
            return std::abs(std::abs(m->x[v]) - 64.0) < 1e-3 ||
                   std::abs(std::abs(m->y[v]) - 64.0) < 1e-3;
          };
        ASSERT_TRUE(on_border(e.first) && on_border(e.second)) << "T-junction";
      }
    }
  }
}

TEST(LodPolicy, SizeCapIsNearFineFarCoarseAndGeometricBetween)
{
  LodParams p;
  p.high_res_radius = 10.0;
  p.low_res_distance = 200.0;
  p.min_size = 1.0;
  p.max_size = 32.0;
  using bigmap_maps_manager::size_cap;
  EXPECT_DOUBLE_EQ(size_cap(p, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(size_cap(p, 10.0), 1.0);
  EXPECT_DOUBLE_EQ(size_cap(p, 200.0), 32.0);
  EXPECT_DOUBLE_EQ(size_cap(p, 1000.0), 32.0);
  // halfway in distance -> geometric mean of the sizes: sqrt(1 * 32)
  EXPECT_NEAR(size_cap(p, 105.0), std::sqrt(32.0), 1e-9);
  double prev = 0.0;
  for (double d = 0.0; d <= 300.0; d += 1.0) {  // never decreases with distance
    const double c = size_cap(p, d);
    ASSERT_GE(c, prev);
    prev = c;
  }
}

TEST(LodMesher, TriangulationCoversTheWholeDomainExactlyOnce)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  const auto m = mesher.extract(10.0, -20.0, params());
  double area = 0.0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
    const double cross = (m.x[b] - m.x[a]) * (m.y[c] - m.y[a]) -
      (m.y[b] - m.y[a]) * (m.x[c] - m.x[a]);
    ASSERT_GT(cross, 0.0) << "triangle not CCW";
    area += 0.5 * cross;
  }
  EXPECT_NEAR(area, 128.0 * 128.0, 1e-3);
}

TEST(LodMesher, FineNearRobotCoarseFarAndFewerTriangles)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  const auto p = params();
  const auto m = mesher.extract(-40.0, -40.0, p);

  const std::size_t full = 2 * 64 * 64;
  EXPECT_LT(m.triangle_count(), full / 3);

  float far_leg = 0.0f;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
    const double cx = (m.x[a] + m.x[b] + m.x[c]) / 3.0, cy = (m.y[a] + m.y[b] + m.y[c]) / 3.0;
    const double d = std::hypot(cx + 40.0, cy + 40.0);
    if (d < p.high_res_radius * 0.5) {
      ASSERT_LE(m.leg[t], dem.resolution + 1e-4) << "not full resolution near the robot";
    }
    far_leg = std::max(far_leg, m.leg[t]);
  }
  EXPECT_GT(far_leg, 4.0f * dem.resolution);
}

TEST(LodMesher, MeshFollowsRobotAndVerticesAreDemNodes)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  const auto a = mesher.extract(-50.0, 0.0, params());
  const auto b = mesher.extract(50.0, 0.0, params());
  EXPECT_NE(a.triangle_count() + a.x.size(), 0u);
  // vertices at the same place keep exact DEM heights
  for (const auto * m : {&a, &b}) {
    for (std::size_t v = 0; v < m->x.size(); ++v) {
      const int i = static_cast<int>(std::lround(m->x[v] / 2.0)) + 32;
      const int j = static_cast<int>(std::lround(m->y[v] / 2.0)) + 32;
      ASSERT_FLOAT_EQ(m->z[v], dem.at(i, j));
    }
  }
  // each mesh is denser around its own robot position
  auto count_near = [](const LodMesh & m, double rx) {
      int n = 0;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        if (std::abs(m.x[m.tri[3 * t]] - rx) < 15.0) {++n;}
      }
      return n;
    };
  EXPECT_GT(count_near(a, -50.0), count_near(b, -50.0));
  EXPECT_GT(count_near(b, 50.0), count_near(a, 50.0));
}

TEST(LodMesher, NonPowerOfTwoDemIsPaddedAndCulled)
{
  auto dem = make_dem();
  // 41x41 real nodes inside a 65 tile: crop
  bigmap_maps_manager::DemGrid g;
  g.side = 41;
  g.resolution = 2.0;
  g.z.resize(41 * 41);
  for (int j = 0; j < 41; ++j) {
    for (int i = 0; i < 41; ++i) {
      g.z[j * 41 + i] = dem.at(i + 12, j + 12);
    }
  }
  const LodMesher mesher(g);
  EXPECT_EQ(mesher.error_map().tile_size(), 64);
  const auto m = mesher.extract(0.0, 0.0, params());
  double area = 0.0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
    area += 0.5 * ((m.x[b] - m.x[a]) * (m.y[c] - m.y[a]) - (m.y[b] - m.y[a]) * (m.x[c] - m.x[a]));
  }
  EXPECT_GE(area, 80.0 * 80.0 - 1e-3);  // real domain fully covered (may overhang the padding)
}

TEST(LodMesher, LargeDemWithNearFineFarCoarsePolicy)
{
  // 1001 x 1001 nodes at 1 m (padded to a 1025 tile): the production-size case
  bigmap_maps_manager::DemGrid g;
  g.side = 1001;
  g.resolution = 1.0;
  g.radius = 500.0;
  g.z.resize(static_cast<std::size_t>(g.side) * g.side);
  for (int j = 0; j < g.side; ++j) {
    for (int i = 0; i < g.side; ++i) {
      const double x = g.x(i), y = g.y(j);
      g.z[static_cast<std::size_t>(j) * g.side + i] = static_cast<float>(
        15.0 * std::sin(x / 80.0) * std::cos(y / 60.0) + 0.3 * std::sin(x / 3.0 + y / 2.0));
    }
  }
  const LodMesher mesher(g);
  EXPECT_EQ(mesher.error_map().tile_size(), 1024);

  LodParams p;  // the defaults: 1 m within 10 m ... 32 m beyond 200 m
  const auto m = mesher.extract(0.0, 0.0, p);
  EXPECT_GT(m.triangle_count(), 500u);
  EXPECT_LT(m.triangle_count(), 60000u);  // vs 2 million at full resolution

  double near_max = 0.0, far_min = 1e9;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
    const double d = std::hypot((m.x[a] + m.x[b] + m.x[c]) / 3.0, (m.y[a] + m.y[b] + m.y[c]) / 3.0);
    if (d < 5.0) {near_max = std::max<double>(near_max, m.leg[t]);}
    if (d > 300.0) {far_min = std::min<double>(far_min, m.leg[t]);}
  }
  EXPECT_LE(near_max, 1.0 + 1e-4);   // full resolution next to the robot
  EXPECT_GT(far_min, 8.0);           // coarse far away (cap there: 32 m)
}

TEST(LodMesher, ErrorDrivenMeshStaysWithinToleranceAtEveryDemNode)
{
  // With a pure error criterion no DEM node is far from the plane of the triangle that covers it.
  // RTIN applies the tolerance to the error at each hypotenuse midpoint (propagated up from its
  // descendants), so the error at other nodes is not strictly bounded by it: in practice up to
  // ~1.5x. A wrong error map gives a much larger error (it was 9 m for any tolerance).
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  LodParams p;
  p.high_res_radius = 0.0;
  p.low_res_distance = 0.0;  // beyond high_res: size cap = max_size
  p.min_size = dem.resolution;
  p.max_size = 1e9;
  p.error_growth = 0.0;
  for (double tol : {0.3, 1.0, 2.5}) {
    p.base_error = tol;
    const auto m = mesher.extract(1e6, 1e6, p);  // robot far away: only the error decides
    ASSERT_LT(m.triangle_count(), 2u * 64 * 64) << "tolerance " << tol << " did not simplify";
    double worst = 0.0;
    std::size_t covered = 0;
    for (std::size_t t = 0; t < m.triangle_count(); ++t) {
      const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
      const double x0 = m.x[a], y0 = m.y[a], x1 = m.x[b], y1 = m.y[b], x2 = m.x[c], y2 = m.y[c];
      const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
      const int i0 = static_cast<int>(std::floor((std::min({x0, x1, x2}) + 64.0) / 2.0));
      const int i1 = static_cast<int>(std::ceil((std::max({x0, x1, x2}) + 64.0) / 2.0));
      const int j0 = static_cast<int>(std::floor((std::min({y0, y1, y2}) + 64.0) / 2.0));
      const int j1 = static_cast<int>(std::ceil((std::max({y0, y1, y2}) + 64.0) / 2.0));
      for (int j = std::max(j0, 0); j <= std::min(j1, 64); ++j) {
        for (int i = std::max(i0, 0); i <= std::min(i1, 64); ++i) {
          const double px = dem.x(i), py = dem.y(j);
          const double l0 = ((y1 - y2) * (px - x2) + (x2 - x1) * (py - y2)) / det;
          const double l1 = ((y2 - y0) * (px - x2) + (x0 - x2) * (py - y2)) / det;
          const double l2 = 1.0 - l0 - l1;
          if (l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9) {continue;}
          const double z = l0 * m.z[a] + l1 * m.z[b] + l2 * m.z[c];
          worst = std::max(worst, std::abs(z - dem.at(i, j)));
          ++covered;
        }
      }
    }
    EXPECT_GT(covered, 65u * 65u);  // every node covered (some by several triangles on edges)
    EXPECT_LE(worst, 1.75 * tol + 1e-4) << "tolerance " << tol << ", worst " << worst;
  }
}

// ---------------------------------------------------------------------------------------------
// Dynamic layer: what the sensors see raises the NavCels they fall in

#include "easynav_bigmap_maps_manager/dynamic_layer.hpp"

using bigmap_maps_manager::DynamicLayer;
using bigmap_maps_manager::DynamicParams;

namespace
{
// the unit tests below feed single points: no vertical extent can be asked of them
DynamicParams NoExtent()
{
  DynamicParams p;
  p.min_vertical_extent = 0.0;
  p.confirm_time = 0.0;
  return p;
}

// the default rules, but taking a single frame as enough
DynamicParams Instant()
{
  DynamicParams p;
  p.confirm_time = 0.0;
  return p;
}
}  // namespace
using bigmap_maps_manager::TriangleLocator;

TEST(DynamicLayer, LocatorFindsTheTriangleOfEveryPoint)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  const auto m = mesher.extract(0.0, 0.0, params());
  const TriangleLocator loc(m);
  for (double x = -63.3; x < 64.0; x += 9.7) {
    for (double y = -63.9; y < 64.0; y += 11.3) {
      const int t = loc.locate(x, y);
      ASSERT_GE(t, 0) << x << ", " << y;
      const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
      const double det = (m.y[b] - m.y[c]) * (m.x[a] - m.x[c]) + (m.x[c] - m.x[b]) *
        (m.y[a] - m.y[c]);
      const double l0 = ((m.y[b] - m.y[c]) * (x - m.x[c]) + (m.x[c] - m.x[b]) * (y - m.y[c])) / det;
      const double l1 = ((m.y[c] - m.y[a]) * (x - m.x[c]) + (m.x[a] - m.x[c]) * (y - m.y[c])) / det;
      EXPECT_GE(l0, -1e-6);
      EXPECT_GE(l1, -1e-6);
      EXPECT_GE(1.0 - l0 - l1, -1e-6);
    }
  }
  EXPECT_EQ(loc.locate(500.0, 0.0), -1);
}

TEST(DynamicLayer, RaisesTheNavCelToWhatTheSensorMeasuresRelativeToTheRobot)
{
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  const auto m = mesher.extract(0.0, 0.0, params());
  const TriangleLocator loc(m);
  // flat terrain: only what the sensor measured decides the height
  const DynamicLayer::Surface terrain = [](double, double) {return 2.0;};
  DynamicParams p = NoExtent();

  DynamicLayer layer;
  // robot at (10, -6) facing +y (yaw 90 deg); an object 3 m ahead: in the robot frame x = 3, and
  // 0.8..1.2 m above its ground. In the map that is (10, -3).
  std::vector<std::array<float, 3>> pts = {{3.0f, 0.0f, 0.8f}, {3.0f, 0.05f, 1.2f},
    {3.1f, 0.0f, 1.0f}};
  EXPECT_TRUE(layer.ingest(pts, 10.0, -6.0, M_PI / 2.0, 0.0, terrain, p));
  const auto tiles = layer.tiles(m, loc);
  ASSERT_GE(tiles.size(), 1u);

  const double ground = 2.0;
  const int expected_tri = loc.locate(10.0, -3.0);
  bool found = false;
  for (const auto & t : tiles) {
    if (static_cast<int>(t.triangle) == expected_tri) {
      found = true;
      // the highest detection (1.2 m above the robot's ground), unless the terrain there is higher
      const auto a = m.tri[3 * t.triangle], b = m.tri[3 * t.triangle + 1],
        c = m.tri[3 * t.triangle + 2];
      const float highest = std::max({m.z[a], m.z[b], m.z[c]});
      EXPECT_NEAR(t.z_top, std::max<double>(ground + 1.2, highest), 1e-3);
      EXPECT_GT(t.above_surface, 0.0f);
    }
  }
  EXPECT_TRUE(found) << "the NavCel under the object was not raised";
}

TEST(DynamicLayer, IgnoresTheGroundAndFarPointsAndForgetsWhatIsNotSeenAgain)
{
  const DynamicLayer::Surface terrain = [](double, double) {return 0.0;};
  DynamicParams p = NoExtent();
  p.forget_time = 2.0;
  DynamicLayer layer;
  // ground points (relative to the terrain), too low, and too far
  std::vector<std::array<float, 3>> ground;
  for (float x = 0.5f; x < 4.0f; x += 0.3f) {
    ground.push_back({x, 0.0f, 0.0f});
  }
  ground.push_back({2.0f, 1.0f, 0.1f});        // 0.1 m above: below min_height
  ground.push_back({20.0f, 0.0f, 1.0f});       // 20 m away: beyond max_range
  ground.push_back({2.0f, 0.0f, 5.0f});        // above max_height
  EXPECT_FALSE(layer.ingest(ground, 0.0, 0.0, 0.0, 0.0, terrain, p));
  EXPECT_EQ(layer.size(), 0u);

  EXPECT_TRUE(layer.ingest({{2.0f, 0.0f, 1.0f}}, 0.0, 0.0, 0.0, 10.0, terrain, p));
  EXPECT_EQ(layer.size(), 1u);
  EXPECT_FALSE(layer.expire(11.0, p));   // seen 1 s ago
  layer.ingest({{2.0f, 0.0f, 1.0f}}, 0.0, 0.0, 0.0, 11.5, terrain, p);  // seen again: refreshed
  EXPECT_FALSE(layer.expire(13.0, p));
  EXPECT_TRUE(layer.expire(14.0, p));    // 2.5 s without seeing it
  EXPECT_EQ(layer.size(), 0u);
}

TEST(DynamicLayer, HeightsBelowTheReferenceAreNotClippedToZero)
{
  // a pit: the terrain is 6 m below the DEM's reference; an object 1 m high stands on its floor
  bigmap_maps_manager::DemGrid g;
  g.side = 33;
  g.resolution = 2.0;
  g.z.assign(33 * 33, -6.0f);
  const LodMesher mesher(g);
  const auto m = mesher.extract(0.0, 0.0, params());
  const TriangleLocator loc(m);
  const DynamicLayer::Surface terrain = [&g](double x, double y) {return g.height_at(x, y);};
  DynamicLayer layer;
  ASSERT_TRUE(layer.ingest({{3.0f, 0.0f, 1.0f}}, 0.0, 0.0, 0.0, 0.0, terrain, NoExtent()));
  const auto tiles = layer.tiles(m, loc);
  ASSERT_EQ(tiles.size(), 1u);
  EXPECT_NEAR(tiles[0].z_top, -5.0f, 1e-3);          // -6 + 1, not clipped to 0
  EXPECT_NEAR(tiles[0].above_surface, 1.0f, 1e-3);
}

TEST(DynamicLayer, HeightIsMeasuredAlongTheNavCelNormalSoARampIsNotAnObstacle)
{
  // a 26.6 degree ramp, z = 0.5 x (slope 0.5): its NavCels are tilted
  bigmap_maps_manager::DemGrid g;
  g.side = 65;
  g.resolution = 2.0;
  g.z.resize(65 * 65);
  for (int j = 0; j < 65; ++j) {
    for (int i = 0; i < 65; ++i) {
      g.z[j * 65 + i] = static_cast<float>(0.5 * g.x(i));
    }
  }
  const LodMesher mesher(g);
  const auto m = mesher.extract(-20.0, 0.0, params());
  const TriangleLocator loc(m);
  const DynamicLayer::Surface terrain = [&g](double x, double y) {return g.height_at(x, y);};
  const double cos_slope = 1.0 / std::sqrt(1.0 + 0.25);  // 0.894

  // robot on the ramp at (-20, 0) facing +x; the ramp 3 m ahead is 1.5 m higher
  auto count_after = [&](float vertical_offset) {
      DynamicLayer layer;
      DynamicParams p = NoExtent();  // min_height 0.3 along the normal
      layer.ingest(
        {{3.0f, 0.0f, 1.5f + vertical_offset}}, -20.0, 0.0, 0.0, 0.0, terrain, p, &m, &loc);
      return layer.size();
    };
  EXPECT_EQ(count_after(0.0f), 0u);    // the ramp's own surface: not an obstacle
  EXPECT_EQ(count_after(0.31f), 0u);   // 0.31 m vertical = 0.277 m along the normal: below 0.3
  EXPECT_EQ(count_after(0.40f), 1u);   // 0.40 vertical = 0.358 normal: an obstacle
  // (the vertical criterion would have accepted the 0.31 m one: 0.31 >= 0.3)
  EXPECT_GT(0.31, 0.3);
  EXPECT_LT(0.31 * cos_slope, 0.3);

  // and the NavCel is raised to the measured height, absolute, as before
  DynamicLayer layer;
  layer.ingest({{3.0f, 0.0f, 1.9f}}, -20.0, 0.0, 0.0, 0.0, terrain, NoExtent(), &m, &loc);
  const auto tiles = layer.tiles(m, loc);
  ASSERT_EQ(tiles.size(), 1u);
  // measured: ground at the robot (-10) + 1.9; a NavCel is never lowered below its highest vertex
  const auto t0 = tiles[0].triangle;
  const double highest = std::max({m.z[m.tri[3 * t0]], m.z[m.tri[3 * t0 + 1]],
        m.z[m.tri[3 * t0 + 2]]});
  EXPECT_NEAR(tiles[0].z_top, std::max(0.5 * -20.0 + 1.9, highest), 1e-3);
}

TEST(DynamicLayer, TheRobotsInclinationIsTakenIntoAccountSoSlopesAreNotObstacles)
{
  // 26.6 degree ramp z = 0.5 x; the robot is on it, pitched up along the slope
  bigmap_maps_manager::DemGrid g;
  g.side = 65;
  g.resolution = 2.0;
  g.z.resize(65 * 65);
  for (int j = 0; j < 65; ++j) {
    for (int i = 0; i < 65; ++i) {
      g.z[j * 65 + i] = static_cast<float>(0.5 * g.x(i));
    }
  }
  const LodMesher mesher(g);
  const auto m = mesher.extract(-20.0, 0.0, params());
  const TriangleLocator loc(m);
  const DynamicLayer::Surface terrain = [&g](double x, double y) {return g.height_at(x, y);};
  const double slope = std::atan(0.5);
  // rotation about +y by -slope: the body x axis points up the ramp (nose up)
  const std::array<double, 4> nose_up = {0.0, std::sin(-slope / 2.0), 0.0, std::cos(slope / 2.0)};

  // the ground seen in the robot's own frame is its x-y plane: ahead, behind and to the sides
  const std::vector<std::array<float, 3>> ground = {
    {3.0f, 0.0f, 0.0f}, {-3.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}, {0.0f, -2.0f, 0.0f},
    {4.0f, 1.0f, 0.0f}, {-4.0f, -1.0f, 0.0f}};

  DynamicLayer tilted;
  tilted.ingest(ground, -20.0, 0.0, nose_up, 0.0, terrain, NoExtent(), &m, &loc);
  EXPECT_EQ(tilted.size(), 0u) << "the ramp the robot is on was taken for obstacles";

  // treating the same points as if the robot were level (what was done before) invents obstacles
  DynamicLayer level;
  level.ingest(ground, -20.0, 0.0, 0.0, 0.0, terrain, NoExtent(), &m, &loc);
  EXPECT_GT(level.size(), 0u);

  // a real object on the ramp: 0.45 m above its surface, along the normal (the robot's z axis)
  DynamicLayer object;
  object.ingest({{3.0f, 0.0f, 0.45f}}, -20.0, 0.0, nose_up, 0.0, terrain, NoExtent(), &m,
    &loc);
  EXPECT_EQ(object.size(), 1u);
  DynamicLayer small;
  small.ingest({{3.0f, 0.0f, 0.20f}}, -20.0, 0.0, nose_up, 0.0, terrain, NoExtent(), &m, &loc);
  EXPECT_EQ(small.size(), 0u);  // 0.2 m: below the 0.3 m of the YAML
}

TEST(DynamicLayer, GroundTheTerrainModelPutsTooLowIsNotAnObstacleButAWallIs)
{
  // flat terrain at z = 0, but the sensor sees the real ground 0.45 m higher (a DEM that is a few
  // decimetres off, as every DEM is): a flat patch of ground, then a wall (an object)
  bigmap_maps_manager::DemGrid g;
  g.side = 65;
  g.resolution = 2.0;
  g.z.assign(65 * 65, 0.0f);
  const LodMesher mesher(g);
  const auto m = mesher.extract(0.0, 0.0, params());
  const TriangleLocator loc(m);
  const DynamicLayer::Surface terrain = [](double, double) {return 0.0;};

  std::vector<std::array<float, 3>> high_ground;
  for (float x = 1.0f; x < 4.0f; x += 0.1f) {
    for (float y = -1.0f; y < 1.0f; y += 0.1f) {
      high_ground.push_back({x, y, 0.45f + 0.02f * std::sin(10.0f * x + 7.0f * y)});
    }
  }
  DynamicLayer ground;
  ground.ingest(high_ground, 0.0, 0.0, 0.0, 0.0, terrain, Instant(), &m, &loc);
  EXPECT_EQ(ground.size(), 0u) << "flat ground 0.45 m above the model was taken for obstacles";

  // without the extent rule it would be (the height rule alone cannot tell them apart)
  DynamicLayer naive;
  DynamicParams no_rules = NoExtent();
  no_rules.ground_fit = false;
  naive.ingest(high_ground, 0.0, 0.0, 0.0, 0.0, terrain, no_rules, &m, &loc);
  EXPECT_GT(naive.size(), 0u);

  // a wall: its face spans from the ground (0.45) up to 1.45
  std::vector<std::array<float, 3>> wall;
  for (float z = 0.45f; z <= 1.45f; z += 0.05f) {
    wall.push_back({3.0f, 0.0f, z});
  }
  DynamicLayer obstacle;
  obstacle.ingest(wall, 0.0, 0.0, 0.0, 0.0, terrain, Instant(), &m, &loc);
  EXPECT_GE(obstacle.size(), 1u);

  // a ramp's own surface on a 26 degree slope: within one 0.25 m cell it spans 0.11 m < 0.2 m
  std::vector<std::array<float, 3>> slope_ground;
  for (float x = 1.0f; x < 4.0f; x += 0.05f) {
    slope_ground.push_back({x, 0.0f, 0.5f * x + 0.5f});
  }
  DynamicLayer slope;
  slope.ingest(slope_ground, 0.0, 0.0, 0.0, 0.0, terrain, Instant(), &m, &loc);
  EXPECT_EQ(slope.size(), 0u) << "the extent of a steep ground cell must stay below the rule";
}

TEST(DynamicLayer, TheSensorsOwnGroundPlaneKeepsARampFromBeingAnObstacleEvenIfTheMapIsWrong)
{
  // the map says flat, the sensor sees a 30 degree ramp ahead: with ground_fit the ramp is the
  // plane the points lie on; a box standing on it is not
  bigmap_maps_manager::DemGrid g;
  g.side = 65;
  g.resolution = 2.0;
  g.z.assign(65 * 65, 0.0f);
  const LodMesher mesher(g);
  const auto m = mesher.extract(0.0, 0.0, params());
  const TriangleLocator loc(m);
  const DynamicLayer::Surface terrain = [](double, double) {return 0.0;};
  const float k = std::tan(30.0f * static_cast<float>(M_PI) / 180.0f);

  std::vector<std::array<float, 3>> ramp;
  for (float x = 0.6f; x < 4.0f; x += 0.05f) {
    for (float y = -1.2f; y < 1.2f; y += 0.05f) {
      ramp.push_back({x, y, k * x});
    }
  }
  DynamicParams p = NoExtent();   // no extent rule: only the plane can save the ramp
  p.ground_fit = false;
  DynamicLayer naive;
  naive.ingest(ramp, 0.0, 0.0, 0.0, 0.0, terrain, p, &m, &loc);
  EXPECT_GT(naive.size(), 0u) << "without the fit a wrong map turns the ramp into obstacles";

  p.ground_fit = true;
  DynamicLayer fitted;
  fitted.ingest(ramp, 0.0, 0.0, 0.0, 0.0, terrain, p, &m, &loc);
  EXPECT_EQ(fitted.size(), 0u) << "the ramp's own plane was taken for obstacles";

  // a 0.6 m box on the ramp at x = 3
  auto with_box = ramp;
  for (float z = 0.0f; z <= 0.6f; z += 0.05f) {
    for (float y = -0.2f; y <= 0.2f; y += 0.05f) {
      with_box.push_back({3.0f, y, k * 3.0f + z});
    }
  }
  DynamicLayer boxed;
  boxed.ingest(with_box, 0.0, 0.0, 0.0, 0.0, terrain, p, &m, &loc);
  EXPECT_GE(boxed.size(), 1u);
}

TEST(DynamicLayer, ADetectionNeedsToPersistInTheSameCellToBeConfirmed)
{
  bigmap_maps_manager::DemGrid g;
  g.side = 33;
  g.resolution = 2.0;
  g.z.assign(33 * 33, 0.0f);
  const LodMesher mesher(g);
  const auto m = mesher.extract(0.0, 0.0, params());
  const TriangleLocator loc(m);
  const DynamicLayer::Surface terrain = [](double, double) {return 0.0;};
  DynamicParams p;  // confirm_time 0.3 s, the extent rule on
  std::vector<std::array<float, 3>> wall;
  for (float z = 0.0f; z <= 1.0f; z += 0.05f) {
    wall.push_back({3.0f, 0.0f, z});
  }

  // a real object: seen again and again in the same place
  DynamicLayer object;
  object.ingest(wall, 0.0, 0.0, 0.0, 0.0, terrain, p, &m, &loc);
  EXPECT_FALSE(object.expire(0.05, p));
  EXPECT_TRUE(object.tiles(m, loc).empty()) << "confirmed after a single frame";
  object.ingest(wall, 0.0, 0.0, 0.0, 0.2, terrain, p, &m, &loc);
  object.ingest(wall, 0.0, 0.0, 0.0, 0.4, terrain, p, &m, &loc);
  EXPECT_TRUE(object.expire(0.4, p));          // confirmed now
  EXPECT_FALSE(object.tiles(m, loc).empty());

  // a glitch: a different cell every frame
  DynamicLayer glitch;
  for (int k = 0; k < 6; ++k) {
    std::vector<std::array<float, 3>> fleeting;
    for (float z = 0.0f; z <= 1.0f; z += 0.05f) {
      fleeting.push_back({2.0f + 0.5f * k, 0.0f, z});
    }
    glitch.ingest(fleeting, 0.0, 0.0, 0.0, 0.1 * k, terrain, p, &m, &loc);
    glitch.expire(0.1 * k, p);
    EXPECT_TRUE(glitch.tiles(m,
      loc).empty()) << "a fleeting detection was confirmed (frame " << k << ")";
  }
  glitch.expire(2.0, p);
  EXPECT_EQ(glitch.size(), 0u);  // and the unconfirmed ones are dropped quickly
}

namespace
{
// 129x129 nodes, 1 m apart, flat, with two walls (8 m high) 2 m apart running along x at y = 0..2
// (a corridor, narrow) and a hall 40 m wide next to it (no walls near)
bigmap_maps_manager::DemGrid corridor_dem()
{
  bigmap_maps_manager::DemGrid g;
  g.side = 129;
  g.resolution = 1.0;
  g.radius = 64.0;
  g.z.assign(129 * 129, 0.0f);
  for (int j = 0; j < 129; ++j) {
    for (int i = 0; i < 129; ++i) {
      const double y = g.y(j), x = g.x(i);
      if (x > -60.0 && x < 0.0 && (std::abs(y - 4.0) < 0.1 || std::abs(y - 7.0) < 0.1)) {
        g.z[j * 129 + i] = 8.0f;   // two walls, 3 m apart; between them 2 free nodes
      }
    }
  }
  return g;
}

std::size_t triangles_in(const LodMesh & m, double x0, double x1, double y0, double y1)
{
  std::size_t n = 0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    double cx = 0, cy = 0;
    for (int k = 0; k < 3; ++k) {
      cx += m.x[m.tri[3 * t + k]] / 3; cy += m.y[m.tri[3 * t + k]] / 3;
    }
    if (cx > x0 && cx < x1 && cy > y0 && cy < y1) {++n;}
  }
  return n;
}
}  // namespace

TEST(LodMesher, NarrowPassagesKeepTheGridResolutionFarFromTheRobot)
{
  const auto dem = corridor_dem();
  LodParams p = params();
  p.min_size = 1.0;
  p.high_res_radius = 5.0;
  p.low_res_distance = 40.0;
  p.error_growth = 2.0;   // far geometry is simplified at once
  const auto pinned = bigmap_maps_manager::narrow_passage_nodes(dem, {}, 35.0, 5.0);
  ASSERT_FALSE(pinned.empty());
  std::size_t n_pinned = 0;
  for (auto v : pinned) {
    n_pinned += v;
  }
  EXPECT_GT(n_pinned, 50u);
  // robot at the east end, the corridor (x -60..0) is 20..60 m away
  const LodMesher plain(dem), kept(dem, {}, pinned);
  const auto a = plain.extract(60.0, -40.0, p);
  const auto b = kept.extract(60.0, -40.0, p);
  EXPECT_GT(triangles_in(b, -60, -5, 3, 8), 3 * triangles_in(a, -60, -5, 3, 8) + 50);
  // the open ground far from the passage is not refined
  EXPECT_LE(triangles_in(b, 20, 60, -64, -30), triangles_in(a, 20, 60, -64, -30) + 8);
  // still crack-free and manifold
  for (const auto & kv : edge_use(b)) {
    EXPECT_LE(kv.second, 2);
  }
  EXPECT_EQ(bigmap_maps_manager::narrow_passage_nodes(dem, {}, 35.0, 0.0).size(), 0u);
}

TEST(LodMesher, TheFullResolutionStripAroundDrivenOverGroundIsKnownMarginWide)
{
  // with the falloff the node uses ((low_res - full_res) / traversed_fade, ~40 here) the strip is
  // still known_margin wide (it used to be known_margin / known_falloff: a few centimetres)
  const auto dem = make_dem();
  const LodMesher mesher(dem);
  bigmap_maps_manager::KnownMap known(dem.side, dem.resolution);
  for (double x = -60.0; x <= -20.0; x += 1.0) {
    known.mark(x, -40.0, 1.0);
  }
  known.update();
  LodParams p = params();
  p.known_falloff = 40.0;
  auto leaf_near = [&](const LodMesh & m, double x, double y) {
      double best = 1e9, leg = -1.0;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        double cx = 0, cy = 0;
        for (int k = 0; k < 3; ++k) {
          cx += m.x[m.tri[3 * t + k]] / 3;
          cy += m.y[m.tri[3 * t + k]] / 3;
        }
        const double d = std::hypot(cx - x, cy - y);
        if (d < best) {best = d; leg = m.leg[t];}
      }
      return leg;
    };
  p.known_margin = 12.0;
  const auto wide = mesher.extract(50.0, 50.0, p, &known);
  p.known_margin = 1.0;
  const auto narrow = mesher.extract(50.0, 50.0, p, &known);
  // 10 m from the trail (2 m DEM: the crack-free slack is kReach x hypotenuse, ~3.5 m): inside the
  // 12 m strip, well outside the 1 m one
  EXPECT_DOUBLE_EQ(leaf_near(wide, -40.0, -50.0), p.min_size);
  EXPECT_GT(leaf_near(narrow, -40.0, -50.0), p.min_size);
  EXPECT_GT(wide.triangle_count(), narrow.triangle_count());
  for (const auto & kv : edge_use(wide)) {
    EXPECT_LE(kv.second, 2);
  }
}

namespace
{
// T-junctions: a vertex of the mesh lying strictly inside an edge used by one triangle only (a
// crack). Vertices are DEM nodes, so the lattice points along each such edge are checked.
std::size_t t_junctions(const LodMesh & m, double res)
{
  std::set<std::pair<long, long>> nodes;   // NOLINT(runtime/int)
  auto node = [&](std::uint32_t v) {
      return std::make_pair(std::lround(m.x[v] / res), std::lround(m.y[v] / res));
    };
  for (std::uint32_t v = 0; v < m.x.size(); ++v) {
    nodes.insert(node(v));
  }
  std::size_t bad = 0;
  for (const auto & [e, n] : edge_use(m)) {
    if (n != 1) {continue;}
    const auto a = node(e.first), b = node(e.second);
    const long dx = b.first - a.first, dy = b.second - a.second;   // NOLINT(runtime/int)
    const long g = std::gcd(std::labs(dx), std::labs(dy));   // NOLINT(runtime/int)
    for (long k = 1; k < g; ++k) {   // NOLINT(runtime/int)
      if (nodes.count({a.first + dx / g * k, a.second + dy / g * k})) {++bad;}
    }
  }
  return bad;
}

bigmap_maps_manager::DemGrid rough_dem(int side)
{
  bigmap_maps_manager::DemGrid g;
  g.side = side;
  g.resolution = 2.0;
  g.radius = (side - 1) / 2 * 2.0;
  g.z.resize(static_cast<std::size_t>(side) * side);
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      const double x = g.x(i), y = g.y(j);
      g.z[static_cast<std::size_t>(j) * side + i] = static_cast<float>(
        8.0 * std::sin(x / 17.0) * std::cos(y / 13.0) + 0.6 * std::sin(x / 3.1 + y / 2.3));
    }
  }
  return g;
}
}  // namespace

TEST(LodMesher, NoCracksWithDrivenOverGroundOnGridsThatAreNotTwoToTheKPlusOne)
{
  // the RTIN pads these DEMs to 65 nodes; the driven-over field must cover that padding with a
  // true distance, or the policy stops being monotone there and the mesh cracks. A sweep of
  // trails, policies and robot positions (deterministic)
  std::size_t cases = 0, bad = 0;
  for (const int side : {41, 45, 51, 61, 65}) {
    const auto dem = rough_dem(side);
    const LodMesher mesher(dem);
    const double edge = (side - 1) / 2 * 2.0;
    for (int trail = 0; trail < 3; ++trail) {
      bigmap_maps_manager::KnownMap known(dem.side, dem.resolution, mesher.error_map().grid_size());
      for (double t = -edge; t <= edge; t += 1.0) {
        if (trail == 0) {known.mark(t, 0.3 * t, 1.0);}       // across the map
        if (trail == 1) {known.mark(edge - 1.0, t, 1.0);}    // along the high-side edge
        if (trail == 2) {known.mark(t, edge - 1.0, 1.0);}
      }
      known.update();
      for (const double falloff : {1.0, 3.0, 8.0, 40.0}) {
        for (const double margin : {0.5, 3.0, 9.0}) {
          LodParams p = params();
          p.high_res_radius = 6.0;
          p.low_res_distance = 40.0;
          p.known_falloff = falloff;
          p.known_margin = margin;
          for (double rx = -90.0; rx <= 90.0; rx += 45.0) {
            const auto m = mesher.extract(rx, 0.7 * rx - 20.0, p, &known);
            const auto n = t_junctions(m, dem.resolution);
            ++cases;
            bad += n > 0;
            EXPECT_EQ(n, 0u) << "side " << side << " trail " << trail << " falloff " << falloff
                             << " margin " << margin << " robot x " << rx;
          }
        }
      }
    }
  }
  EXPECT_EQ(bad, 0u) << "of " << cases << " meshes";
}
