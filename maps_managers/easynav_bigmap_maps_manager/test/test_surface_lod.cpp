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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "easynav_bigmap_maps_manager/surface_lod.hpp"
#include "navmap_core/NavMap.hpp"

using bigmap_maps_manager::LodMesh;
using bigmap_maps_manager::LodParams;
using bigmap_maps_manager::SurfaceLodMesher;

namespace
{
// A rectangular patch of triangles of leg h over [x0, x1] x [y0, y1] with height z(x, y). Its
// vertices are its own: neighbouring patches meet only through the welding of equal positions.
void add_patch(
  ::navmap::NavMap & nm, double x0, double x1, double y0, double y1, double h,
  const std::function<double(double, double)> & z, bool mark_traversed = false)
{
  const int nx = static_cast<int>(std::lround((x1 - x0) / h));
  const int ny = static_cast<int>(std::lround((y1 - y0) / h));
  const auto base = static_cast<std::uint32_t>(nm.positions.x.size());
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      const double x = x0 + i * h, y = y0 + j * h;
      nm.positions.x.push_back(static_cast<float>(x));
      nm.positions.y.push_back(static_cast<float>(y));
      nm.positions.z.push_back(static_cast<float>(z(x, y)));
    }
  }
  auto vid = [&](int i, int j) {return base + static_cast<std::uint32_t>(j * (nx + 1) + i);};
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      for (int half = 0; half < 2; ++half) {
        ::navmap::NavCel c;
        if (half == 0) {
          c.v[0] = vid(i, j); c.v[1] = vid(i + 1, j); c.v[2] = vid(i + 1, j + 1);
        } else {
          c.v[0] = vid(i, j); c.v[1] = vid(i + 1, j + 1); c.v[2] = vid(i, j + 1);
        }
        nm.navcels.push_back(c);
        (void)mark_traversed;
      }
    }
  }
}

// Two floors one over the other (z = 0 and z = 3) joined by a ramp that starts over the lower one:
//   floor 0: x in [-30, 20]      ramp: x in [-20, -10], z from 0 to 3      floor 1: x in [-10, 20]
::navmap::NavMap garage()
{
  ::navmap::NavMap nm;
  add_patch(nm, -30, 20, -10, 10, 0.5, [](double, double) {return 0.0;});
  add_patch(nm, -20, -10, -10, 10, 0.5, [](double x, double) {return 0.3 * (x + 20.0);});
  add_patch(nm, -10, 20, -10, 10, 0.5, [](double, double) {return 3.0;});
  return nm;
}

LodParams params()
{
  LodParams p;
  p.high_res_radius = 10.0;
  p.low_res_distance = 30.0;
  p.min_size = 1.0;
  p.max_size = 16.0;
  p.base_error = 0.02;
  p.error_growth = 0.05;
  p.known_margin = 1.0;
  p.known_falloff = 4.0;
  return p;
}

SurfaceLodMesher::Config config()
{
  SurfaceLodMesher::Config c;
  c.min_size = 1.0;
  c.max_size = 16.0;
  c.cluster_height = 1.2;
  c.vertical_tolerance = 1.5;
  return c;
}

struct Centroid
{
  double x, y, z;
};

Centroid centroid(const LodMesh & m, std::size_t t)
{
  const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
  return {(m.x[a] + m.x[b] + m.x[c]) / 3.0, (m.y[a] + m.y[b] + m.y[c]) / 3.0,
    (m.z[a] + m.z[b] + m.z[c]) / 3.0};
}

// Can a walker go from a triangle for which `from` holds to one for which `to` holds crossing
// shared edges only?
bool connected(
  const LodMesh & m, const std::function<bool(const Centroid &)> & from,
  const std::function<bool(const Centroid &)> & to)
{
  const std::size_t nt = m.triangle_count();
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>> edges;
  for (std::size_t t = 0; t < nt; ++t) {
    for (int k = 0; k < 3; ++k) {
      auto a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
      if (a > b) {std::swap(a, b);}
      edges[{a, b}].push_back(t);
    }
  }
  std::vector<char> seen(nt, 0);
  std::queue<std::size_t> q;
  for (std::size_t t = 0; t < nt; ++t) {
    if (from(centroid(m, t))) {seen[t] = 1; q.push(t);}
  }
  if (q.empty()) {return false;}
  while (!q.empty()) {
    const auto t = q.front();
    q.pop();
    if (to(centroid(m, t))) {return true;}
    for (int k = 0; k < 3; ++k) {
      auto a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
      if (a > b) {std::swap(a, b);}
      for (const auto n : edges[{a, b}]) {
        if (!seen[n]) {seen[n] = 1; q.push(n);}
      }
    }
  }
  return false;
}
}  // namespace

TEST(SurfaceLod, BuildsAHierarchy)
{
  SurfaceLodMesher mesher(garage(), config());
  EXPECT_GT(mesher.level_count(), 3);
  EXPECT_GT(mesher.vertex_count(), 1000u);
  // the three patches share their edges: welded vertices are fewer than the sum of the patches
  EXPECT_LT(mesher.vertex_count(), 101u * 41u + 21u * 41u + 61u * 41u);
}

TEST(SurfaceLod, FarAwayIsCoarserAndNearIsFine)
{
  SurfaceLodMesher mesher(garage(), config());
  const auto near_mesh = mesher.extract(15.0, 0.0, 0.0, params());
  EXPECT_LT(near_mesh.triangle_count(), mesher.source_triangle_count() / 2);
  for (std::size_t t = 0; t < near_mesh.triangle_count(); ++t) {
    const auto c = centroid(near_mesh, t);
    if (std::hypot(c.x - 15.0, c.y) < 4.0 && std::abs(c.z) < 0.5) {
      ASSERT_LE(near_mesh.leg[t], 1.01f) << "at " << c.x << ", " << c.y;
    }
  }
  float far_leg = 0;
  for (std::size_t t = 0; t < near_mesh.triangle_count(); ++t) {
    const auto c = centroid(near_mesh, t);
    if (c.x < -20.0) {far_leg = std::max(far_leg, near_mesh.leg[t]);}
  }
  EXPECT_GE(far_leg, 8.0f);
}

TEST(SurfaceLod, FloorsAreNeverMerged)
{
  SurfaceLodMesher mesher(garage(), config());
  for (const auto & pos : {std::array<double, 3>{15, 0, 0}, std::array<double, 3>{-25, 0, 0},
      std::array<double, 3>{15, 0, 3}})
  {
    const auto m = mesher.extract(pos[0], pos[1], pos[2], params());
    for (std::size_t v = 0; v < m.x.size(); ++v) {
      if (m.x[v] > -9.0) {
        ASSERT_TRUE(std::abs(m.z[v]) < 0.01 || std::abs(m.z[v] - 3.0) < 0.01)
          << "x " << m.x[v] << " z " << m.z[v];
      }
    }
  }
}

TEST(SurfaceLod, FloorsStayConnectedThroughTheRamp)
{
  SurfaceLodMesher mesher(garage(), config());
  const auto on_floor0 = [](const Centroid & c) {return c.x > 10.0 && std::abs(c.z) < 0.1;};
  const auto on_floor1 = [](const Centroid & c) {return c.x > 10.0 && std::abs(c.z - 3.0) < 0.1;};
  // wherever the robot is, the planner can still go up the ramp and the other way round
  for (const auto & pos : {std::array<double, 3>{15, 0, 0}, std::array<double, 3>{15, 0, 3},
      std::array<double, 3>{-25, 0, 0}, std::array<double, 3>{-15, 0, 1.5},
      std::array<double, 3>{0, 5, 3}, std::array<double, 3>{-100, 0, 0}})
  {
    const auto m = mesher.extract(pos[0], pos[1], pos[2], params());
    ASSERT_TRUE(connected(m, on_floor0, on_floor1))
      << "robot at " << pos[0] << ", " << pos[1] << ", " << pos[2];
    ASSERT_TRUE(connected(m, on_floor1, on_floor0));
  }
}

TEST(SurfaceLod, NoFloorMeshWithoutARamp)
{
  // Without the ramp the floors are two components: the test above would not be meaningful if
  // everything were connected anyway
  ::navmap::NavMap nm;
  add_patch(nm, -30, 20, -10, 10, 0.5, [](double, double) {return 0.0;});
  add_patch(nm, -10, 20, -10, 10, 0.5, [](double, double) {return 3.0;});
  SurfaceLodMesher mesher(nm, config());
  const auto m = mesher.extract(15.0, 0.0, 0.0, params());
  EXPECT_FALSE(connected(
      m, [](const Centroid & c) {return std::abs(c.z) < 0.1;},
      [](const Centroid & c) {return std::abs(c.z - 3.0) < 0.1;}));
}

TEST(SurfaceLod, DrivenOverGroundStaysFineOnItsOwnFloorOnly)
{
  SurfaceLodMesher mesher(garage(), config());
  EXPECT_GT(mesher.mark(15.0, 0.0, 0.0, 1.5), 0);
  EXPECT_EQ(mesher.mark(15.0, 0.0, 0.0, 1.5), 0);  // nothing new the second time
  const auto m = mesher.extract(-25.0, 0.0, 0.0, params());  // the robot is 40 m away
  bool fine_below = false, upper_is_coarse = true, flagged = false, upper_flagged = false;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto c = centroid(m, t);
    if (std::hypot(c.x - 15.0, c.y) < 1.0) {
      if (std::abs(c.z) < 0.1) {
        fine_below = fine_below || m.leg[t] <= 1.01f;
        flagged = flagged || m.traversed[t];
      } else if (std::abs(c.z - 3.0) < 0.1) {
        upper_is_coarse = upper_is_coarse && m.leg[t] >= 4.0f;
        upper_flagged = upper_flagged || m.traversed[t];
      }
    }
  }
  EXPECT_TRUE(fine_below);
  EXPECT_TRUE(flagged);
  EXPECT_TRUE(upper_is_coarse);  // the floor above was not driven over
  EXPECT_FALSE(upper_flagged);

  // forgotten beyond a radius: the robot at the other end, 40 m away
  EXPECT_GT(mesher.forget_outside(-25.0, 0.0, 0.0, 20.0), 0u);
  EXPECT_EQ(mesher.traversed_count(), 0u);
  const auto m2 = mesher.extract(-25.0, 0.0, 0.0, params());
  for (std::size_t t = 0; t < m2.triangle_count(); ++t) {
    const auto c = centroid(m2, t);
    if (std::hypot(c.x - 15.0, c.y) < 1.0) {ASSERT_GE(m2.leg[t], 4.0f);}
    ASSERT_FALSE(m2.traversed[t]);
  }
}

TEST(SurfaceLod, RestoresTheTraversedLayerOfTheMap)
{
  auto nm = garage();
  nm.add_layer<std::uint8_t>("traversed", "", "", 0);
  for (std::size_t t = 0; t < nm.navcels.size(); ++t) {
    const auto & v = nm.navcels[t].v;
    if (nm.positions.x[v[0]] > 12.0 && nm.positions.z[v[0]] < 0.1) {
      nm.layer_set<std::uint8_t>("traversed", static_cast<::navmap::NavCelId>(t), 255);
    }
  }
  SurfaceLodMesher mesher(nm, config());
  EXPECT_TRUE(mesher.had_traversed_layer());
  EXPECT_GT(mesher.traversed_count(), 0u);
  const auto m = mesher.extract(-25.0, 0.0, 0.0, params());
  std::size_t flagged = 0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    flagged += m.traversed[t];
  }
  EXPECT_GT(flagged, 10u);
}

TEST(SurfaceLod, RampKeepsItsShape)
{
  SurfaceLodMesher mesher(garage(), config());
  // robot on the ramp: every vertex of the ramp zone is on (or within the cluster error of) it
  const auto m = mesher.extract(-15.0, 0.0, 1.5, params());
  for (std::size_t v = 0; v < m.x.size(); ++v) {
    if (m.x[v] > -19.0 && m.x[v] < -11.0) {
      // the ramp, or the floor under it
      const double ramp = 0.3 * (m.x[v] + 20.0);
      ASSERT_TRUE(std::abs(m.z[v] - ramp) < 0.65 || std::abs(m.z[v]) < 0.01)
        << "x " << m.x[v] << " z " << m.z[v];
    }
  }
}

TEST(SurfaceLod, ABorderKeepsItsPlaceAndANarrowRampItsWidth)
{
  // a ramp 2.4 m wide: clusters of 1 m must not shrink it
  ::navmap::NavMap nm;
  add_patch(nm, -20, 20, -1.2, 1.2, 0.4, [](double x, double) {return 0.1 * (x + 20.0);});
  SurfaceLodMesher mesher(nm, config());
  for (const double rx : {0.0, -19.0, 19.0}) {
    const auto m = mesher.extract(rx, 0.0, 0.1 * (rx + 20.0), params());
    float ymin = 1e9f, ymax = -1e9f;
    for (std::size_t v = 0; v < m.y.size(); ++v) {
      if (std::abs(m.x[v] - rx) < 8.0) {
        ymin = std::min(ymin, m.y[v]);
        ymax = std::max(ymax, m.y[v]);
      }
    }
    EXPECT_NEAR(ymin, -1.2f, 0.05f) << "robot at " << rx;
    EXPECT_NEAR(ymax, 1.2f, 0.05f) << "robot at " << rx;
  }
}

TEST(SurfaceLod, SlopeComesFromTheSourceNotFromTheSimplification)
{
  SurfaceLodMesher mesher(garage(), config());
  const double ramp_deg = std::atan(0.3) * 180.0 / M_PI;  // the steepest thing in the map
  for (const auto & pos : {std::array<double, 3>{15, 0, 0}, std::array<double, 3>{-100, 0, 0},
      std::array<double, 3>{-15, 0, 1.5}})
  {
    const auto m = mesher.extract(pos[0], pos[1], pos[2], params());
    ASSERT_EQ(m.slope.size(), m.triangle_count());
    float steepest = 0;
    for (const auto s : m.slope) {
      steepest = std::max(steepest, s);
    }
    EXPECT_LE(steepest, ramp_deg + 0.5) << "robot at " << pos[0];
    EXPECT_GT(steepest, ramp_deg - 0.5);  // and the ramp is there
  }
}

TEST(SurfaceLod, NarrowPassagesKeepTheirResolutionWhateverTheDistance)
{
  // a ramp 2.4 m wide (narrow) and a hall 40 m wide, side by side
  ::navmap::NavMap nm;
  add_patch(nm, -20, 20, -1.2, 1.2, 0.4, [](double x, double) {return 0.1 * (x + 20.0);});
  add_patch(nm, -20, 20, 10, 50, 0.4, [](double, double) {return 0.0;});
  auto cfg = config();
  cfg.keep_narrow_width = 3.0;
  SurfaceLodMesher kept(nm, cfg);
  EXPECT_GT(kept.narrow_vertex_count(), 100u);
  cfg.keep_narrow_width = 0.0;
  SurfaceLodMesher plain(nm, cfg);
  EXPECT_EQ(plain.narrow_vertex_count(), 0u);
  // robot at the far end of the hall: the ramp is 60 m away
  const auto a = kept.extract(0.0, 45.0, 0.0, params());
  const auto b = plain.extract(0.0, 45.0, 0.0, params());
  std::size_t fine_ramp = 0, coarse_ramp = 0, coarse_hall = 0;
  for (std::size_t t = 0; t < a.triangle_count(); ++t) {
    const auto c = centroid(a, t);
    if (std::abs(c.y) < 1.5 && a.leg[t] <= 1.0f) {++fine_ramp;}
  }
  for (std::size_t t = 0; t < b.triangle_count(); ++t) {
    const auto c = centroid(b, t);
    if (std::abs(c.y) < 1.5 && b.leg[t] <= 1.0f) {++coarse_ramp;}
    if (c.y > 10 && b.leg[t] >= 4.0f) {++coarse_hall;}
  }
  EXPECT_GT(fine_ramp, 5 * coarse_ramp + 100);  // the ramp is at full resolution
  EXPECT_GT(coarse_hall, 0u);                   // the hall is not
  // and the hall is as coarse as ever with the option on
  std::size_t hall_fine_kept = 0, hall_fine_plain = 0;
  for (std::size_t t = 0; t < a.triangle_count(); ++t) {
    hall_fine_kept += centroid(a, t).y < 9 ? 0 : (a.leg[t] <= 1.0f && centroid(a, t).y < 25);
  }
  for (std::size_t t = 0; t < b.triangle_count(); ++t) {
    hall_fine_plain += centroid(b, t).y < 9 ? 0 : (b.leg[t] <= 1.0f && centroid(b, t).y < 25);
  }
  EXPECT_LE(hall_fine_kept, hall_fine_plain + 5);  // (a corner or two may count as narrow)
}

TEST(SurfaceLod, AGapOrALedgeIsNeverBridgedHoweverCoarse)
{
  // two pieces of floor that do not touch: 0.6 m apart (inside one 1 m cell), 5 m apart, and a
  // 1 m ledge with a gap. The mesh must not join them at any distance from the robot.
  struct Case {double gap0, gap1, z_right;};
  for (const Case c : {Case{0.2, 0.8, 0.0}, Case{1.0, 6.0, 0.0}, Case{0.2, 0.8, 1.0}}) {
    ::navmap::NavMap nm;
    add_patch(nm, -20.0, c.gap0, -5.0, 5.0, 0.2, [](double, double) {return 0.0;});
    add_patch(nm, c.gap1, 20.0, -5.0, 5.0, 0.2, [&](double, double) {return c.z_right;});
    auto cfg = config();
    cfg.drop_unreachable_ground = false;
    const SurfaceLodMesher mesher(nm, cfg);
    for (const double rx : {0.5, 40.0, 200.0}) {
      const auto m = mesher.extract(rx, 0.0, 0.0, params());
      ASSERT_GT(m.triangle_count(), 0u);
      EXPECT_FALSE(connected(
          m, [&](const Centroid & q) {return q.x < c.gap0 - 0.5;},
          [&](const Centroid & q) {return q.x > c.gap1 + 0.5;}))
        << "gap " << c.gap0 << ".." << c.gap1 << ", ledge " << c.z_right << ", robot at " << rx;
    }
  }
}

TEST(SurfaceLod, AMapWithANonFiniteVertexOrABadIndexIsRefusedAndNaNPosesMarkNothing)
{
  // a NaN vertex was welded, cast to a cell index (undefined: INT64_MIN, then an overflow in the
  // cell loops) and came out in every mesh; a NaN robot position did the same in mark()
  SurfaceLodMesher::Config cfg;
  cfg.min_size = 0.5;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const float bad : {std::nanf(""), std::numeric_limits<float>::infinity()}) {
    auto nm = garage();
    nm.positions.z[7] = bad;
    EXPECT_THROW(SurfaceLodMesher(nm, cfg), std::invalid_argument);
    nm = garage();
    nm.positions.x[0] = bad;
    EXPECT_THROW(SurfaceLodMesher(nm, cfg), std::invalid_argument);
  }
  {
    auto nm = garage();
    nm.navcels[3].v[1] = static_cast<std::uint32_t>(nm.positions.x.size()) + 5;
    EXPECT_THROW(SurfaceLodMesher(nm, cfg), std::invalid_argument);
  }
  SurfaceLodMesher s(garage(), cfg);
  EXPECT_EQ(s.mark(nan, 0.0, 0.0, 1.0), 0);
  EXPECT_EQ(s.mark(0.0, nan, 0.0, 1.0), 0);
  EXPECT_EQ(s.mark(0.0, 0.0, 0.0, nan), 0);
  EXPECT_EQ(s.traversed_count(), 0u);
  EXPECT_GT(s.mark(0.0, 0.0, 0.0, 1.0), 0);
  EXPECT_EQ(s.forget_outside(nan, 0.0, 0.0, 1.0), 0u);   // not forgotten around nowhere
  EXPECT_GT(s.traversed_count(), 0u);
  const auto m = s.extract(nan, nan, nan, params());
  EXPECT_GT(m.triangle_count(), 0u);   // a mesh all the same (the policy sees no distance)
  for (const auto z : m.z) {
    EXPECT_TRUE(std::isfinite(z));
  }
}

namespace
{
// A mesh as a sorted list of triangles, each as its three corners (rotated so that the smallest
// comes first, the orientation kept) and what it carries: two meshes with the same triangles in
// another order compare equal
using Corner = std::array<float, 3>;
struct Tri
{
  std::array<Corner, 3> c;
  float leg, slope;
  int traversed;
  bool operator<(const Tri & o) const
  {
    return std::tie(c, leg, slope, traversed) < std::tie(o.c, o.leg, o.slope, o.traversed);
  }
  bool operator==(const Tri & o) const
  {
    return c == o.c && leg == o.leg && slope == o.slope && traversed == o.traversed;
  }
};

std::vector<Tri> canonical(const LodMesh & m)
{
  std::vector<Tri> out;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    std::array<Corner, 3> c;
    for (int k = 0; k < 3; ++k) {
      const auto v = m.tri[3 * t + k];
      c[k] = {m.x[v], m.y[v], m.z[v]};
    }
    std::rotate(c.begin(), std::min_element(c.begin(), c.end()), c.end());
    out.push_back({c, m.leg[t], m.slope[t], m.traversed[t]});
  }
  std::sort(out.begin(), out.end());
  return out;
}

// The garage with bumps on the lower floor and holes (obstacles) here and there
::navmap::NavMap holed_garage(unsigned seed)
{
  ::navmap::NavMap nm;
  add_patch(nm, -30, 20, -10, 10, 0.5, [](double x, double y) {
      return 0.15 * std::sin(0.7 * x) * std::cos(0.9 * y);
    });
  add_patch(nm, -20, -10, -10, 10, 0.5, [](double x, double) {return 0.3 * (x + 20.0);});
  add_patch(nm, -10, 20, -10, 10, 0.5, [](double, double) {return 3.0;});
  std::vector<::navmap::NavCel> kept;
  unsigned s = seed;
  for (const auto & c : nm.navcels) {
    s = s * 1103515245u + 12345u;
    if ((s >> 16) % 37 != 0) {kept.push_back(c);}
  }
  nm.navcels = kept;
  return nm;
}
}  // namespace

TEST(SurfaceLod, TheIncrementalExtractionIsTheFullOneWhereverTheRobotAndTheGroundAre)
{
  // extract() takes from a precomputed far cut whatever the robot (and the driven-over /
  // observed ground) cannot change and climbs again only the rest: the result must be the
  // reference's triangle by triangle, for any position, policy and ground
  for (unsigned seed : {1u, 7u}) {
    for (const double hole_error : {0.0, 0.3}) {
      for (const double narrow : {0.0, 1.6}) {
        auto cfg = config();
        cfg.hole_error = hole_error;
        cfg.keep_narrow_width = narrow;
        SurfaceLodMesher mesher(holed_garage(seed), cfg);
        auto p = params();
        unsigned s = seed;
        const auto rnd = [&s](double lo, double hi) {
            s = s * 1103515245u + 12345u;
            return lo + (hi - lo) * ((s >> 8) % 100000) / 100000.0;
          };
        for (int i = 0; i < 12; ++i) {
          const double x = rnd(-35.0, 25.0), y = rnd(-12.0, 12.0);
          const double z = rnd(0.0, 1.0) < 0.5 ? 0.0 : 3.0;
          if (i == 4) {mesher.mark(x, y, z, 2.0);}
          if (i == 6) {mesher.mark_observed(x, y, z, 6.0);}
          if (i == 7) {p.observed_margin = 0.5;}
          if (i == 8) {p.high_res_radius = 4.0; p.known_falloff = 1.0;}   // the far cut stays
          if (i == 9) {p.max_size = 8.0; p.error_growth = 0.0;}           // it is rebuilt
          if (i == 10) {mesher.forget_outside(x, y, z, 15.0);}
          if (i == 11) {mesher.forget_observed_outside(x, y, z, 0.0);}
          const auto fast = mesher.extract(x, y, z, p);
          const auto full = mesher.extract_reference(x, y, z, p);
          ASSERT_GT(full.triangle_count(), 0u);
          ASSERT_EQ(fast.triangle_count(), full.triangle_count()) << "seed " << seed << " i " << i;
          ASSERT_TRUE(canonical(fast) == canonical(full)) << "seed " << seed << " i " << i;
          ASSERT_EQ(fast.x.size(), full.x.size());
          ASSERT_EQ(fast.r.size(), full.r.size());
        }
      }
    }
  }
}

TEST(SurfaceLod, AHoleSeenFromNearKeepsItsShapeFromFarWhenObserved)
{
  // A square hole of 1 m in the middle of a flat floor. The hierarchy never bridges it, but from
  // far the clusters around it are coarse and its shape with them; once its border was observed
  // it is kept at full resolution wherever the robot goes
  ::navmap::NavMap nm;
  add_patch(nm, -40, 40, -10, 10, 0.5, [](double, double) {return 0.0;});
  std::vector<::navmap::NavCel> kept;
  for (const auto & c : nm.navcels) {
    double cx = 0, cy = 0;
    for (int k = 0; k < 3; ++k) {
      cx += nm.positions.x[c.v[k]] / 3.0;
      cy += nm.positions.y[c.v[k]] / 3.0;
    }
    if (std::abs(cx - 20.25) < 0.5 && std::abs(cy - 0.25) < 0.5) {continue;}
    kept.push_back(c);
  }
  nm.navcels = kept;
  // the largest triangle with a corner within 1 m of the hole's centre
  const auto coarsest_at_hole = [](const LodMesh & m) {
      float worst = 0.0f;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        for (int k = 0; k < 3; ++k) {
          const auto v = m.tri[3 * t + k];
          if (std::hypot(m.x[v] - 20.25, m.y[v] - 0.25) < 1.0) {worst = std::max(worst, m.leg[t]);}
        }
      }
      return worst;
    };
  auto p = params();
  SurfaceLodMesher mesher(nm, config());
  EXPECT_LE(coarsest_at_hole(mesher.extract(20.0, 0.0, 0.0, p)), 1.01f);
  EXPECT_GE(coarsest_at_hole(mesher.extract(-35.0, 0.0, 0.0, p)), 4.0f);   // 55 m away
  EXPECT_GT(mesher.mark_observed(20.0, 0.0, 0.0, 3.0), 0);
  EXPECT_EQ(mesher.mark_observed(20.0, 0.0, 0.0, 3.0), 0);
  EXPECT_GT(mesher.observed_count(), 0u);
  EXPECT_EQ(mesher.traversed_count(), 0u);   // observed is not driven over
  EXPECT_LE(coarsest_at_hole(mesher.extract(-35.0, 0.0, 0.0, p)), 1.01f);
  EXPECT_GT(mesher.forget_observed_outside(-35.0, 0.0, 0.0, 10.0), 0u);
  EXPECT_GE(coarsest_at_hole(mesher.extract(-35.0, 0.0, 0.0, p)), 4.0f);

  // a hole_error above the tolerance at that distance keeps its border where the map has it with
  // nothing observed: the vertices around the hole are the map's (on its 0.5 m lattice), not the
  // means of border clusters
  const auto border_is_the_maps = [](const LodMesh & m) {
      for (std::size_t v = 0; v < m.x.size(); ++v) {
        if (std::hypot(m.x[v] - 20.25, m.y[v] - 0.25) > 1.0) {continue;}
        if (std::abs(m.x[v] * 2.0f - std::round(m.x[v] * 2.0f)) > 1e-3f ||
          std::abs(m.y[v] * 2.0f - std::round(m.y[v] * 2.0f)) > 1e-3f)
        {
          return false;
        }
      }
      return true;
    };
  EXPECT_FALSE(border_is_the_maps(mesher.extract(-35.0, 0.0, 0.0, p)));
  auto cfg = config();
  cfg.hole_error = 5.0;
  SurfaceLodMesher strict(nm, cfg);
  EXPECT_TRUE(border_is_the_maps(strict.extract(-35.0, 0.0, 0.0, p)));
}

TEST(SurfaceLod, BuildsFromTheArraysOfAMapAsFromTheMapAndReportsItsMemory)
{
  const auto nm = garage();
  SurfaceLodMesher a(nm, config());
  SurfaceLodMesher b(bigmap_maps_manager::SourceMesh::from_navmap(nm), config());
  EXPECT_EQ(a.vertex_count(), b.vertex_count());
  EXPECT_TRUE(canonical(a.extract(0, 0, 0, params())) == canonical(b.extract(0, 0, 0, params())));
  EXPECT_GT(a.memory_bytes(), a.vertex_count() * 12);
  // far less than the NavMap it comes from (positions, NavCels, normals, adjacency, BVH...)
  EXPECT_LT(a.memory_bytes(), 200 * a.source_triangle_count());
}

// Full resolution within the fine radius (paper, Section 3.4): guaranteed on a flat floor; on a
// ramp the clusters may be coarser. The longest edge of the triangles near the robot on its own
// surface, for the robot on the upper floor and on the ramp (the ramp's value is reported).
TEST(SurfaceLod, TheFineDiscIsAtFullResolutionOnAFloorAndIsMeasuredOnARamp)
{
  SurfaceLodMesher mesher(garage(), config());
  const auto p = params();
  auto longest_near = [&](double rx, double ry, double rz, double radius, std::size_t & n) {
      const auto m = mesher.extract(rx, ry, rz, p);
      double longest = 0.0;
      n = 0;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        const auto c = centroid(m, t);
        if (std::hypot(c.x - rx, c.y - ry) > radius || std::abs(c.z - rz) > 1.0) {continue;}
        ++n;
        for (int k = 0; k < 3; ++k) {
          const auto a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
          longest = std::max(longest, static_cast<double>(std::hypot(
              std::hypot(m.x[a] - m.x[b], m.y[a] - m.y[b]), m.z[a] - m.z[b])));
        }
      }
      return longest;
    };
  std::size_t n_floor = 0, n_ramp = 0;
  // the fine disc minus a margin of one coarse edge (triangles straddling its border may be larger)
  const double r = p.high_res_radius - 2.0 * config().min_size;
  const double floor = longest_near(10.0, 0.0, 3.0, r, n_floor);
  const double ramp = longest_near(-15.0, 0.0, 1.5, r, n_ramp);
  std::printf("[fine disc] longest edge within %.1f m: floor %.2f m (%zu triangles), ramp %.2f m "
    "(%zu triangles); min_size %.2f m\n", r, floor, n_floor, ramp, n_ramp, config().min_size);
  ASSERT_GT(n_floor, 0u);
  ASSERT_GT(n_ramp, 0u);
  EXPECT_LE(floor, config().min_size * std::sqrt(2.0) + 1e-3);
  RecordProperty("ramp_longest_edge_m", std::to_string(ramp));
}
