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
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "easynav_bigmap_maps_manager/known_map.hpp"
#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/navmap_dem.hpp"
#include "navmap_core/NavMap.hpp"

using bigmap_maps_manager::dem_from_navmap;

namespace
{
double plane(double x, double y) {return 0.3 * x + 0.1 * y + 2.0;}

// Triangles of leg `h` over [x0, x1] x [y0, y1] (an L-shape if `cut`), on the plane above,
// coloured by x; the NavCels with x < -10 are `traversed`
::navmap::NavMap make_map(double h, bool cut, bool with_traversed)
{
  ::navmap::NavMap nm;
  const int nx = static_cast<int>(std::ceil(60.0 / h)), ny = static_cast<int>(std::ceil(40.0 / h));
  ::navmap::Colors col;
  auto vid = [&](int i, int j) {return static_cast<std::uint32_t>(j * (nx + 1) + i);};
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      const double x = -30.0 + i * h, y = -20.0 + j * h;
      nm.positions.x.push_back(static_cast<float>(x));
      nm.positions.y.push_back(static_cast<float>(y));
      nm.positions.z.push_back(static_cast<float>(plane(x, y)));
      col.r.push_back(static_cast<std::uint8_t>(128 + 2 * x));
      col.g.push_back(50);
      col.b.push_back(200);
      col.a.push_back(255);
    }
  }
  nm.colors = col;
  std::vector<double> cx;
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const double x = -30.0 + (i + 0.5) * h, y = -20.0 + (j + 0.5) * h;
      if (cut && x > 0.0 && y > 0.0) {continue;}  // L shape: the upper right quarter is missing
      for (int k = 0; k < 2; ++k) {
        ::navmap::NavCel c;
        if (k == 0) {
          c.v[0] = vid(i, j); c.v[1] = vid(i + 1, j); c.v[2] = vid(i + 1, j + 1);
        } else {
          c.v[0] = vid(i, j); c.v[1] = vid(i + 1, j + 1); c.v[2] = vid(i, j + 1);
        }
        nm.navcels.push_back(c);
        cx.push_back(x);
      }
    }
  }
  nm.surfaces.resize(1);
  nm.surfaces[0].navcels.resize(nm.navcels.size());
  for (std::size_t t = 0; t < nm.navcels.size(); ++t) {
    nm.surfaces[0].navcels[t] = static_cast<::navmap::NavCelId>(t);
  }
  if (with_traversed) {
    nm.add_layer<std::uint8_t>("traversed", "", "", 0);
    for (std::size_t t = 0; t < nm.navcels.size(); ++t) {
      if (cx[t] < -10.0) {
        nm.layer_set<std::uint8_t>("traversed", static_cast<::navmap::NavCelId>(t), 255);
      }
    }
  }
  return nm;
}
}  // namespace

TEST(NavMapDem, HeightsAndColoursAreResampledOnTheGridAtAnyNavCelSize)
{
  for (double h : {0.5, 3.0, 7.5}) {
    const auto nd = dem_from_navmap(make_map(h, false, false), 1.0);
    const auto & d = nd.dem;
    EXPECT_EQ(d.side % 2, 1);
    int checked = 0;
    for (double x = -29.0; x <= 29.0; x += 2.0) {
      for (double y = -19.0; y <= 19.0; y += 2.0) {
        const int i = static_cast<int>(std::lround(x / d.resolution)) + (d.side - 1) / 2;
        const int j = static_cast<int>(std::lround(y / d.resolution)) + (d.side - 1) / 2;
        ASSERT_TRUE(nd.valid[j * d.side + i]) << "h " << h << " at " << x << "," << y;
        EXPECT_NEAR(d.at(i, j), plane(x, y), 2e-3);
        EXPECT_NEAR(d.rgb[3 * (j * d.side + i)], 128 + 2 * x, 1.5);
        ++checked;
      }
    }
    EXPECT_GT(checked, 100);
  }
}

TEST(NavMapDem, GroundOutsideTheMapIsNoDataAndTheMesherDropsIt)
{
  const auto nd = dem_from_navmap(make_map(2.0, true, false), 1.0);
  const auto & d = nd.dem;
  auto idx = [&](double x, double y) {
      return (static_cast<int>(std::lround(y)) + (d.side - 1) / 2) * d.side +
             static_cast<int>(std::lround(x)) + (d.side - 1) / 2;
    };
  EXPECT_TRUE(nd.valid[idx(-10, 10)]);
  EXPECT_FALSE(nd.valid[idx(15, 10)]);              // the missing quarter
  EXPECT_TRUE(std::isfinite(d.z[idx(15, 10)]));     // but filled, never an infinity

  const bigmap_maps_manager::LodMesher mesher(d, nd.valid);
  bigmap_maps_manager::LodParams p;
  p.high_res_radius = 10.0;
  p.min_size = 1.0;
  p.max_size = 8.0;
  const auto m = mesher.extract(-20.0, 0.0, p);
  ASSERT_GT(m.triangle_count(), 0u);
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    double cx = 0, cy = 0;
    for (int k = 0; k < 3; ++k) {
      cx += m.x[m.tri[3 * t + k]] / 3;
      cy += m.y[m.tri[3 * t + k]] / 3;
    }
    EXPECT_FALSE(cx > 3.0 &&
      cy > 3.0) << "a NavCel over the missing quarter at " << cx << "," << cy;
  }
}

TEST(NavMapDem, TraversedLayerIsRestoredAsKnownGround)
{
  const auto without = dem_from_navmap(make_map(2.0, false, false), 1.0);
  EXPECT_FALSE(without.has_traversed);
  EXPECT_TRUE(without.traversed_nodes.empty());

  const auto nd = dem_from_navmap(make_map(2.0, false, true), 1.0);
  EXPECT_TRUE(nd.has_traversed);
  ASSERT_FALSE(nd.traversed_nodes.empty());
  bigmap_maps_manager::KnownMap known(nd.dem.side, nd.dem.resolution);
  for (const auto & [i, j] : nd.traversed_nodes) {
    known.mark_node(i, j);
  }
  known.update();
  const int c = (nd.dem.side - 1) / 2;
  EXPECT_TRUE(known.known(c - 20, c));   // x = -20: traversed
  EXPECT_FALSE(known.known(c + 10, c));  // x = 10: not
}

TEST(NavMapDem, OriginFileRoundTripsAndItsAbsenceMeansNoGeolocation)
{
  const std::string dir = std::filesystem::temp_directory_path().string() + "/lmm_origin_test";
  std::filesystem::create_directories(dir);
  const std::string map = dir + "/some_map.navmap";
  EXPECT_EQ(bigmap_maps_manager::origin_file_for(map), dir + "/some_map.yaml");
  std::filesystem::remove(dir + "/some_map.yaml");

  auto none = bigmap_maps_manager::load_origin(map);
  EXPECT_FALSE(none.georeferenced);
  EXPECT_EQ(none.lat, 0.0);
  EXPECT_EQ(none.altitude, 0.0);

  bigmap_maps_manager::MapOrigin o{true, 40.284796, -3.821407, 675.25};
  ASSERT_TRUE(bigmap_maps_manager::save_origin(map, o, 1.0));
  const auto back = bigmap_maps_manager::load_origin(map);
  EXPECT_TRUE(back.georeferenced);
  EXPECT_NEAR(back.lat, 40.284796, 1e-9);
  EXPECT_NEAR(back.lon, -3.821407, 1e-9);
  EXPECT_NEAR(back.altitude, 675.25, 1e-9);

  ASSERT_TRUE(bigmap_maps_manager::save_origin(map, bigmap_maps_manager::MapOrigin{}, 1.0));
  EXPECT_FALSE(bigmap_maps_manager::load_origin(map).georeferenced);
  // "nan" and "inf" are numbers to std::stod: a georeferenced origin nowhere was accepted
  for (const char * text : {"georeferenced: true\norigin_lat: nan\norigin_lon: -3.8\n",
      "georeferenced: true\norigin_lat: 40.2\norigin_lon: inf\n",
      "georeferenced: true\norigin_lat: 40.2\norigin_lon: -3.8\nreference_altitude: -inf\n"})
  {
    std::ofstream(dir + "/some_map.yaml", std::ios::trunc) << text;
    EXPECT_FALSE(bigmap_maps_manager::load_origin(map).georeferenced) << text;
  }
  std::filesystem::remove_all(dir);
}

TEST(NavMapDem, RefusesAnEmptyMapAndAHugeGrid)
{
  EXPECT_THROW(dem_from_navmap(::navmap::NavMap(), 1.0), std::invalid_argument);
  EXPECT_THROW(dem_from_navmap(make_map(2.0, false, false), 0.001), std::invalid_argument);
}

namespace
{
// Flat strips of 1 m NavCels over [x0, x1] x [-5, 5] (one per pair), z = 0
::navmap::NavMap strips(const std::vector<std::pair<int, int>> & spans)
{
  ::navmap::NavMap nm;
  for (const auto & [x0, x1] : spans) {
    const auto base = static_cast<std::uint32_t>(nm.positions.x.size());
    const int nx = x1 - x0;
    for (int j = 0; j <= 10; ++j) {
      for (int i = 0; i <= nx; ++i) {
        nm.positions.x.push_back(static_cast<float>(x0 + i));
        nm.positions.y.push_back(static_cast<float>(j - 5));
        nm.positions.z.push_back(0.0f);
      }
    }
    auto vid = [&](int i, int j) {return base + static_cast<std::uint32_t>(j * (nx + 1) + i);};
    for (int j = 0; j < 10; ++j) {
      for (int i = 0; i < nx; ++i) {
        ::navmap::NavCel a, b;
        a.v[0] = vid(i, j); a.v[1] = vid(i + 1, j); a.v[2] = vid(i + 1, j + 1);
        b.v[0] = vid(i, j); b.v[1] = vid(i + 1, j + 1); b.v[2] = vid(i, j + 1);
        nm.navcels.push_back(a);
        nm.navcels.push_back(b);
      }
    }
  }
  return nm;
}

bool valid_at(const bigmap_maps_manager::NavMapDem & d, int x, int y)
{
  const int c = (d.dem.side - 1) / 2;
  return d.valid[static_cast<std::size_t>(y + c) * d.dem.side + (x + c)] != 0;
}
}  // namespace

TEST(NavMapDem, BordersDoNotGrowAndAGapOfTwoNodesStays)
{
  // a platform over x 0..4 and another over x 7..10: nodes x = 5, 6 are a gap of 2 nodes
  const auto d = dem_from_navmap(strips({{0, 4}, {7, 10}}), 1.0);
  EXPECT_EQ(d.covered_nodes, 11u * (5u + 4u));   // exactly the nodes on the NavCels
  for (int y = -5; y <= 5; ++y) {
    EXPECT_FALSE(valid_at(d, -1, y)) << y;     // not past the border
    EXPECT_FALSE(valid_at(d, 5, y)) << y;      // the gap stays
    EXPECT_FALSE(valid_at(d, 6, y)) << y;
    EXPECT_FALSE(valid_at(d, 11, y)) << y;
  }
  // a gap of one node (sampling) is closed
  const auto one = dem_from_navmap(strips({{0, 4}, {6, 10}}), 1.0);
  for (int y = -4; y <= 4; ++y) {
    EXPECT_TRUE(valid_at(one, 5, y)) << y;
  }
}

TEST(NavMapDem, ANaNHeightIsNoData)
{
  auto nm = strips({{0, 4}});
  nm.positions.z[0] = std::nanf("");   // the vertex at (0, -5)
  const auto d = dem_from_navmap(nm, 1.0);
  for (std::size_t k = 0; k < d.dem.z.size(); ++k) {
    EXPECT_TRUE(std::isfinite(d.dem.z[k])) << k;
  }
}

TEST(NavMapDem, AbsurdOrNonFiniteVertexCoordinatesNeitherSizeTheGridNorOverflowItsIndices)
{
  // a vertex at 3e38 m made `ceil(half / resolution)` overflow its cast to int (undefined:
  // INT_MIN, a negative side that passed the max_side check), and a NaN one passed every
  // comparison of the rasteriser and was cast to an index
  const auto reference = dem_from_navmap(strips({{0, 4}}), 1.0);
  {
    auto nm = strips({{0, 4}});
    nm.positions.x[0] = 3.0e38f;
    EXPECT_THROW(dem_from_navmap(nm, 1.0), std::invalid_argument);   // too big a grid, said so
  }
  for (const float bad : {std::nanf(""), std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity()})
  {
    auto nm = strips({{0, 4}});
    nm.positions.x[0] = bad;
    nm.positions.y[1] = bad;
    const auto d = dem_from_navmap(nm, 1.0);
    EXPECT_EQ(d.dem.side, reference.dem.side) << bad;   // the grid is sized by the finite ones
    for (std::size_t k = 0; k < d.dem.z.size(); ++k) {
      ASSERT_TRUE(std::isfinite(d.dem.z[k])) << k;
    }
  }
}

namespace
{
// make_map(1.0) without some cells of 1 m: one alone at (5, 5), one at (-12, 3), a 2 x 2 block at
// (15..16, -8..-7). Those are obstacles of one or two cells.
const std::vector<std::pair<int, int>> kHoles{{5, 5}, {-12, 3}, {15, -8}, {16, -8}, {15, -7},
  {16, -7}};

::navmap::NavMap holed_map()
{
  auto nm = make_map(1.0, false, false);
  std::vector<::navmap::NavCel> kept;
  for (const auto & c : nm.navcels) {
    double cx = 0, cy = 0;
    for (int k = 0; k < 3; ++k) {
      cx += nm.positions.x[c.v[k]] / 3.0;
      cy += nm.positions.y[c.v[k]] / 3.0;
    }
    bool hole = false;
    for (const auto & [hx, hy] : kHoles) {
      hole = hole || (std::floor(cx) == hx && std::floor(cy) == hy);
    }
    if (!hole) {kept.push_back(c);}
  }
  nm.navcels = kept;
  nm.surfaces[0].navcels.resize(kept.size());
  for (std::size_t t = 0; t < kept.size(); ++t) {
    nm.surfaces[0].navcels[t] = static_cast<::navmap::NavCelId>(t);
  }
  return nm;
}

// Is the point inside some triangle of the mesh (in xy)?
bool covered(const bigmap_maps_manager::LodMesh & m, double px, double py)
{
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], c = m.tri[3 * t + 2];
    const auto side = [&](std::uint32_t u, std::uint32_t v) {
        return (m.x[v] - m.x[u]) * (py - m.y[u]) - (m.y[v] - m.y[u]) * (px - m.x[u]);
      };
    const double s1 = side(a, b), s2 = side(b, c), s3 = side(c, a);
    if ((s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0)) {return true;}
  }
  return false;
}
}  // namespace

TEST(NavMapDem, EveryHoleOfTheMapIsACellWithoutGroundWhateverItsSize)
{
  const auto nd = dem_from_navmap(holed_map(), 1.0);
  const int side = nd.dem.side, nc = side - 1, c = (side - 1) / 2;
  ASSERT_EQ(nd.cell_valid.size(), static_cast<std::size_t>(nc) * nc);
  const auto cell = [&](int x, int y) {
      return nd.cell_valid[static_cast<std::size_t>(y + c) * nc + (x + c)];
    };
  for (const auto & [hx, hy] : kHoles) {
    EXPECT_EQ(cell(hx, hy), 0) << hx << "," << hy;
  }
  EXPECT_EQ(cell(4, 5), 1);
  EXPECT_EQ(cell(6, 5), 1);
  EXPECT_EQ(cell(14, -8), 1);
  EXPECT_EQ(cell(0, 0), 1);
  EXPECT_EQ(cell(29, 19), 1);
  EXPECT_EQ(cell(-31, 0), 0);    // beyond the map
}

TEST(LodMesher, AtFullResolutionTheHolesOfTheMapAreExactlyWhereTheyAre)
{
  // The node rule (a NavCel exists if the node under its centre has ground) closed every hole
  // narrower than three cells: the obstacles of one or two cells were driven through. With the
  // cell mask a NavCel exists if the map has ground under its centroid.
  const auto nd = dem_from_navmap(holed_map(), 1.0);
  bigmap_maps_manager::LodParams p;
  p.high_res_radius = 60.0;   // all of it at full resolution
  p.low_res_distance = 100.0;
  p.min_size = 1.0;
  p.max_size = 8.0;
  const bigmap_maps_manager::LodMesher old_rule(nd.dem, nd.valid);
  const bigmap_maps_manager::LodMesher exact(nd.dem, nd.valid, {}, nd.cell_valid);
  const auto before = old_rule.extract(0.0, 0.0, p);
  const auto now = exact.extract(0.0, 0.0, p);
  for (const auto & [hx, hy] : kHoles) {
    EXPECT_TRUE(covered(before, hx + 0.5, hy + 0.5)) << "the old rule closed it";
    EXPECT_FALSE(covered(now, hx + 0.5, hy + 0.5)) << hx << "," << hy;
  }
  // and the ground around them is there
  for (const auto & [x, y] : std::vector<std::pair<double, double>>{{4.5, 5.5}, {5.5, 6.5},
      {-10.5, 3.5}, {14.5, -7.5}, {0.5, 0.5}})
  {
    EXPECT_TRUE(covered(now, x, y)) << x << "," << y;
  }
}

TEST(LodMesher, AHoleStaysOpenFarFromTheRobotWithAHoleErrorOrOnceObserved)
{
  const auto nd = dem_from_navmap(holed_map(), 1.0);
  bigmap_maps_manager::LodParams p;
  p.high_res_radius = 5.0;
  p.low_res_distance = 40.0;
  p.min_size = 1.0;
  p.max_size = 16.0;
  p.base_error = 0.02;
  p.error_growth = 0.05;
  p.known_margin = 1.0;
  p.known_falloff = 4.0;
  // the robot 40 m from the hole at (15, -8): a coarse NavCel covers it, and it is not emitted
  // only if the cell under its centroid happens to be the hole
  const bigmap_maps_manager::LodMesher plain(nd.dem, nd.valid, {}, nd.cell_valid);
  EXPECT_TRUE(covered(plain.extract(-25.0, 0.0, p), 15.5, -7.5));
  // a hole error above the tolerance at that distance (0.02 + 0.05 * (40 - 5 - ...)) splits down
  // to the hole's border
  const bigmap_maps_manager::LodMesher strict(nd.dem, nd.valid, {}, nd.cell_valid, 5.0);
  const auto far = strict.extract(-25.0, 0.0, p);
  for (const auto & [hx, hy] : kHoles) {
    EXPECT_FALSE(covered(far, hx + 0.5, hy + 0.5)) << hx << "," << hy;
  }
  EXPECT_LT(far.triangle_count(), strict.extract(15.0, -8.0, p).triangle_count() * 4);
  // without a hole error, the border observed from near keeps it
  bigmap_maps_manager::KnownMap observed(nd.dem.side, 1.0);
  for (const auto & [hx, hy] : kHoles) {
    observed.mark(hx + 0.5, hy + 0.5, 1.0);
  }
  observed.update();
  const auto kept = plain.extract(-25.0, 0.0, p, nullptr, &observed);
  for (const auto & [hx, hy] : kHoles) {
    EXPECT_FALSE(covered(kept, hx + 0.5, hy + 0.5)) << hx << "," << hy;
  }
}

TEST(NavMapDem, TheMeshFromTheArraysOfAMapIsTheMeshFromTheMap)
{
  const auto nm = holed_map();
  const auto a = dem_from_navmap(nm, 1.0);
  const auto b = dem_from_navmap(bigmap_maps_manager::SourceMesh::from_navmap(nm), 1.0);
  EXPECT_EQ(a.dem.z, b.dem.z);
  EXPECT_EQ(a.valid, b.valid);
  EXPECT_EQ(a.cell_valid, b.cell_valid);
  auto broken = bigmap_maps_manager::SourceMesh::from_navmap(nm);
  broken.v2[3] = static_cast<std::uint32_t>(broken.x.size());
  EXPECT_THROW(broken.validate(), std::invalid_argument);
  broken = bigmap_maps_manager::SourceMesh::from_navmap(nm);
  broken.traversed.assign(2, 1);
  EXPECT_THROW(broken.validate(), std::invalid_argument);
}
