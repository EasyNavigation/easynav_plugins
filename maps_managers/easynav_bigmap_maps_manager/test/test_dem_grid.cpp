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

// The DEM file is also written by navmap_tools' navmap_gis_tool (Python, --dem-out). Both sides
// keep the same golden file (test/data/dem_golden.bin, made once by the C++ DemGrid::build and
// save of the former bigmap_dem package from a smooth function: 9 x 9 nodes of 1 m around
// 40.4, -3.7, sampled every 2 m and bicubically interpolated). This side checks that it loads and
// saves back byte by byte; navmap_tools' test checks that its writer reproduces the bytes and
// that its sampling agrees with the values.

#include <gtest/gtest.h>

#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <random>
#include <string>

#include "easynav_bigmap_maps_manager/dem_grid.hpp"

using bigmap_maps_manager::DemGrid;

namespace
{
std::string golden()
{
  return (std::filesystem::path(BIGMAP_TEST_DATA) / "dem_golden.bin").string();
}

std::string bytes(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}
}  // namespace

TEST(DemGrid, GoldenFileLoadsWithItsHeaderAndHeights)
{
  const auto g = DemGrid::load(golden());
  ASSERT_TRUE(g.has_value());
  EXPECT_EQ(g->side, 9);
  EXPECT_EQ(g->source, DemGrid::kCopernicus);
  EXPECT_DOUBLE_EQ(g->resolution, 1.0);
  EXPECT_DOUBLE_EQ(g->data_resolution, 2.0);
  EXPECT_DOUBLE_EQ(g->radius, 4.0);
  EXPECT_DOUBLE_EQ(g->origin_lat, 40.4);
  EXPECT_DOUBLE_EQ(g->origin_lon, -3.7);
  EXPECT_TRUE(std::isnan(g->requested_reference));
  EXPECT_NEAR(g->reference_altitude, 694.392, 1e-3);
  ASSERT_EQ(g->z.size(), 81u);
  EXPECT_TRUE(g->rgb.empty());
  EXPECT_NEAR(g->at(4, 4), 0.0, 1e-6) << "the origin is the reference";
  EXPECT_NEAR(g->at(0, 0), -25.29486, 1e-4);
  EXPECT_NEAR(g->at(8, 8), 3.52368, 1e-4);
  EXPECT_NEAR(g->at(6, 2), -2.40851, 1e-4);   // i = east, j = north
  EXPECT_NEAR(g->at(0, 8), -11.6571, 1e-3);
}

TEST(DemGrid, SavedAgainItIsTheSameFileByteForByte)
{
  const auto g = DemGrid::load(golden());
  ASSERT_TRUE(g.has_value());
  const auto out = (std::filesystem::temp_directory_path() / "bigmap_dem_grid_again.bin").string();
  ASSERT_TRUE(g->save(out));
  EXPECT_EQ(bytes(out), bytes(golden()));
  std::filesystem::remove(out);
}

TEST(DemGrid, ColoursSurviveASaveAndLoad)
{
  auto g = *DemGrid::load(golden());
  g.imagery_zoom = 18;
  g.rgb.assign(81 * 3, 7);
  const auto out = (std::filesystem::temp_directory_path() / "bigmap_dem_grid_rgb.bin").string();
  ASSERT_TRUE(g.save(out));
  const auto back = DemGrid::load(out);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->imagery_zoom, 18);
  EXPECT_EQ(back->rgb.size(), 81u * 3u);
  std::filesystem::remove(out);
}

TEST(DemGrid, TheCellsWithGroundSurviveASaveAndLoadAndAnOlderFileHasNone)
{
  auto g = *DemGrid::load(golden());
  EXPECT_TRUE(g.cell_valid.empty()) << "the golden file is older than the cells block";
  g.imagery_zoom = 18;
  g.rgb.assign(81 * 3, 7);
  g.cell_valid.assign(64, 1);
  g.cell_valid[3 * 8 + 5] = 0;   // a hole at cell (5, 3)
  const auto out = (std::filesystem::temp_directory_path() / "bigmap_dem_grid_cells.bin").string();
  ASSERT_TRUE(g.save(out));
  const auto back = DemGrid::load(out);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->cell_valid, g.cell_valid);
  EXPECT_EQ(back->rgb.size(), 81u * 3u);
  // a cells block cut short or with another tag is a broken file, not a file without cells
  const auto all = bytes(out);
  for (const std::size_t cut : {std::size_t{1}, std::size_t{30}, std::size_t{64}}) {
    std::ofstream(out, std::ios::binary | std::ios::trunc) << all.substr(0, all.size() - cut);
    EXPECT_FALSE(DemGrid::load(out).has_value()) << "cut " << cut;
  }
  std::string other = all;
  other[other.size() - 64 - 8] = 'X';
  std::ofstream(out, std::ios::binary | std::ios::trunc) << other;
  EXPECT_FALSE(DemGrid::load(out).has_value());
  // the wrong number of cells is not written
  g.cell_valid.resize(10);
  EXPECT_FALSE(g.save(out));
  std::filesystem::remove(out);
}

TEST(DemGrid, SubsampleKeepsEveryHole)
{
  auto g = *DemGrid::load(golden());
  g.cell_valid.assign(64, 1);
  g.cell_valid[5 * 8 + 2] = 0;   // cell (2, 5): in the coarse cell (1, 2)
  const auto s = g.subsample(2);
  ASSERT_EQ(s.cell_valid.size(), 16u);
  for (int j = 0; j < 4; ++j) {
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(s.cell_valid[j * 4 + i], (i == 1 && j == 2) ? 0 : 1) << i << ", " << j;
    }
  }
}

TEST(DemGrid, HeightAtIsBilinearAndClampedToTheGrid)
{
  const auto g = *DemGrid::load(golden());
  EXPECT_NEAR(g.height_at(0.0, 0.0), 0.0, 1e-6);
  EXPECT_NEAR(g.height_at(g.x(6), g.y(2)), g.at(6, 2), 1e-5);
  EXPECT_NEAR(g.height_at(0.5, 0.0), 0.5 * (g.at(4, 4) + g.at(5, 4)), 1e-5);
  EXPECT_NEAR(g.height_at(100.0, 100.0), g.at(8, 8), 1e-6);
}

TEST(DemGrid, ANonFiniteCoordinateHasNoHeightInsteadOfReadingOutsideTheGrid)
{
  // a NaN went through std::clamp and the int cast (undefined: INT_MIN on x86) into z[]
  const auto g = *DemGrid::load(golden());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(std::isnan(g.height_at(nan, 0.0)));
  EXPECT_TRUE(std::isnan(g.height_at(0.0, nan)));
  EXPECT_TRUE(std::isnan(g.height_at(inf, 0.0)));
  EXPECT_TRUE(std::isnan(g.height_at(0.0, -inf)));
  EXPECT_NEAR(g.height_at(1e300, -1e300), g.at(8, 0), 1e-6);   // huge but finite: clamped
}

TEST(DemGrid, SubsampleKeepsEveryNthNodeAndRefusesWhatDoesNotDivide)
{
  const auto g = *DemGrid::load(golden());
  const auto s = g.subsample(2);
  EXPECT_EQ(s.side, 5);
  EXPECT_DOUBLE_EQ(s.resolution, 2.0);
  EXPECT_FLOAT_EQ(s.at(1, 3), g.at(2, 6));
  EXPECT_THROW(g.subsample(3), std::invalid_argument);
}

TEST(DemGrid, MissingOrBrokenFilesAreNotLoaded)
{
  EXPECT_FALSE(DemGrid::load("/nonexistent/dem.bin").has_value());
  const auto out = (std::filesystem::temp_directory_path() / "bigmap_dem_grid_broken.bin").string();
  {
    std::ofstream f(out, std::ios::binary);
    f << "LMDEM006 not really";
  }
  EXPECT_FALSE(DemGrid::load(out).has_value());
  std::filesystem::remove(out);
}

namespace
{
// A DEM file written by hand: `side` and `resolution` as given, the heights and (optionally) the
// colours cut at `payload` floats / bytes, to tell what the loader trusts from what it checks
std::string craft(
  std::int32_t side, double resolution, std::size_t floats, std::int32_t zoom = 0,
  std::size_t rgb_bytes = 0)
{
  std::string out("LMDEM006", 8);
  const std::int32_t source = 0;
  const double hdr[7] = {resolution, 4.0, 40.4, -3.7, 600.0, 600.0, 2.0};
  out.append(reinterpret_cast<const char *>(&side), sizeof(side));
  out.append(reinterpret_cast<const char *>(&source), sizeof(source));
  out.append(reinterpret_cast<const char *>(hdr), sizeof(hdr));
  out.append(floats * sizeof(float), '\0');
  out.append(reinterpret_cast<const char *>(&zoom), sizeof(zoom));
  out.append(rgb_bytes, '\0');
  return out;
}

std::optional<DemGrid> load_bytes(const std::string & content)
{
  const auto path = (std::filesystem::temp_directory_path() /
    ("bigmap_dem_grid_crafted_" + std::to_string(::getpid()) + ".bin")).string();
  {
    std::ofstream f(path, std::ios::binary);
    f.write(content.data(), static_cast<std::streamsize>(content.size()));
  }
  auto g = DemGrid::load(path);
  std::filesystem::remove(path);
  return g;
}
}  // namespace

TEST(DemGrid, AHeaderThatPromisesMoreHeightsThanTheFileHasIsRefusedBeforeAllocating)
{
  // 19999 x 19999 floats = 1.6 GB claimed by a 60-byte file
  EXPECT_FALSE(load_bytes(craft(19999, 1.0, 10)).has_value());
}

TEST(DemGrid, ASideOfOneIsNotAGrid)
{
  // height_at() reads node (side - 2) = -1 of it
  EXPECT_FALSE(load_bytes(craft(1, 1.0, 1)).has_value());
}

TEST(DemGrid, ANonPositiveOrNonFiniteResolutionIsRefused)
{
  for (const double r : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_FALSE(load_bytes(craft(9, r, 81)).has_value()) << "resolution " << r;
  }
  EXPECT_TRUE(load_bytes(craft(9, 1.0, 81)).has_value());
}

TEST(DemGrid, AnImageryZoomWithoutAllItsColoursOrOutOfRangeIsRefused)
{
  EXPECT_FALSE(load_bytes(craft(9, 1.0, 81, 18, 10)).has_value());     // colours cut short
  EXPECT_TRUE(load_bytes(craft(9, 1.0, 81, 18, 81 * 3)).has_value());
  EXPECT_FALSE(load_bytes(craft(9, 1.0, 81, -5, 81 * 3)).has_value());  // negative zoom
}

TEST(DemGrid, RandomBytesAfterAValidHeaderNeverCrashTheLoader)
{
  std::mt19937 rng(12345);   // fixed seed: a failure is reproducible
  for (int n = 0; n < 300; ++n) {
    std::string content = craft(9, 1.0, 81, n % 2 ? 18 : 0, n % 2 ? 81 * 3 : 0);
    const std::size_t flips = 1 + rng() % 8;
    for (std::size_t k = 0; k < flips; ++k) {
      content[rng() % content.size()] = static_cast<char>(rng());
    }
    if (n % 5 == 0) {content.resize(rng() % content.size());}
    const auto g = load_bytes(content);
    if (g) {   // whatever loads must be a usable grid
      EXPECT_EQ(g->z.size(), static_cast<std::size_t>(g->side) * g->side);
      EXPECT_GE(g->side, 3);
      EXPECT_TRUE(g->resolution > 0.0 && std::isfinite(g->resolution));
    }
  }
}
