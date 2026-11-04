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

#include <array>
#include <vector>

#include "easynav_bigmap_maps_manager/dynamic_layer.hpp"
#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/obstacle_memory.hpp"

using bigmap_maps_manager::DynamicLayer;
using bigmap_maps_manager::DynamicParams;
using bigmap_maps_manager::ObstacleMemory;
using bigmap_maps_manager::SustainedHit;

namespace
{
// a box 0.5 m wide and 1 m high in front of the robot, seen from a robot at the origin
std::vector<std::array<float, 3>> box(float x, float y)
{
  std::vector<std::array<float, 3>> pts;
  for (float dx = 0.0f; dx <= 0.4f; dx += 0.1f) {
    for (float dy = 0.0f; dy <= 0.4f; dy += 0.1f) {
      for (float z = 0.0f; z <= 1.0f; z += 0.1f) {
        pts.push_back({x + dx, y + dy, z});
      }
    }
  }
  return pts;
}

DynamicParams params()
{
  DynamicParams p;
  p.ground_fit = false;
  p.confirm_time = 0.3;
  p.min_vertical_extent = 0.2;
  return p;
}

const DynamicLayer::Surface flat = [](double, double) {return 0.0;};
}  // namespace

TEST(ObstacleMemory, ADetectionOverFiveSecondsIsOneHitAndTenSecondsAreTwo)
{
  DynamicLayer layer;
  const auto p = params();
  const auto pts = box(2.0f, 0.0f);
  std::size_t hits = 0;
  for (double t = 0.0; t <= 11.0; t += 0.1) {
    layer.ingest(pts, 0.0, 0.0, 0.0, t, flat, p);
    layer.expire(t, p);
    const auto h = layer.sustained(t, 5.0);
    if (t < 4.9) {EXPECT_TRUE(h.empty()) << "t=" << t;}
    hits += h.empty() ? 0 : 1;
  }
  EXPECT_GE(hits, 2u);   // at 5 s and at 10 s (each detection cell reports its own periods)
}

TEST(ObstacleMemory, AGapOverASecondStartsTheCountAgain)
{
  DynamicLayer layer;
  const auto p = params();
  const auto pts = box(2.0f, 0.0f);
  for (double t = 0.0; t <= 4.0; t += 0.1) {
    layer.ingest(pts, 0.0, 0.0, 0.0, t, flat, p);
    layer.expire(t, p);
  }
  // not seen for 2 s (still remembered: forget_time is 5 s), then seen again
  for (double t = 6.0; t <= 9.0; t += 0.1) {
    layer.ingest(pts, 0.0, 0.0, 0.0, t, flat, p);
    layer.expire(t, p);
    EXPECT_TRUE(layer.sustained(t, 5.0).empty()) << "t=" << t;   // 3 s since the gap
  }
}

TEST(ObstacleMemory, HitsAddUpToMaxHitsAndTheObstacleIsThenFixed)
{
  ObstacleMemory mem(1.0, 5.0, 3);
  const std::vector<SustainedHit> d{{3.2f, 4.1f, 0.5f, 0.0f}, {3.4f, 4.3f, 0.7f, 0.0f}};
  EXPECT_TRUE(mem.add(d, 5.0));     // two detections in one spot: one hit
  ASSERT_EQ(mem.cells().size(), 1u);
  EXPECT_EQ(mem.cells().begin()->second.hits, 1);
  EXPECT_FALSE(mem.fixed(mem.cells().begin()->second));
  EXPECT_FALSE(mem.add(d, 6.0));    // half a period later: the same period
  EXPECT_TRUE(mem.add(d, 10.0));
  EXPECT_EQ(mem.cells().begin()->second.hits, 2);
  EXPECT_TRUE(mem.add(d, 15.0));
  EXPECT_TRUE(mem.fixed(mem.cells().begin()->second));
  EXPECT_EQ(mem.fixed_count(), 1u);
  EXPECT_FALSE(mem.add(d, 20.0));   // saturated
  EXPECT_EQ(mem.cells().begin()->second.hits, 3);
  mem.clear();
  EXPECT_TRUE(mem.cells().empty());
}

TEST(ObstacleMemory, SpotsApartAndFloorsApartAreCountedApart)
{
  ObstacleMemory mem(1.0, 5.0, 3);
  mem.add({{0.2f, 0.2f, 0.0f, 0.0f}, {5.2f, 0.2f, 0.0f, 0.0f}, {0.2f, 0.2f, 2.2f, 2.2f}}, 5.0);
  EXPECT_EQ(mem.cells().size(), 3u);
}

TEST(ObstacleMemory, StaggeredDetectionsOfOneSpotDoNotFixItSooner)
{
  // two detection cells of the same 1 m spot, first seen 2.6 s apart: each reports its own
  // periods (A at 5, 10, 15 s; B at 7.6, 12.6 s). The spot is fixed at 15 s, not at 12.6 s
  ObstacleMemory mem(1.0, 5.0, 3);
  const SustainedHit a{3.2f, 4.1f, 0.5f, 0.0f}, b{3.7f, 4.6f, 0.5f, 0.0f};
  mem.add({a}, 5.0);
  mem.add({b}, 7.6);
  mem.add({a}, 10.0);
  mem.add({b}, 12.6);
  ASSERT_EQ(mem.cells().size(), 1u);
  EXPECT_EQ(mem.cells().begin()->second.hits, 2);
  EXPECT_FALSE(mem.fixed(mem.cells().begin()->second));
  mem.add({a}, 15.0);
  EXPECT_TRUE(mem.fixed(mem.cells().begin()->second));
}

TEST(ObstacleMemory, RestoredCountsCarryOnFromWhereTheSavedMapLeftThem)
{
  ObstacleMemory mem(1.0, 5.0, 3);
  mem.restore(3.2, 4.1, 0.5, 2);
  mem.restore(9.0, 9.0, 0.0, 99);   // beyond max_hits: clamped, fixed
  ASSERT_EQ(mem.cells().size(), 2u);
  EXPECT_EQ(mem.fixed_count(), 1u);
  mem.restore(3.4, 4.3, 0.5, 1);    // the same spot: it keeps what it had
  EXPECT_EQ(mem.cells().size(), 2u);
  // a new detection adds to it at once (the saved hit was long ago) and fixes it
  mem.add({{3.2f, 4.1f, 0.5f, 0.5f}}, 5.0);
  EXPECT_EQ(mem.fixed_count(), 2u);
}
