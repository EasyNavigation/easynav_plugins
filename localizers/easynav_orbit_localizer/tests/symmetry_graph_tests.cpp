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

/// \brief The symmetry graph: twin rooms give alternatives, an L room none, a square room its
/// turns; the global match finds a scan's pose; the graph survives a save and a load.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "easynav_orbit_localizer/SymmetryGraph.hpp"
#include "test_maps.hpp"

using namespace easynav::orbit;  // NOLINT

namespace
{
struct Built
{
  std::unique_ptr<easynav::Costmap2D> map;
  std::unique_ptr<GridDistanceField2D> field;
  std::unique_ptr<GridViews2D> views;
  std::unique_ptr<Matcher> matcher;
  SymmetryGraph graph;
  GraphOptions options;
};

std::unique_ptr<Built> build(const easynav::Costmap2D & map, double step = 0.5)
{
  auto b = std::make_unique<Built>();
  b->map = std::make_unique<easynav::Costmap2D>(map);
  b->field = std::make_unique<GridDistanceField2D>(*b->map);
  b->views = std::make_unique<GridViews2D>(*b->field, 12.0, step);
  MatchOptions opts;
  opts.mode = Mode::Planar;
  opts.inlier_distance = 0.1;
  b->matcher = std::make_unique<Matcher>(*b->field, opts);
  b->options.sample_step = step;
  b->options.threads = 4;
  b->graph.build(*b->views, *b->matcher, b->options);
  return b;
}
}  // namespace

TEST(GridViews2D, ViewsOfTheMapAndOfAScanAgree)
{
  const auto map = test_maps::lRoom();
  GridDistanceField2D field(map);
  GridViews2D views(field, 12.0, 0.5);
  const Pose X = fromXYZYaw(2.0, 2.0, 0.0, 0.0);
  const View from_map = views.fromMap(X.translation());
  const View from_scan = views.fromScan(test_maps::scan(field, X));
  EXPECT_LT(SymmetryGraph::descriptorDistance(from_map.descriptor, from_scan.descriptor), 0.1);
  // The same place turned: same descriptor, profile shifted by the turn
  const Pose turned = fromXYZYaw(2.0, 2.0, 0.0, 1.0);
  const View turned_scan = views.fromScan(test_maps::scan(field, turned));
  EXPECT_LT(SymmetryGraph::descriptorDistance(from_map.descriptor, turned_scan.descriptor), 0.1);
  const auto seeds = SymmetryGraph::yawSeeds(from_map.profile, turned_scan.profile, 3);
  ASSERT_FALSE(seeds.empty());
  EXPECT_NEAR(seeds.front(), 1.0, 2.0 * 2.0 * M_PI / 72.0);
  // The samples are free cells away from the walls
  for (const auto & p : views.samplePositions()) {
    EXPECT_TRUE(field.isFree(p.x(), p.y()));
    EXPECT_GE(field.distance(p, nullptr), 0.25);
  }
}

TEST(SymmetryGraph, TwinRoomsAreEachOthersAlternative)
{
  auto b = build(test_maps::twinRooms());
  ASSERT_GT(b->graph.nodes().size(), 20u);
  // A place in the left room has its twin 5 m to the right, heading the same
  const Pose X = fromXYZYaw(2.3, 3.1, 0.0, 0.4);
  const auto twins = b->graph.twins(X);
  ASSERT_FALSE(twins.empty()) << "no alternatives for a place of the left room";
  bool found = false;
  for (const auto & t : twins) {
    if (std::abs(t.pose.translation().x() - 7.3) < 0.3 &&
      std::abs(t.pose.translation().y() - 3.1) < 0.3 && test_maps::yawError(t.pose, X) < 0.2)
    {
      found = true;
      EXPECT_GT(t.score, 0.8);
    }
  }
  EXPECT_TRUE(found);
  // And the other way round
  const auto back = b->graph.twins(fromXYZYaw(7.3, 3.1, 0.0, 0.4));
  bool found_back = false;
  for (const auto & t : back) {
    if (std::abs(t.pose.translation().x() - 2.3) < 0.3) {found_back = true;}
  }
  EXPECT_TRUE(found_back);
}

TEST(SymmetryGraph, AnAsymmetricRoomHasNoAlternatives)
{
  auto b = build(test_maps::lRoom());
  ASSERT_GT(b->graph.nodes().size(), 20u);
  EXPECT_EQ(b->graph.alternativesCount(), 0u);
  for (const auto & n : b->graph.nodes()) {
    EXPECT_TRUE(n.alternatives.empty()) << "place (" << n.position.x() << ", " << n.position.y()
                                        << ") has " << n.alternatives.size() << " alternatives";
  }
  EXPECT_TRUE(b->graph.twins(fromXYZYaw(2.0, 2.0, 0.0, 0.0)).empty());
}

TEST(SymmetryGraph, ASquareRoomLooksTheSameTurnedByQuarterTurns)
{
  auto b = build(test_maps::emptyRoom(5.0, 5.0));
  const Pose centre = fromXYZYaw(2.5, 2.5, 0.0, 0.0);
  const auto twins = b->graph.twins(centre);
  // Three other headings at the centre
  int turns = 0;
  for (const auto & t : twins) {
    if ((t.pose.translation() - centre.translation()).norm() < 0.3) {++turns;}
  }
  EXPECT_GE(turns, 3);
  // Off centre, the twins are the place turned about the centre
  const Pose off = fromXYZYaw(1.0, 1.0, 0.0, 0.0);
  bool mirrored = false;
  for (const auto & t : b->graph.twins(off)) {
    if (std::abs(t.pose.translation().x() - 4.0) < 0.3 &&
      std::abs(t.pose.translation().y() - 4.0) < 0.3 &&
      test_maps::yawError(t.pose, fromXYZYaw(0.0, 0.0, 0.0, M_PI)) < 0.2)
    {
      mirrored = true;
    }
  }
  EXPECT_TRUE(mirrored);
}

TEST(SymmetryGraph, GlobalMatchFindsWhereAScanWasTaken)
{
  auto b = build(test_maps::twinRooms());
  const Pose truth = fromXYZYaw(1.7, 2.8, 0.0, -0.9);
  const auto pts = test_maps::scan(*b->field, truth, 360, 12.0, 0.01);
  const auto cands = b->graph.globalMatch(*b->views, *b->matcher, pts, 20, 3, 0.6);
  ASSERT_GE(cands.size(), 2u) << "the twin rooms give two places";
  bool truth_found = false, twin_found = false;
  for (const auto & c : cands) {
    if (test_maps::planarError(c.pose, truth) < 0.1 && test_maps::yawError(c.pose, truth) < 0.05) {
      truth_found = true;
      EXPECT_GT(c.score, 0.85);
    }
    if (std::abs(c.pose.translation().x() - 6.7) < 0.2 &&
      std::abs(c.pose.translation().y() - 2.8) < 0.2) {twin_found = true;}
  }
  EXPECT_TRUE(truth_found);
  EXPECT_TRUE(twin_found);
  // Best first
  for (std::size_t k = 1; k < cands.size(); ++k) {
    EXPECT_LE(cands[k].score, cands[k - 1].score);
                                                                                                }
  // A scan in the L room has one place
  auto l = build(test_maps::lRoom());
  const Pose t2 = fromXYZYaw(2.0, 6.0, 0.0, 2.0);
  const auto c2 = l->graph.globalMatch(
    *l->views, *l->matcher, test_maps::scan(*l->field, t2), 20, 3, 0.7);
  ASSERT_FALSE(c2.empty());
  EXPECT_LT(test_maps::planarError(c2.front().pose, t2), 0.1);
  EXPECT_LT(test_maps::yawError(c2.front().pose, t2), 0.05);
  EXPECT_TRUE(c2.size() == 1u || c2[1].score < 0.9);
}

TEST(SymmetryGraph, SaveAndLoad)
{
  auto b = build(test_maps::twinRooms());
  const std::string path = "/tmp/symmetry_graph_test.bin";
  ASSERT_TRUE(b->graph.save(path));
  SymmetryGraph loaded;
  ASSERT_TRUE(loaded.load(path));
  std::remove(path.c_str());
  ASSERT_EQ(loaded.nodes().size(), b->graph.nodes().size());
  EXPECT_EQ(loaded.alternativesCount(), b->graph.alternativesCount());
  EXPECT_DOUBLE_EQ(loaded.sampleStep(), b->graph.sampleStep());
  const Pose X = fromXYZYaw(2.3, 3.1, 0.0, 0.4);
  const auto a = b->graph.twins(X), c = loaded.twins(X);
  ASSERT_EQ(a.size(), c.size());
  for (std::size_t k = 0; k < a.size(); ++k) {
    EXPECT_LT((a[k].pose.matrix() - c[k].pose.matrix()).norm(), 1e-9);
  }
  SymmetryGraph none;
  EXPECT_FALSE(none.load("/tmp/does_not_exist.bin"));
  EXPECT_EQ(none.nearest(Eigen::Vector3d::Zero()), nullptr);
}

TEST(SymmetryGraph, TheFingerprintTellsAnotherMapOrOptionsApart)
{
  auto b = build(test_maps::twinRooms());
  const std::string path = "/tmp/symmetry_graph_fingerprint_test.bin";
  ASSERT_TRUE(b->graph.save(path, 12345u));
  SymmetryGraph same, other, any;
  EXPECT_TRUE(same.load(path, 12345u));
  EXPECT_EQ(same.nodes().size(), b->graph.nodes().size());
  EXPECT_FALSE(other.load(path, 12346u));   // built from another map or other options
  EXPECT_TRUE(any.load(path));              // no fingerprint asked: whatever is there
  std::remove(path.c_str());
}

TEST(SymmetryGraph, NearestPlace)
{
  auto b = build(test_maps::lRoom());
  const Node * n = b->graph.nearest(Eigen::Vector3d(2.1, 2.2, 0.0));
  ASSERT_NE(n, nullptr);
  EXPECT_LT((n->position - Eigen::Vector3d(2.1, 2.2, 0.0)).norm(), 0.5);
  EXPECT_EQ(b->graph.nearest(Eigen::Vector3d(6.0, 7.0, 0.0)), nullptr);   // inside the wall block
}
