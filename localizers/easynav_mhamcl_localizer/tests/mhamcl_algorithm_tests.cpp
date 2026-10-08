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

#include <cmath>
#include <vector>

#include "tf2/LinearMath/Quaternion.hpp"

#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"

#include "easynav_mhamcl_localizer/MapMatcher.hpp"
#include "easynav_mhamcl_localizer/ParticlesDistribution.hpp"

using easynav::mhamcl::MapMatcher;
using easynav::mhamcl::ParticlesDistribution;
using easynav::mhamcl::ParticlesParams;
using easynav::mhamcl::ScanPoint;

namespace
{

constexpr double kRes = 0.1;

// 10 x 8 m room, walls around and an asymmetric block, so that there is only one place
// and orientation from where a scan looks like a given one.
easynav::Costmap2D make_room()
{
  easynav::Costmap2D map(100, 80, kRes, 0.0, 0.0, easynav::FREE_SPACE);

  auto fill = [&map](unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
      for (unsigned int x = x0; x <= x1; ++x) {
        for (unsigned int y = y0; y <= y1; ++y) {
          map.setCost(x, y, easynav::LETHAL_OBSTACLE);
        }
      }
    };

  fill(0, 0, 99, 0);
  fill(0, 79, 99, 79);
  fill(0, 0, 0, 79);
  fill(99, 0, 99, 79);
  fill(20, 30, 40, 33);
  fill(70, 10, 72, 40);
  fill(50, 60, 80, 62);

  return map;
}

tf2::Transform make_pose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
}

double yaw_of(const tf2::Transform & tf)
{
  return easynav::mhamcl::yaw_of(tf);
}

// What a 2D laser at the pose would see (points in the robot frame)
std::vector<tf2::Vector3> simulate_scan(
  const easynav::Costmap2D & map, const tf2::Transform & pose, int beams = 180)
{
  std::vector<tf2::Vector3> points;
  const double yaw = yaw_of(pose);

  for (int i = 0; i < beams; ++i) {
    const double angle = -M_PI + 2.0 * M_PI * i / beams;
    for (double r = 0.1; r < 15.0; r += 0.05) {
      const double wx = pose.getOrigin().x() + r * std::cos(yaw + angle);
      const double wy = pose.getOrigin().y() + r * std::sin(yaw + angle);
      unsigned int mx, my;
      if (!map.worldToMap(wx, wy, mx, my)) {break;}
      if (map.getCost(mx, my) == easynav::LETHAL_OBSTACLE) {
        points.emplace_back(r * std::cos(angle), r * std::sin(angle), 0.0);
        break;
      }
    }
  }
  return points;
}

std::vector<ScanPoint> to_scan_points(const std::vector<tf2::Vector3> & points)
{
  std::vector<ScanPoint> ret;
  for (const auto & p : points) {
    const double n = std::hypot(p.x(), p.y());
    ret.push_back({p, tf2::Vector3(p.x() / n, p.y() / n, 0.0)});
  }
  return ret;
}

}  // namespace

TEST(ParticlesDistributionTest, InitSamplesAroundPose)
{
  ParticlesParams params;
  ParticlesDistribution dist(params, 1);
  dist.init(make_pose(2.0, -1.0, 0.7), 0.1, 0.05, 0.5);

  EXPECT_EQ(
    dist.get_particles().size(),
    static_cast<std::size_t>((params.max_particles + params.min_particles) / 2));

  const auto pose = dist.get_pose();
  EXPECT_NEAR(pose.getOrigin().x(), 2.0, 0.1);
  EXPECT_NEAR(pose.getOrigin().y(), -1.0, 0.1);
  EXPECT_NEAR(yaw_of(pose), 0.7, 0.1);
  EXPECT_DOUBLE_EQ(dist.get_quality(), 0.5);

  double sum = 0.0;
  for (const auto & p : dist.get_particles()) {
    sum += p.weight;
                                                               }
  EXPECT_NEAR(sum, 1.0, 1e-9);

  const auto cov = dist.get_covariance();
  EXPECT_GT(cov[0], 0.0);
  EXPECT_GT(cov[35], 0.0);
  EXPECT_NEAR(cov[1], cov[6], 1e-12);
}

TEST(ParticlesDistributionTest, PredictAppliesOdometryIncrement)
{
  ParticlesParams params;
  params.noise_translation = 1e-9;
  params.noise_rotation = 1e-9;
  params.noise_translation_to_rotation = 1e-9;

  ParticlesDistribution dist(params, 1);
  dist.init(make_pose(1.0, 1.0, M_PI_2), 1e-9, 1e-9, 1.0);

  // Moving 1 m ahead in the robot frame, when it looks at +y
  dist.predict(make_pose(1.0, 0.0, 0.0));
  auto pose = dist.get_pose();
  EXPECT_NEAR(pose.getOrigin().x(), 1.0, 1e-4);
  EXPECT_NEAR(pose.getOrigin().y(), 2.0, 1e-4);

  dist.predict(make_pose(0.0, 0.0, 0.5));
  EXPECT_NEAR(yaw_of(dist.get_pose()), M_PI_2 + 0.5, 1e-4);
}

TEST(ParticlesDistributionTest, CorrectFavorsThePoseThatExplainsTheScan)
{
  const auto map = make_room();
  const auto truth = make_pose(3.0, 6.0, 0.4);
  const auto scan = to_scan_points(simulate_scan(map, truth));
  ASSERT_GT(scan.size(), 100u);

  ParticlesParams params;
  ParticlesDistribution good(params, 1);
  good.init(truth, 0.02, 0.01, 0.5);
  good.correct(scan, map);

  ParticlesDistribution bad(params, 1);
  bad.init(make_pose(7.0, 2.0, -2.0), 0.02, 0.01, 0.5);
  bad.correct(scan, map);

  EXPECT_GT(good.get_quality(), 0.6);
  EXPECT_LT(bad.get_quality(), 0.3);
}

TEST(ParticlesDistributionTest, ConvergesToTheTruthWithCorrectAndReseed)
{
  const auto map = make_room();
  const auto truth = make_pose(3.0, 6.0, 0.4);
  const auto scan = to_scan_points(simulate_scan(map, truth));

  ParticlesParams params;
  params.max_particles = 300;
  params.min_particles = 100;
  ParticlesDistribution dist(params, 7);
  dist.init(make_pose(3.3, 5.8, 0.55), 0.15, 0.1, 0.5);

  for (int i = 0; i < 15; ++i) {
    dist.correct(scan, map);
    dist.reseed();
  }
  dist.correct(scan, map);

  const auto pose = dist.get_pose();
  EXPECT_NEAR(pose.getOrigin().x(), 3.0, 0.15);
  EXPECT_NEAR(pose.getOrigin().y(), 6.0, 0.15);
  EXPECT_NEAR(yaw_of(pose), 0.4, 0.1);
}

TEST(ParticlesDistributionTest, ReseedKeepsParticlesInRange)
{
  const auto map = make_room();
  const auto scan = to_scan_points(simulate_scan(map, make_pose(3.0, 6.0, 0.4)));

  ParticlesParams params;
  params.max_particles = 100;
  params.min_particles = 40;
  params.particles_step = 10;

  // A bad hypothesis grows...
  ParticlesDistribution bad(params, 1);
  bad.init(make_pose(7.0, 2.0, -2.0), 0.02, 0.01, 0.5);
  bad.correct(scan, map);
  const auto initial = bad.get_particles().size();
  for (int i = 0; i < 20; ++i) {
    bad.reseed();
    EXPECT_LE(bad.get_particles().size(), static_cast<std::size_t>(params.max_particles));
    EXPECT_GE(bad.get_particles().size(), static_cast<std::size_t>(params.min_particles));
  }
  EXPECT_GT(bad.get_particles().size(), initial);

  // ...and a good one shrinks
  ParticlesDistribution good(params, 1);
  good.init(make_pose(3.0, 6.0, 0.4), 0.02, 0.01, 0.5);
  good.correct(scan, map);
  for (int i = 0; i < 20; ++i) {
    good.reseed();
    good.correct(scan, map);
    EXPECT_GE(good.get_particles().size(), static_cast<std::size_t>(params.min_particles));
  }
  EXPECT_EQ(good.get_particles().size(), static_cast<std::size_t>(params.min_particles));
}

TEST(ParticlesDistributionTest, MergeKeepsSizeAndBestParticles)
{
  ParticlesParams params;
  ParticlesDistribution a(params, 1);
  ParticlesDistribution b(params, 2);
  a.init(make_pose(0.0, 0.0, 0.0), 0.1, 0.1, 0.3);
  b.init(make_pose(0.1, 0.0, 0.0), 0.1, 0.1, 0.8);

  const auto size = a.get_particles().size();
  a.merge(b);

  EXPECT_EQ(a.get_particles().size(), size);
  EXPECT_DOUBLE_EQ(a.get_quality(), 0.8);
  double sum = 0.0;
  for (const auto & p : a.get_particles()) {
    sum += p.weight;
                                                            }
  EXPECT_NEAR(sum, 1.0, 1e-9);
}

TEST(MapMatcherTest, BuildsPyramidOfDecreasingResolution)
{
  const auto map = make_room();
  MapMatcher matcher(map, 4);

  ASSERT_EQ(matcher.num_levels(), 4);
  for (int i = 1; i < matcher.num_levels(); ++i) {
    EXPECT_NEAR(matcher.level(i).getResolution(), matcher.level(i - 1).getResolution() * 2.0, 1e-9);
    EXPECT_EQ(matcher.level(i).getSizeInCellsX(), matcher.level(i - 1).getSizeInCellsX() / 2);
  }

  // Obstacles are kept when the map is downsampled
  EXPECT_EQ(matcher.level(1).getCost(0, 5), easynav::LETHAL_OBSTACLE);
  EXPECT_EQ(matcher.level(3).getCost(0, 5), easynav::LETHAL_OBSTACLE);
}

TEST(MapMatcherTest, FindsThePoseInTheWholeMap)
{
  const auto map = make_room();
  const auto truth = make_pose(3.0, 6.0, 0.4);
  const auto scan = simulate_scan(map, truth);

  MapMatcher matcher(map, 4, M_PI / 8.0);
  const auto matches = matcher.get_matches(scan, 0.5);

  ASSERT_FALSE(matches.empty());
  for (std::size_t i = 1; i < matches.size(); ++i) {
    EXPECT_GE(matches[i - 1].weight, matches[i].weight);
  }

  // Some candidate must be close to the true pose
  bool found = false;
  for (const auto & m : matches) {
    const double dist = (m.transform.getOrigin() - truth.getOrigin()).length();
    const double dyaw = std::abs(easynav::mhamcl::normalize_angle(yaw_of(m.transform) - 0.4));
    if (dist < 0.3 && dyaw < M_PI / 8.0 + 0.01) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);

  // and the best ones are in free space
  for (const auto & m : matches) {
    unsigned int mx, my;
    ASSERT_TRUE(map.worldToMap(m.transform.getOrigin().x(), m.transform.getOrigin().y(), mx, my));
  }
}

TEST(MapMatcherTest, NoPointsNoCandidates)
{
  const auto map = make_room();
  MapMatcher matcher(map, 3);
  EXPECT_TRUE(matcher.get_matches({}, 0.5).empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
