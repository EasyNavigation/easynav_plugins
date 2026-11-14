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

/// \brief The Gauss-Newton matcher: convergence, outliers, the prior, unobservable directions
/// and the absolute measurements.

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "easynav_orbit_localizer/Matcher.hpp"
#include "test_maps.hpp"

using namespace easynav::orbit;  // NOLINT

class MatcherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    map_ = std::make_unique<easynav::Costmap2D>(test_maps::lRoom());
    field_ = std::make_unique<GridDistanceField2D>(*map_);
    options_.mode = Mode::Planar;
    matcher_ = std::make_unique<Matcher>(*field_, options_);
  }
  std::unique_ptr<easynav::Costmap2D> map_;
  std::unique_ptr<GridDistanceField2D> field_;
  MatchOptions options_;
  std::unique_ptr<Matcher> matcher_;
};

TEST_F(MatcherTest, ConvergesFromAnOffsetStart)
{
  const Pose truth = fromXYZYaw(2.0, 2.5, 0.0, 0.7);
  const auto pts = test_maps::scan(*field_, truth, 360, 12.0, 0.01);
  const Pose start = truth * expSE3((Vec6() << 0.3, -0.25, 0.0, 0.0, 0.0, 0.15).finished());
  const auto r = matcher_->match(pts, start);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(test_maps::planarError(r.pose, truth), 0.03);
  EXPECT_LT(test_maps::yawError(r.pose, truth), 0.01);
  EXPECT_GT(r.score, 0.95);
  EXPECT_EQ(r.n_points, pts.size());
  // Planar: z, roll and pitch untouched and (almost) certain
  EXPECT_NEAR(r.pose.translation().z(), 0.0, 1e-12);
  EXPECT_LT(r.cov(2, 2), 1e-6);
  EXPECT_GT(r.cov(0, 0), 0.0);
}

TEST_F(MatcherTest, DynamicObstaclesAreOutliersAndDoNotMoveTheSolution)
{
  const Pose truth = fromXYZYaw(2.0, 2.5, 0.0, 0.7);
  auto pts = test_maps::scan(*field_, truth, 360, 12.0, 0.01);
  // A group of people ahead, 1 to 2 m away over 90 degrees: a fifth of the returns, not on the map
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> a(-0.8, 0.8), d(1.0, 2.0);
  const std::size_t n = pts.size() / 4;
  for (std::size_t k = 0; k < n; ++k) {
    const double ang = a(rng), r = d(rng);
    pts.emplace_back(r * std::cos(ang), r * std::sin(ang), 0.0);
  }
  const Pose start = truth * expSE3((Vec6() << 0.2, 0.2, 0.0, 0.0, 0.0, -0.1).finished());
  const auto r = matcher_->match(pts, start);
  EXPECT_LT(test_maps::planarError(r.pose, truth), 0.05);
  EXPECT_LT(test_maps::yawError(r.pose, truth), 0.02);
  EXPECT_LT(r.score, 0.9);   // the outliers show in the score, not in the pose
  EXPECT_GT(r.score, 0.7);
}

TEST_F(MatcherTest, ScoreWithoutOptimizing)
{
  const Pose truth = fromXYZYaw(2.0, 2.5, 0.0, 0.7);
  const auto pts = test_maps::scan(*field_, truth);
  EXPECT_GT(matcher_->score(pts, truth), 0.95);
  EXPECT_LT(matcher_->score(pts, fromXYZYaw(2.0, 2.5, 0.0, 0.7 + 1.0)), 0.5);
  EXPECT_DOUBLE_EQ(matcher_->score({}, truth), 0.0);
}

TEST_F(MatcherTest, ACorridorLeavesItsDirectionToThePrior)
{
  const auto map = test_maps::corridor();
  GridDistanceField2D field(map);
  Matcher matcher(field, options_);
  const Pose truth = fromXYZYaw(10.0, 0.6, 0.0, 0.0);
  const auto pts = test_maps::scan(field, truth, 360, 6.0, 0.005);   // only the side walls
  // The prior says 10.3 along the corridor with 0.3 m deviation; the scan cannot argue
  const Pose start = fromXYZYaw(10.3, 0.75, 0.0, 0.05);
  Mat6 info = Mat6::Zero();
  info.diagonal() << 1.0 / 0.09, 1.0 / 0.09, 1e6, 1e6, 1e6, 1.0 / 0.01;
  const auto r = matcher.match(pts, start, &info);
  EXPECT_NEAR(r.pose.translation().x(), 10.3, 0.05);     // stays where the prior put it
  EXPECT_NEAR(r.pose.translation().y(), 0.6, 0.03);      // across: fixed by the walls
  EXPECT_LT(test_maps::yawError(r.pose, truth), 0.01);
  // The uncertainty along the corridor stays the prior's; across it collapses
  EXPECT_NEAR(std::sqrt(r.cov(0, 0)), 0.3, 0.03);
  EXPECT_LT(std::sqrt(r.cov(1, 1)), 0.02);
}

TEST_F(MatcherTest, ThePriorHoldsAnEmptyScan)
{
  const Pose start = fromXYZYaw(1.0, 1.0, 0.0, 0.3);
  Mat6 info = Mat6::Identity() * 100.0;
  const auto r = matcher_->match({}, start, &info);
  EXPECT_LT((r.pose.matrix() - start.matrix()).norm(), 1e-9);
  EXPECT_NEAR(r.cov(0, 0), 0.01, 1e-6);
  EXPECT_DOUBLE_EQ(r.score, 0.0);
}

TEST_F(MatcherTest, PositionMeasurementsPullThePose)
{
  // No points, a GPS antenna 0.5 m behind the body seen at (3, 3): body at (3.5, 3) heading 0
  PositionMeasurement gps;
  gps.p_map = Eigen::Vector3d(3.0, 3.0, 0.0);
  gps.lever = Eigen::Vector3d(-0.5, 0.0, 0.0);
  gps.sigma = Eigen::Vector3d(0.2, 0.2, 0.2);
  gps.use_z = false;
  Mat6 info = Mat6::Zero();
  info.diagonal() << 1e-3, 1e-3, 1.0, 1.0, 1.0, 1e4;   // heading known, position not
  const auto r = matcher_->match({}, fromXYZYaw(2.0, 2.0, 0.0, 0.0), &info, {gps});
  EXPECT_NEAR(r.pose.translation().x(), 3.5, 0.02);
  EXPECT_NEAR(r.pose.translation().y(), 3.0, 0.02);
  EXPECT_NEAR(std::sqrt(r.cov(0, 0)), 0.2, 0.02);
}

TEST_F(MatcherTest, DepthAndAttitudeInFreeMode)
{
  MatchOptions opts;
  opts.mode = Mode::Free;
  Matcher matcher(*field_, opts);
  DepthMeasurement depth;
  depth.z = -4.0;
  depth.sigma = 0.05;
  AttitudeMeasurement att;
  att.R_map_body = fromRPY(0.1, -0.05, 1.0);
  att.sigma_roll_pitch = 0.01;
  att.use_yaw = true;
  att.sigma_yaw = 0.02;
  Mat6 info = Mat6::Identity() * 1e-2;
  const Pose start = fromXYZYaw(1.0, 1.0, -3.0, 0.5);
  const auto r = matcher.match({}, start, &info, {}, {depth}, {att});
  EXPECT_NEAR(r.pose.translation().z(), -4.0, 0.01);
  const Eigen::Vector3d rpy = rpyOf(r.pose.linear());
  EXPECT_NEAR(rpy.x(), 0.1, 0.01);
  EXPECT_NEAR(rpy.y(), -0.05, 0.01);
  EXPECT_NEAR(rpy.z(), 1.0, 0.01);
  EXPECT_NEAR(r.pose.translation().x(), 1.0, 0.02);   // nothing said about x, y
}

TEST_F(MatcherTest, AttitudeWithoutYawLeavesTheHeading)
{
  MatchOptions opts;
  opts.mode = Mode::Free;
  Matcher matcher(*field_, opts);
  AttitudeMeasurement att;
  att.R_map_body = fromRPY(0.0, 0.0, 2.0);
  att.use_yaw = false;
  Mat6 info = Mat6::Identity() * 1e-2;
  const auto r = matcher.match({}, fromXYZYaw(1.0, 1.0, 0.0, 0.5), &info, {}, {}, {att});
  EXPECT_NEAR(yawOf(r.pose), 0.5, 1e-6);
}

TEST_F(MatcherTest, ThreeDimensionalCloudInFreeMode)
{
  // A floor and a wall: a tilted start recovers z, roll, pitch, x (from the wall) and y (none)
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = -40; i <= 40; ++i) {
    for (int j = -40; j <= 40; ++j) {
      cloud->points.emplace_back(0.25f * i, 0.25f * j, 0.0f);
      if (j <= 12) {cloud->points.emplace_back(5.0f, 0.25f * i, 0.25f * (j + 20));}
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  CloudDistanceField3D field(cloud, 0.25, 3.0);
  MatchOptions opts;
  opts.mode = Mode::Free;
  opts.inlier_distance = 0.2;
  Matcher matcher(field, opts);
  const Pose truth = fromXYZYaw(1.0, 0.0, 1.0, 0.0);
  // Body points: the floor below and the wall ahead, seen from the truth
  std::vector<Eigen::Vector3d> pts;
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      pts.push_back(truth.inverse() * Eigen::Vector3d(1.0 + 0.3 * i, 0.3 * j, 0.0));
      pts.push_back(truth.inverse() * Eigen::Vector3d(5.0, 0.3 * i, 0.3 * (j + 10) * 0.5));
    }
  }
  const Pose start = truth * expSE3((Vec6() << 0.2, 0.0, 0.3, 0.05, -0.05, 0.0).finished());
  Mat6 info = Mat6::Zero();
  info.diagonal() << 1e-2, 1e2, 1e-2, 1e-2, 1e-2, 1e2;   // y and yaw held by the prior
  const auto r = matcher.match(pts, start, &info);
  EXPECT_NEAR(r.pose.translation().x(), 1.0, 0.1);    // the map's points are 0.25 m apart
  EXPECT_NEAR(r.pose.translation().z(), 1.0, 0.1);
  const Eigen::Vector3d rpy = rpyOf(r.pose.linear());
  EXPECT_NEAR(rpy.x(), 0.0, 0.02);
  EXPECT_NEAR(rpy.y(), 0.0, 0.02);
  EXPECT_GT(r.score, 0.9);
}
