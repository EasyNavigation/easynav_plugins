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

/// \brief LocalizationQuality, and the AMCL's scores and random particles on a map.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_localizer/AMCLLocalizer.hpp"
#include "easynav_costmap_localizer/LocalizationQuality.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

namespace
{

using easynav::LocalizationQuality;

TEST(LocalizationQuality, NoDataMeansNoDrop)
{
  LocalizationQuality q;
  EXPECT_FALSE(q.has_data());
  EXPECT_DOUBLE_EQ(q.quality_ratio(), 1.0);
  EXPECT_DOUBLE_EQ(q.injection_probability(), 0.0);
}

TEST(LocalizationQuality, TheFirstScoresSetTheLevel)
{
  LocalizationQuality q(0.01, 0.2);
  q.add(0.4, 0.7);
  EXPECT_TRUE(q.has_data());
  EXPECT_DOUBLE_EQ(q.match_score(), 0.7);
  EXPECT_DOUBLE_EQ(q.slow(), 0.4);
  EXPECT_DOUBLE_EQ(q.fast(), 0.4);
  EXPECT_DOUBLE_EQ(q.quality_ratio(), 1.0);
}

TEST(LocalizationQuality, TheUnconvergedStartDoesNotSetTheLevel)
{
  LocalizationQuality q(0.001, 0.1);
  for (int i = 0; i < 5; ++i) {
    q.add(0.05, 0.1);  // the initial cloud, not yet on the map
  }
  for (int i = 0; i < 300; ++i) {
    q.add(0.5, 0.75);  // converged
  }
  // The slow average is the mean so far, not the first value: the ratio is near 1, not 10.
  EXPECT_NEAR(q.slow(), (5 * 0.05 + 300 * 0.5) / 305.0, 0.02);
  EXPECT_NEAR(q.quality_ratio(), 1.0, 0.05);
  // So a drop shows clearly.
  for (int i = 0; i < 20; ++i) {
    q.add(0.1, 0.25);
  }
  EXPECT_LT(q.quality_ratio(), 0.35);
}

TEST(LocalizationQuality, ASteadyMatchKeepsTheRatioAtOne)
{
  LocalizationQuality q(0.01, 0.2);
  for (int i = 0; i < 500; ++i) {
    q.add(0.4, 0.7);
  }
  EXPECT_NEAR(q.quality_ratio(), 1.0, 1e-9);
  EXPECT_NEAR(q.injection_probability(), 0.0, 1e-9);
  EXPECT_NEAR(q.match_score(), 0.7, 1e-9);
}

TEST(LocalizationQuality, ADropLowersTheRatioAndAskForRandomParticles)
{
  LocalizationQuality q(0.01, 0.2);
  for (int i = 0; i < 200; ++i) {
    q.add(0.4, 0.7);
  }
  for (int i = 0; i < 20; ++i) {
    q.add(0.05, 0.1);  // kidnapped
  }
  EXPECT_LT(q.quality_ratio(), 0.3);
  EXPECT_GT(q.injection_probability(), 0.7);
  EXPECT_LT(q.match_score(), 0.15);
  // If it stays there, the slow average follows: the drop is no longer news.
  for (int i = 0; i < 3000; ++i) {
    q.add(0.05, 0.1);
  }
  EXPECT_NEAR(q.quality_ratio(), 1.0, 0.01);
  EXPECT_LT(q.match_score(), 0.15) << "but the match is still low";
}

TEST(LocalizationQuality, ABetterMatchIsNoDrop)
{
  LocalizationQuality q(0.01, 0.2);
  q.add(0.2, 0.3);
  for (int i = 0; i < 20; ++i) {
    q.add(0.6, 0.9);
  }
  EXPECT_GT(q.quality_ratio(), 1.0);
  EXPECT_DOUBLE_EQ(q.injection_probability(), 0.0);
}

TEST(LocalizationQuality, ZeroAlphasDisableTheAverages)
{
  for (const auto & [slow, fast] : std::vector<std::pair<double, double>>{{0.0, 0.1},
      {0.01, 0.0}})
  {
    LocalizationQuality q(slow, fast);
    q.add(0.5, 0.8);
    q.add(0.01, 0.02);
    EXPECT_DOUBLE_EQ(q.quality_ratio(), 1.0);
    EXPECT_DOUBLE_EQ(q.injection_probability(), 0.0);
  }
}

TEST(LocalizationQuality, NonFiniteScoresAreIgnoredAndResetForgets)
{
  LocalizationQuality q;
  q.add(NAN, 0.5);
  EXPECT_FALSE(q.has_data());
  q.add(0.3, 0.6);
  q.add(0.3, INFINITY);
  EXPECT_DOUBLE_EQ(q.match_score(), 0.6);
  q.reset();
  EXPECT_FALSE(q.has_data());
  EXPECT_DOUBLE_EQ(q.quality_ratio(), 1.0);
}

class TestAMCLLocalizer : public easynav::AMCLLocalizer
{
public:
  using easynav::AMCLLocalizer::correct;
  using easynav::AMCLLocalizer::particles_;
  using easynav::AMCLLocalizer::quality_;
  using easynav::AMCLLocalizer::random_particles_to_inject;
  using easynav::AMCLLocalizer::reseed;
};

/// 4 x 4 m, free, with a wall along x = 1 m.
easynav::Costmap2D walled_map()
{
  easynav::Costmap2D map(80, 80, 0.05, -2.0, -2.0);
  for (unsigned int cy = 0; cy < 80; ++cy) {
    unsigned int cx, unused;
    map.worldToMap(1.0, 0.0, cx, unused);
    map.setCost(cx, cy, easynav::LETHAL_OBSTACLE);
  }
  return map;
}

class AMCLScoresTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<TestAMCLLocalizer> make(const std::string & name, int n = 100)
  {
    node_ = std::make_shared<easynav::LocalizerNode>();
    node_->declare_parameter(name + ".num_particles", n);
    auto localizer = std::make_shared<TestAMCLLocalizer>();
    localizer->initialize(node_, name);
    return localizer;
  }

  /// The wall seen 1 m ahead, from the robot footprint.
  void see_the_wall(easynav::NavState & nav_state)
  {
    easynav::PointPerception perception;
    perception.frame_id =
      easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    perception.stamp = node_->now();
    perception.valid = true;
    for (int i = -10; i <= 10; ++i) {
      perception.data.push_back(pcl::PointXYZ(1.0f, 0.05f * static_cast<float>(i), 0.3f));
    }
    nav_state.set("scan", perception);
  }

  static void place_all(TestAMCLLocalizer & localizer, double x)
  {
    for (auto & p : localizer.particles_) {
      p.pose.setIdentity();
      p.pose.setOrigin(tf2::Vector3(x, 0.0, 0.0));
    }
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
};

TEST_F(AMCLScoresTest, TheScoresTellARightPoseFromAWrongOne)
{
  auto localizer = make("scores_localizer");
  easynav::NavState nav_state;
  nav_state.set("map.base", walled_map());
  see_the_wall(nav_state);

  place_all(*localizer, 0.0);  // where the robot is: the wall 1 m ahead
  localizer->correct(nav_state);
  ASSERT_TRUE(localizer->quality_.has_data());
  EXPECT_GT(localizer->quality_.match_score(), 0.8);

  // All of them 0.5 m off, still a compact cloud: the scan misses the wall.
  place_all(*localizer, -0.5);
  for (int i = 0; i < 40; ++i) {
    localizer->correct(nav_state);
  }
  EXPECT_LT(localizer->quality_.match_score(), 0.1);
  EXPECT_LT(localizer->quality_.quality_ratio(), 0.5) << "a drop from the first level";
  EXPECT_GT(localizer->random_particles_to_inject(), 0u);
}

TEST_F(AMCLScoresTest, AReseedPlacesTheWorstAtRandomFreePoses)
{
  auto localizer = make("inject_localizer", 100);
  const auto map = walled_map();
  // Better half at x = 0, worse half at x = 0.5, told apart by their weights.
  for (std::size_t i = 0; i < localizer->particles_.size(); ++i) {
    auto & p = localizer->particles_[i];
    p.pose.setIdentity();
    p.pose.setOrigin(tf2::Vector3(i < 50 ? 0.0 : 0.5, 0.0, 0.0));
    p.hits = i < 50 ? 30 : 1;
    p.possible_hits = 40;
    p.weight = i < 50 ? 0.019 : 0.001;
  }
  localizer->reseed(&map, 20);

  std::size_t far = 0;
  for (const auto & p : localizer->particles_) {
    const auto o = p.pose.getOrigin();
    ASSERT_TRUE(std::isfinite(o.x()) && std::isfinite(o.y()));
    // Inside the map and never on the wall.
    EXPECT_GE(o.x(), -2.0);
    EXPECT_LT(o.x(), 2.0);
    EXPECT_GE(o.y(), -2.0);
    EXPECT_LT(o.y(), 2.0);
    unsigned int cx, cy;
    ASSERT_TRUE(map.worldToMap(o.x(), o.y(), cx, cy));
    EXPECT_LT(map.getCost(cx, cy), easynav::INSCRIBED_INFLATED_OBSTACLE);
    far += std::hypot(o.x(), o.y()) > 0.5;  // copies stay within a few cm of x = 0
  }
  EXPECT_EQ(localizer->particles_.size(), 100u) << "same population";
  EXPECT_GE(far, 15u) << "about the 20 random ones";
  EXPECT_LE(far, 22u);
  double total = 0.0;
  for (const auto & p : localizer->particles_) {
    total += p.weight;
  }
  EXPECT_NEAR(total, 1.0, 1e-9);
}

TEST_F(AMCLScoresTest, NeverMoreRandomOnesThanTheWorseHalf)
{
  auto localizer = make("cap_localizer", 40);
  const auto map = walled_map();
  for (auto & p : localizer->particles_) {
    p.pose.setIdentity();
    p.hits = 1;
    p.possible_hits = 40;
  }
  localizer->reseed(&map, 1000);
  std::size_t at_origin = 0;
  for (const auto & p : localizer->particles_) {
    at_origin += p.pose.getOrigin().length() < 0.3;
  }
  EXPECT_GE(at_origin, 20u) << "the better half is kept";
}

TEST_F(AMCLScoresTest, WithoutAMapOrADropNoneIsRandom)
{
  auto localizer = make("none_localizer", 50);
  for (auto & p : localizer->particles_) {
    p.pose.setIdentity();
    p.hits = 10;
    p.possible_hits = 40;
  }
  EXPECT_EQ(localizer->random_particles_to_inject(), 0u) << "no scores yet";
  localizer->reseed(nullptr, 10);
  for (const auto & p : localizer->particles_) {
    EXPECT_LT(p.pose.getOrigin().length(), 0.5);
  }
}

TEST_F(AMCLScoresTest, TheShareOfRandomOnesIsCapped)
{
  node_ = std::make_shared<easynav::LocalizerNode>();
  node_->declare_parameter("capped_localizer.num_particles", 100);
  node_->declare_parameter("capped_localizer.random_particles_max_fraction", 0.1);
  auto localizer = std::make_shared<TestAMCLLocalizer>();
  localizer->initialize(node_, "capped_localizer");
  localizer->quality_.add(0.5, 0.8);
  for (int i = 0; i < 50; ++i) {
    localizer->quality_.add(0.0, 0.0);
  }
  EXPECT_EQ(localizer->random_particles_to_inject(), 10u);
}

}  // namespace
