// Copyright 2026 Intelligent Robotics Lab
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

/// \file
/// \brief Recovery (augmented MCL): when the scan likelihood drops below its trend, reseed()
/// spreads the worst particles widely around the estimate.

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_ros/conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_navmap_localizer/AMCLLocalizer.hpp"

namespace
{

class TestAmcl : public easynav::navmap::AMCLLocalizer
{
public:
  using AMCLLocalizer::reseed;
  using AMCLLocalizer::apply_pending_poses_;
  using AMCLLocalizer::update_recovery_;
  using AMCLLocalizer::best_mean_;
  using AMCLLocalizer::recovery_probability_;
  using AMCLLocalizer::getEstimatedPose;
  using AMCLLocalizer::particles_;
  using AMCLLocalizer::w_slow_;
  using AMCLLocalizer::w_fast_;
};

constexpr int N = 200;
constexpr std::size_t N_TOP = 40;  // top_keep_fraction 0.2

}  // namespace

class NavmapAmclRecoveryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void make(std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> params {
      rclcpp::Parameter("loc.initial_pose.x", 2.0),
      rclcpp::Parameter("loc.initial_pose.y", 2.0),
      rclcpp::Parameter("loc.initial_pose.std_dev_xy", 0.05),
      rclcpp::Parameter("loc.initial_pose.std_dev_yaw", 0.05),
      rclcpp::Parameter("loc.num_particles", N),
      // reseed() is called here without moving the robot
      rclcpp::Parameter("loc.reseed_min_d", 0.0),
      rclcpp::Parameter("loc.reseed_min_a", 0.0),
      rclcpp::Parameter("loc.odom_slip_fraction", 0.0),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    node_ = std::make_shared<easynav::LocalizerNode>(
      rclcpp::NodeOptions().parameter_overrides(params));
    amcl_ = std::make_shared<TestAmcl>();
    amcl_->initialize(node_, "loc");
  }

  // Top particles exactly at (2, 2), the rest elsewhere, weights decreasing with the index
  void cluster()
  {
    for (std::size_t i = 0; i < amcl_->particles_.size(); ++i) {
      auto & p = amcl_->particles_[i];
      p.pose = tf2::Transform(
        tf2::Quaternion::getIdentity(),
        i < N_TOP ? tf2::Vector3(2.0, 2.0, 0.0) : tf2::Vector3(-5.0, -5.0, 0.0));
      p.weight = 1.0 - 0.001 * static_cast<double>(i);
      p.hits = 1.0f;
      p.possible_hits = 2.0f;
    }
  }

  // Weights as a sensor that sees the robot at truth: what correct() writes, and the likelihood
  // it feeds the averages with
  double weigh(const tf2::Vector3 & truth)
  {
    double sum = 0.0;
    std::vector<double> likelihood;
    for (auto & p : amcl_->particles_) {
      const double d = (p.pose.getOrigin() - truth).length();
      p.weight = std::exp(-d * d / (2.0 * 0.3 * 0.3)) + 1e-9;
      p.hits = 1.0f;
      p.possible_hits = 2.0f;
      sum += p.weight;
      likelihood.push_back(p.weight);
    }
    for (auto & p : amcl_->particles_) {
      p.weight /= sum;
    }
    return amcl_->best_mean_(likelihood);
  }

  // A filter cycle: weigh, feed the averages, reseed, the RT thread takes the poses
  void cycle(const tf2::Vector3 & truth)
  {
    amcl_->update_recovery_(weigh(truth));
    amcl_->reseed();
    amcl_->apply_pending_poses_();
  }

  double error_to(const tf2::Vector3 & truth)
  {
    return (amcl_->getEstimatedPose().getOrigin() - truth).length();
  }

  std::size_t moved_off_cluster(const std::vector<tf2::Vector3> & before) const
  {
    // A plain reseed of a point cluster puts each particle exactly min_noise_xy (0.05) away
    std::size_t n = 0;
    for (std::size_t i = 0; i < amcl_->particles_.size(); ++i) {
      if (i < before.size() && amcl_->particles_[i].pose.getOrigin() == before[i]) {continue;}
      const double d = (amcl_->particles_[i].pose.getOrigin() - tf2::Vector3(2, 2, 0)).length();
      if (std::abs(d - 0.05) > 1e-6) {++n;}
    }
    return n;
  }

  std::vector<tf2::Vector3> origins() const
  {
    std::vector<tf2::Vector3> o;
    for (const auto & p : amcl_->particles_) {o.push_back(p.pose.getOrigin());}
    return o;
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
  std::shared_ptr<TestAmcl> amcl_;
};

TEST_F(NavmapAmclRecoveryTest, AStableLikelihoodNeverSpreads)
{
  make();
  EXPECT_DOUBLE_EQ(amcl_->recovery_probability_(), 0.0);  // no data yet
  amcl_->update_recovery_(0.7);
  EXPECT_DOUBLE_EQ(amcl_->w_slow_, 0.7);  // the first one starts both
  EXPECT_DOUBLE_EQ(amcl_->w_fast_, 0.7);
  for (int i = 0; i < 200; ++i) {
    amcl_->update_recovery_(0.7);
    EXPECT_DOUBLE_EQ(amcl_->recovery_probability_(), 0.0);
  }
  // Getting better is no reason either
  for (int i = 0; i < 20; ++i) {
    amcl_->update_recovery_(0.9);
    EXPECT_DOUBLE_EQ(amcl_->recovery_probability_(), 0.0);
  }
}

TEST_F(NavmapAmclRecoveryTest, ADropSpreadsInProportionAndFadesWhenItComesBack)
{
  make();
  for (int i = 0; i < 100; ++i) {
    amcl_->update_recovery_(0.8);
                                                              }
  double last = 0.0;
  for (int i = 0; i < 10; ++i) {
    amcl_->update_recovery_(0.2);
    const double p = amcl_->recovery_probability_();
    EXPECT_GT(p, last);  // the longer it lasts, the more
    EXPECT_LE(p, 1.0);
    EXPECT_NEAR(p, 1.0 - amcl_->w_fast_ / amcl_->w_slow_, 1e-12);
    last = p;
  }
  EXPECT_GT(last, 0.4);
  for (int i = 0; i < 60; ++i) {
    amcl_->update_recovery_(0.8);
                                                             }
  EXPECT_LT(amcl_->recovery_probability_(), 0.01);
}

TEST_F(NavmapAmclRecoveryTest, BadInputsDoNotTouchTheAverages)
{
  make();
  amcl_->update_recovery_(0.5);
  amcl_->update_recovery_(std::nan(""));
  amcl_->update_recovery_(std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(amcl_->w_slow_, 0.5);
  EXPECT_DOUBLE_EQ(amcl_->w_fast_, 0.5);
}

TEST_F(NavmapAmclRecoveryTest, TheLikelihoodIsThatOfTheBestOnes)
{
  make();
  std::vector<double> none;
  EXPECT_DOUBLE_EQ(amcl_->best_mean_(none), 0.0);
  std::vector<double> one {0.3};
  EXPECT_DOUBLE_EQ(amcl_->best_mean_(one), 0.3);  // at least one
  // 20 % of 10: the two best, whatever the rest and the order
  std::vector<double> ten {0.1, 0.9, 0.0, 0.2, 0.7, 0.0, 0.1, 0.0, 0.3, 0.0};
  EXPECT_DOUBLE_EQ(amcl_->best_mean_(ten), 0.8);
  std::vector<double> worse_rest {0.1, 0.9, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(amcl_->best_mean_(worse_rest), 0.8);
}

TEST_F(NavmapAmclRecoveryTest, ZeroAlphasTurnItOff)
{
  make({rclcpp::Parameter("loc.recovery_alpha_slow", 0.0),
      rclcpp::Parameter("loc.recovery_alpha_fast", 0.0)});
  for (int i = 0; i < 100; ++i) {
    amcl_->update_recovery_(0.8);
                                                              }
  for (int i = 0; i < 20; ++i) {
    amcl_->update_recovery_(0.01);
                                                              }
  EXPECT_DOUBLE_EQ(amcl_->recovery_probability_(), 0.0);
}

TEST_F(NavmapAmclRecoveryTest, InvalidAlphasTurnItOff)
{
  for (auto [slow, fast] : std::vector<std::pair<double, double>>{
    {0.1, 0.1}, {0.2, 0.1}, {-0.1, 0.1}, {0.001, 1.5}})
  {
    make({rclcpp::Parameter("loc.recovery_alpha_slow", slow),
        rclcpp::Parameter("loc.recovery_alpha_fast", fast)});
    for (int i = 0; i < 100; ++i) {
      amcl_->update_recovery_(0.8);
    }
    for (int i = 0; i < 20; ++i) {
      amcl_->update_recovery_(0.01);
    }
    EXPECT_DOUBLE_EQ(amcl_->recovery_probability_(), 0.0) << slow << " " << fast;
  }
}

TEST_F(NavmapAmclRecoveryTest, WithoutADropReseedKeepsToTheBest)
{
  make();
  cluster();
  for (int i = 0; i < 50; ++i) {
    amcl_->update_recovery_(0.8);
                                                             }
  const auto before = origins();
  amcl_->reseed();
  amcl_->apply_pending_poses_();
  ASSERT_EQ(amcl_->particles_.size(), static_cast<std::size_t>(N));
  EXPECT_EQ(moved_off_cluster(before), 0u);
  EXPECT_DOUBLE_EQ(amcl_->w_slow_, 0.8);  // untouched
}

TEST_F(NavmapAmclRecoveryTest, ADropReplacesExactlyTheWorstOnes)
{
  make();
  cluster();
  for (int i = 0; i < 100; ++i) {
    amcl_->update_recovery_(0.8);
                                                              }
  for (int i = 0; i < 5; ++i) {
    amcl_->update_recovery_(0.2);
                                                            }
  const double p = amcl_->recovery_probability_();
  ASSERT_GT(p, 0.0);
  const auto k = static_cast<std::size_t>(std::round(p * static_cast<double>(N - N_TOP)));
  const auto before = origins();
  amcl_->reseed();
  // Not until the RT thread takes them
  EXPECT_EQ(origins(), before);
  amcl_->apply_pending_poses_();
  ASSERT_EQ(amcl_->particles_.size(), static_cast<std::size_t>(N));
  // The k lowest weights (the highest indices) are the spread ones, and only them
  std::size_t spread_worst = 0;
  for (std::size_t i = N - k; i < static_cast<std::size_t>(N); ++i) {
    const double d = (amcl_->particles_[i].pose.getOrigin() - tf2::Vector3(2, 2, 0)).length();
    if (std::abs(d - 0.05) > 1e-6) {++spread_worst;}
  }
  EXPECT_EQ(spread_worst, k);
  EXPECT_EQ(moved_off_cluster(before), k);
  // Around the estimate, about recovery_spread_xy (1 m) apart, finite
  double sum2 = 0.0;
  for (std::size_t i = N - k; i < static_cast<std::size_t>(N); ++i) {
    const auto & o = amcl_->particles_[i].pose.getOrigin();
    ASSERT_TRUE(std::isfinite(o.x()) && std::isfinite(o.y()));
    ASSERT_TRUE(std::isfinite(tf2::getYaw(amcl_->particles_[i].pose.getRotation())));
    sum2 += (o - tf2::Vector3(2, 2, 0)).length2() / 2.0;
  }
  EXPECT_NEAR(std::sqrt(sum2 / static_cast<double>(k)), 1.0, 0.3);
  // The averages go on: it keeps spreading while the likelihood stays low
  EXPECT_DOUBLE_EQ(amcl_->recovery_probability_(), p);
}

TEST_F(NavmapAmclRecoveryTest, ConvergedOneMetreOffItComesBack)
{
  // Localized at (2, 2), then the robot is really 1 m away: the cluster only crawls without
  // recovery, and jumps back with it
  for (bool recovery : {false, true}) {
    make({rclcpp::Parameter("loc.recovery_alpha_fast", recovery ? 0.1 : 0.0),
        rclcpp::Parameter("loc.recovery_alpha_slow", recovery ? 0.001 : 0.0)});
    const tf2::Vector3 start(2.0, 2.0, 0.0), truth(3.0, 2.0, 0.0);
    for (int i = 0; i < 30; ++i) {
      cycle(start);
    }
    EXPECT_LT(error_to(start), 0.1);
    // Without it each cycle moves the cluster 0.05 m at most
    for (int i = 0; i < 6; ++i) {
      cycle(truth);
    }
    if (!recovery) {
      EXPECT_GT(error_to(truth), 0.5);
      continue;
    }
    for (int i = 0; i < 9 && error_to(truth) > 0.3; ++i) {
      cycle(truth);
    }
    EXPECT_LT(error_to(truth), 0.3);
    // And it stays there, and stops spreading
    for (int i = 0; i < 40; ++i) {
      cycle(truth);
    }
    EXPECT_LT(error_to(truth), 0.15);
    EXPECT_LT(amcl_->recovery_probability_(), 0.05);
  }
}
