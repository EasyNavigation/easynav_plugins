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
/// \brief AMCLLocalizer reseed(): while predict() runs on the real-time thread, when the
/// particles collapse, and when the weights do not tell them apart.

#include <atomic>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

namespace
{

class TestAMCLLocalizer : public easynav::AMCLLocalizer
{
public:
  using easynav::AMCLLocalizer::odom_callback;
  using easynav::AMCLLocalizer::particles_;
  using easynav::AMCLLocalizer::reseed;
};

}  // namespace

class CostmapAMCLReseedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// Every particle starts at the same pose and moves without noise, and every reseed copies one of
// them, also without noise: whatever correct()/reseed() do meanwhile, all of them must end where
// the odometry took the robot. A particle swapped by reseed() while predict() moves it gets the
// motion of a step twice, or never.
TEST_F(CostmapAMCLReseedTest, PredictAndReseedOnTwoThreadsKeepEveryParticleOnTheOdometry)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  const std::string name = "test_localizer";
  node->declare_parameter(name + ".num_particles", 200);
  node->declare_parameter(name + ".initial_pose.std_dev_xy", 1e-9);
  node->declare_parameter(name + ".initial_pose.std_dev_yaw", 1e-9);
  node->declare_parameter(name + ".noise_translation", 1e-12);
  node->declare_parameter(name + ".noise_rotation", 1e-12);
  node->declare_parameter(name + ".noise_translation_to_rotation", 1e-12);
  node->declare_parameter(name + ".min_noise_xy", 1e-12);
  node->declare_parameter(name + ".min_noise_yaw", 1e-12);
  node->declare_parameter(name + ".reseed_freq", 1e6);
  auto localizer = std::make_shared<TestAMCLLocalizer>();
  localizer->initialize(node, name);

  // A wall ahead that the points hit, so that correct() scores and reseed() runs.
  easynav::NavState corrections;
  easynav::Costmap2D map(400, 100, 0.05, -2.0, -2.5);
  for (unsigned int y = 0; y < 100; ++y) {
    for (unsigned int x = 300; x < 305; ++x) {
      map.setCost(x, y, easynav::LETHAL_OBSTACLE);
    }
  }
  corrections.set("map.base", map);
  easynav::PointPerception perception;
  perception.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
  perception.stamp = node->now();
  perception.valid = true;
  for (int i = 0; i < 40; ++i) {
    perception.data.push_back(pcl::PointXYZ(3.0f, -1.0f + 0.05f * i, 0.3f));
  }
  corrections.set("scan", perception);

  constexpr int kSteps = 4000;
  constexpr double kStep = 0.0025;  // 10 m in all
  std::atomic_bool done{false};
  std::thread corrector([&] {
      // As often as possible while leaving the predictions room (EasyNav runs it at freq).
      while (!done.load()) {
        localizer->update(corrections);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    });

  easynav::NavState predictions;
  for (int k = 0; k <= kSteps; ++k) {
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = node->now();
    odom->pose.pose.position.x = k * kStep;
    odom->pose.pose.orientation.w = 1.0;
    localizer->odom_callback(std::move(odom));
    localizer->update_rt(predictions);
  }
  done.store(true);
  corrector.join();

  for (const auto & p : localizer->particles_) {
    EXPECT_NEAR(p.pose.getOrigin().x(), kSteps * kStep, 1e-6);
    EXPECT_NEAR(p.pose.getOrigin().y(), 0.0, 1e-6);
  }
}

// The best particles at a single point (or on a line) have a singular covariance: its Cholesky
// factor gave NaN and so did every reseeded particle.
TEST_F(CostmapAMCLReseedTest, ReseedOfCollapsedParticlesKeepsThemFinite)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  const std::string name = "collapsed_localizer";
  node->declare_parameter(name + ".num_particles", 50);
  auto localizer = std::make_shared<TestAMCLLocalizer>();
  localizer->initialize(node, name);

  for (const bool on_a_line : {false, true}) {
    for (std::size_t i = 0; i < localizer->particles_.size(); ++i) {
      auto & p = localizer->particles_[i];
      p.pose.setIdentity();
      p.pose.setOrigin(tf2::Vector3(on_a_line ? 0.1 * static_cast<double>(i) : 1.0, 2.0, 0.0));
      p.hits = 3;
      p.possible_hits = 4;
      p.weight = 1.0 / static_cast<double>(localizer->particles_.size());
    }
    localizer->reseed();
    for (const auto & p : localizer->particles_) {
      EXPECT_TRUE(std::isfinite(p.pose.getOrigin().x())) << "on a line: " << on_a_line;
      EXPECT_TRUE(std::isfinite(p.pose.getOrigin().y())) << "on a line: " << on_a_line;
      EXPECT_TRUE(std::isfinite(p.pose.getRotation().w())) << "on a line: " << on_a_line;
    }
  }
}

// In open areas few points hit the map and every particle gets about the same weight. Reseeding
// then must not spread the cloud more each time: its noise came from the spread of the best half,
// which is the whole cloud's when the weights are alike, and the cloud grew without bound.
TEST_F(CostmapAMCLReseedTest, ReseedWithAlikeWeightsDoesNotSpreadTheCloud)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  const std::string name = "alike_localizer";
  node->declare_parameter(name + ".num_particles", 200);
  node->declare_parameter(name + ".min_noise_xy", 0.01);
  node->declare_parameter(name + ".min_noise_yaw", 0.01);
  auto localizer = std::make_shared<TestAMCLLocalizer>();
  localizer->initialize(node, name);

  std::mt19937 rng(3);
  std::normal_distribution<double> spread(0.0, 0.1);
  for (auto & p : localizer->particles_) {
    p.pose.setIdentity();
    p.pose.setOrigin(tf2::Vector3(spread(rng), spread(rng), 0.0));
  }
  auto std_xy = [&] {
      const auto & ps = localizer->particles_;
      double mx = 0.0, my = 0.0;
      for (const auto & p : ps) {
        mx += p.pose.getOrigin().x();
        my += p.pose.getOrigin().y();
      }
      mx /= ps.size();
      my /= ps.size();
      double var = 0.0;
      for (const auto & p : ps) {
        var += std::pow(p.pose.getOrigin().x() - mx, 2) + std::pow(p.pose.getOrigin().y() - my, 2);
      }
      return std::sqrt(var / (2.0 * ps.size()));
    };

  const double before = std_xy();
  for (int k = 0; k < 30; ++k) {
    for (auto & p : localizer->particles_) {
      p.hits = 1;
      p.possible_hits = 40;
      p.weight = 1.0 / static_cast<double>(localizer->particles_.size());
    }
    localizer->reseed();
  }
  EXPECT_LT(std_xy(), 2.0 * before) << "before " << before << " after " << std_xy();
}
