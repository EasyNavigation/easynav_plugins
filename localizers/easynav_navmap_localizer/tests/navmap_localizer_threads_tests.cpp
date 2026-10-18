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
/// \brief The RT thread (predict(), the pose estimate) and the other one (weights, reseed())
/// share the particles without locks: each writes its own fields, and poses reach the RT thread
/// through a handover slot.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_ros/conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_navmap_localizer/AMCLLocalizer.hpp"

using namespace std::chrono_literals;

namespace
{

class TestAmcl : public easynav::navmap::AMCLLocalizer
{
public:
  using AMCLLocalizer::predict;
  using AMCLLocalizer::reseed;
  using AMCLLocalizer::particles_;
  using AMCLLocalizer::odom_;
  using AMCLLocalizer::last_odom_;
  using AMCLLocalizer::initialized_odom_;
  using AMCLLocalizer::pending_;
};

}  // namespace

class NavmapAmclThreadsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<easynav::LocalizerNode>(
      rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("loc.initial_pose.x", 2.0),
      rclcpp::Parameter("loc.initial_pose.y", 2.0),
      rclcpp::Parameter("loc.initial_pose.yaw", 0.0),
      rclcpp::Parameter("loc.initial_pose.std_dev_xy", 0.3),
      rclcpp::Parameter("loc.initial_pose.std_dev_yaw", 0.3),
      rclcpp::Parameter("loc.num_particles", 500),
      rclcpp::Parameter("loc.reseed_min_d", 0.0),  // it barely moves between reseeds
      rclcpp::Parameter("loc.reseed_min_a", 0.0),
    }));
    amcl_ = std::make_shared<TestAmcl>();
    amcl_->initialize(node_, "loc");
    // Free 4 x 4 m flat NavMap
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.1;
    grid.info.width = 40;
    grid.info.height = 40;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(40 * 40, 0);
    nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid));
    amcl_->initialized_odom_ = true;
  }

  // One RT cycle: the odometry moves a little, then the prediction
  void rt_step(int k)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.002 * ((k % 7) - 3));
    amcl_->last_odom_ = amcl_->odom_;
    amcl_->odom_ = amcl_->odom_ * tf2::Transform(q, tf2::Vector3(0.001, 0.0, 0.0));
    amcl_->predict(nav_state_);
  }

  // What correct() writes for a robot near (2, 2): weights and hits, never poses (it reads them,
  // and they may be moving)
  void weigh(std::mt19937 & rng)
  {
    std::uniform_real_distribution<double> U(0.9, 1.0);
    for (auto & p : amcl_->particles_) {
      const double d = std::hypot(p.pose.getOrigin().x() - 2.0, p.pose.getOrigin().y() - 2.0);
      p.weight = std::isfinite(d) ? std::exp(-d * d) * U(rng) + 1e-9 : 1e-9;
      p.hits = 1.0f;
      p.possible_hits = 2.0f;
    }
  }

  bool poses_finite() const
  {
    for (const auto & p : amcl_->particles_) {
      const auto & o = p.pose.getOrigin();
      if (!std::isfinite(o.x()) || !std::isfinite(o.y()) ||
        !std::isfinite(tf2::getYaw(p.pose.getRotation())))
      {
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
  std::shared_ptr<TestAmcl> amcl_;
  easynav::NavState nav_state_;
};

TEST_F(NavmapAmclThreadsTest, SortingWeightsBeingWrittenStaysInBounds)
{
  // The pose estimate sorts by weight while the other thread rewrites the weights: a sort whose
  // keys change underneath runs out of bounds (the crash this guards against)
  std::atomic<bool> stop {false};
  std::thread writer([&] {
      std::mt19937 rng(1);
      std::uniform_real_distribution<double> U(0.0, 1.0);
      while (!stop) {
        for (auto & p : amcl_->particles_) {
          p.weight = U(rng);
        }
      }
    });
  const auto end = std::chrono::steady_clock::now() + 1500ms;
  int n = 0;
  while (std::chrono::steady_clock::now() < end) {
    const auto pose = amcl_->getEstimatedPose();
    ASSERT_TRUE(std::isfinite(pose.getOrigin().x()));
    ++n;
  }
  stop = true;
  writer.join();
  EXPECT_GT(n, 100);
}

TEST_F(NavmapAmclThreadsTest, BothCyclesAtOnceKeepEveryPoseValid)
{
  std::atomic<bool> stop {false};
  std::atomic<int> reseeds {0};
  std::thread other([&] {
      std::mt19937 rng(2);
      while (!stop) {
        weigh(rng);
        amcl_->reseed();
        ++reseeds;
      }
    });
  const auto end = std::chrono::steady_clock::now() + 1500ms;
  int k = 0;
  while (std::chrono::steady_clock::now() < end) {
    rt_step(k++);
    const auto pose = amcl_->getEstimatedPose();
    ASSERT_TRUE(std::isfinite(pose.getOrigin().x()));
  }
  stop = true;
  other.join();
  EXPECT_GT(reseeds.load(), 10);
  EXPECT_TRUE(poses_finite());
  // Still around where they started: no particle took garbage
  for (const auto & p : amcl_->particles_) {
    EXPECT_LT(std::hypot(p.pose.getOrigin().x() - 2.0, p.pose.getOrigin().y() - 2.0), 3.0);
  }
}

TEST_F(NavmapAmclThreadsTest, ReseededPosesAreAppliedByTheRtThread)
{
  // Half the particles weigh nothing: reseed() moves them next to the others
  for (std::size_t i = 0; i < amcl_->particles_.size(); ++i) {
    auto & p = amcl_->particles_[i];
    p.weight = i % 2 ? 1e-9 : 1.0;
    p.hits = 1.0f;
    p.possible_hits = 2.0f;
  }
  std::vector<tf2::Transform> before;
  for (const auto & p : amcl_->particles_) {
    before.push_back(p.pose);
                                                                     }

  amcl_->reseed();
  ASSERT_TRUE(amcl_->pending_.load());
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_EQ(amcl_->particles_[i].pose.getOrigin(), before[i].getOrigin())
      << "poses are the RT thread's: reseed() leaves them";
  }
  // A second reseed while the slot is busy changes nothing
  amcl_->reseed();

  rt_step(0);
  EXPECT_FALSE(amcl_->pending_.load());
  std::size_t moved = 0;
  for (std::size_t i = 0; i < before.size(); ++i) {
    moved += (amcl_->particles_[i].pose.getOrigin() - before[i].getOrigin()).length() > 0.01;
  }
  EXPECT_GE(moved, before.size() / 3);
  EXPECT_TRUE(poses_finite());
}
