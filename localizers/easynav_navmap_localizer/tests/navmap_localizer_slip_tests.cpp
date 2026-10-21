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
/// \brief Reseeding only after the robot moves, and each particle's odometry scale: a robot whose
/// wheels slip (the odometry says it moves, it does not) is still followed.

#include <cmath>
#include <memory>
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
  using AMCLLocalizer::predict;
  using AMCLLocalizer::reseed;
  using AMCLLocalizer::getEstimatedPose;
  using AMCLLocalizer::particles_;
  using AMCLLocalizer::odom_;
  using AMCLLocalizer::last_odom_;
  using AMCLLocalizer::initialized_odom_;
  using AMCLLocalizer::pending_;
};

}  // namespace

class NavmapAmclSlipTest : public ::testing::Test
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
      rclcpp::Parameter("loc.initial_pose.y", 5.0),
      rclcpp::Parameter("loc.initial_pose.std_dev_xy", 0.05),
      rclcpp::Parameter("loc.initial_pose.std_dev_yaw", 0.02),
      rclcpp::Parameter("loc.num_particles", 200),
      rclcpp::Parameter("loc.noise_translation", 0.05),
      rclcpp::Parameter("loc.min_noise_xy", 0.02),  // reseeding alone does not cover a slip
      rclcpp::Parameter("loc.recovery_alpha_slow", 0.0),
      rclcpp::Parameter("loc.recovery_alpha_fast", 0.0),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    node_ = std::make_shared<easynav::LocalizerNode>(
      rclcpp::NodeOptions().parameter_overrides(params));
    amcl_ = std::make_shared<TestAmcl>();
    amcl_->initialize(node_, "loc");
    nav_msgs::msg::OccupancyGrid grid;  // free 10 x 10 m
    grid.header.frame_id = "map";
    grid.info.resolution = 0.1;
    grid.info.width = 100;
    grid.info.height = 100;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(100 * 100, 0);
    nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid));
    amcl_->initialized_odom_ = true;
  }

  // The odometry says the robot went dx forward
  void move(double dx)
  {
    amcl_->last_odom_ = amcl_->odom_;
    amcl_->odom_ = amcl_->odom_ * tf2::Transform(tf2::Quaternion::getIdentity(),
      tf2::Vector3(dx, 0.0, 0.0));
    amcl_->predict(nav_state_);
  }

  // A sensor that sees the robot at (x, y) heading along x: weights and hits, as correct()
  // writes them (the heading too: else the filter "explains" a slip by turning the particles)
  void weigh(double x, double y)
  {
    for (auto & p : amcl_->particles_) {
      const double d = std::hypot(p.pose.getOrigin().x() - x, p.pose.getOrigin().y() - y);
      const double a = tf2::getYaw(p.pose.getRotation());
      p.weight = std::exp(-d * d / (2.0 * 0.2 * 0.2) - a * a / (2.0 * 0.1 * 0.1)) + 1e-12;
      p.hits = static_cast<float>(p.weight);
      p.possible_hits = 1.0f;
    }
  }

  double x_estimate() const {return amcl_->getEstimatedPose().getOrigin().x();}

  std::shared_ptr<easynav::LocalizerNode> node_;
  std::shared_ptr<TestAmcl> amcl_;
  easynav::NavState nav_state_;
};

TEST_F(NavmapAmclSlipTest, StandingStillThereIsNoReseed)
{
  make();
  weigh(2.0, 5.0);
  for (int i = 0; i < 5; ++i) {
    amcl_->reseed();
    EXPECT_FALSE(amcl_->pending_.load()) << "still, reseed " << i;
    move(0.0);
  }
  // A little, less than reseed_min_d (0.1 m): still no reseed
  move(0.05);
  amcl_->reseed();
  EXPECT_FALSE(amcl_->pending_.load());
  // Past it: a reseed, then none again until it moves on
  move(0.06);
  amcl_->reseed();
  EXPECT_TRUE(amcl_->pending_.load());
  move(0.0);  // the RT thread takes the poses
  EXPECT_FALSE(amcl_->pending_.load());
  amcl_->reseed();
  EXPECT_FALSE(amcl_->pending_.load());
}

TEST_F(NavmapAmclSlipTest, TurningInPlaceAlsoCountsAsMoving)
{
  make();
  weigh(2.0, 5.0);
  amcl_->last_odom_ = amcl_->odom_;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.15);  // past reseed_min_a (0.1 rad)
  amcl_->odom_ = amcl_->odom_ * tf2::Transform(q, tf2::Vector3(0.0, 0.0, 0.0));
  amcl_->predict(nav_state_);
  amcl_->reseed();
  EXPECT_TRUE(amcl_->pending_.load());
}

TEST_F(NavmapAmclSlipTest, ReseededParticlesTakeTheirOdometryScaleAlong)
{
  // All of the reseeded ones get a slip scale in [0, 1]; the kept ones keep 1
  make({rclcpp::Parameter("loc.odom_slip_fraction", 1.0)});
  for (const auto & p : amcl_->particles_) {
    ASSERT_DOUBLE_EQ(p.odom_scale, 1.0);  // a new filter does not slip
  }
  move(0.2);
  weigh(2.2, 5.0);
  amcl_->reseed();
  move(0.0);  // handed over
  std::size_t slipping = 0;
  for (const auto & p : amcl_->particles_) {
    ASSERT_GE(p.odom_scale, 0.0);
    ASSERT_LE(p.odom_scale, 1.2);
    if (p.odom_scale < 0.999) {++slipping;}
  }
  EXPECT_GE(slipping, 150u);  // the 80 % reseeded (all of them slipping)
  // Each one moves with its own scale: 1 m of odometry, between 0 and 1 m
  std::vector<double> x0, k;
  for (const auto & p : amcl_->particles_) {
    x0.push_back(p.pose.getOrigin().x());
    k.push_back(p.odom_scale);
  }
  make({rclcpp::Parameter("loc.odom_slip_fraction", 1.0),
      rclcpp::Parameter("loc.noise_translation", 1e-9)});
  // (a new filter: its particles at the same places and scales)
  amcl_->particles_.resize(x0.size());
  for (std::size_t i = 0; i < x0.size(); ++i) {
    amcl_->particles_[i].pose.setOrigin(tf2::Vector3(x0[i], 5.0, 0.0));
    amcl_->particles_[i].pose.setRotation(tf2::Quaternion::getIdentity());
    amcl_->particles_[i].odom_scale = k[i];
  }
  move(1.0);
  for (std::size_t i = 0; i < x0.size(); ++i) {
    EXPECT_NEAR(amcl_->particles_[i].pose.getOrigin().x() - x0[i], k[i], 0.01) << i;
  }
}

TEST_F(NavmapAmclSlipTest, ARobotWhoseWheelsSlipIsStillFollowed)
{
  // It drives 2 m, then gets stuck: the odometry says it goes 3 m more, it does not move. With
  // the slip hypothesis the estimate stays with it; without it, it goes on with the odometry.
  for (double slip : {0.0, 0.1}) {
    // (without the slip hypothesis the scale stays fixed: no noise either)
    make({rclcpp::Parameter("loc.odom_slip_fraction", slip),
        rclcpp::Parameter("loc.odom_scale_noise", slip > 0.0 ? 0.02 : 0.0)});
    double x = 2.0;
    for (int i = 0; i < 20; ++i) {  // 0.1 m a cycle, really moving
      move(0.1);
      x += 0.1;
      weigh(x, 5.0);
      amcl_->reseed();
      move(0.0);
    }
    EXPECT_NEAR(x_estimate(), x, 0.15) << slip;
    for (int i = 0; i < 30; ++i) {  // stuck: the wheels turn, the robot does not move
      move(0.1);
      weigh(x, 5.0);
      amcl_->reseed();
      move(0.0);
    }
    if (slip > 0.0) {
      EXPECT_NEAR(x_estimate(), x, 0.4) << "followed";
    } else {
      EXPECT_GT(x_estimate() - x, 1.0) << "lost: it went on with the odometry";
    }
  }
}
