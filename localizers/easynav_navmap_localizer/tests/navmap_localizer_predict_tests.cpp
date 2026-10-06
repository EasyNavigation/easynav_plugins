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
/// \brief The motion model of the NavMap AMCL with odometry deltas that are zero in some or all
/// components (robot still, straight motion, turning in place): std::normal_distribution is
/// undefined for a zero deviation (an assertion aborts in builds with glibcxx assertions).

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
  using AMCLLocalizer::particles_;
  using AMCLLocalizer::odom_;
  using AMCLLocalizer::last_odom_;
  using AMCLLocalizer::initialized_odom_;
};

}  // namespace

class NavmapAmclPredictTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<easynav::LocalizerNode>(
      rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("loc.initial_pose.x", 2.0),
      rclcpp::Parameter("loc.initial_pose.y", 2.0),
      rclcpp::Parameter("loc.initial_pose.yaw", 0.0),
      rclcpp::Parameter("loc.initial_pose.std_dev_xy", 0.01),
      rclcpp::Parameter("loc.initial_pose.std_dev_yaw", 0.01),
      }));
    amcl_ = std::make_shared<TestAmcl>();
    amcl_->initialize(node_, "loc");

    // Free 4 x 4 m flat NavMap.
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.1;
    grid.info.width = 40;
    grid.info.height = 40;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(40 * 40, 0);
    nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid));
  }

  // One prediction with the odometry moving by (dx, dy, dyaw) in the robot frame.
  void predict(double dx, double dy, double dyaw)
  {
    amcl_->initialized_odom_ = true;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dyaw);
    amcl_->last_odom_ = amcl_->odom_;
    amcl_->odom_ = amcl_->odom_ * tf2::Transform(q, tf2::Vector3(dx, dy, 0.0));
    amcl_->predict(nav_state_);
  }

  bool particles_finite() const
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

  // Mean x, y and yaw of the particles.
  std::vector<double> mean() const
  {
    double x = 0.0, y = 0.0, s = 0.0, c = 0.0;
    for (const auto & p : amcl_->particles_) {
      x += p.pose.getOrigin().x();
      y += p.pose.getOrigin().y();
      const double yaw = tf2::getYaw(p.pose.getRotation());
      s += std::sin(yaw);
      c += std::cos(yaw);
    }
    const double n = static_cast<double>(amcl_->particles_.size());
    return {x / n, y / n, std::atan2(s, c)};
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
  std::shared_ptr<TestAmcl> amcl_;
  easynav::NavState nav_state_;
};

TEST_F(NavmapAmclPredictTest, ARobotStandingStillDoesNotMoveTheParticles)
{
  const auto before = mean();
  for (int i = 0; i < 5; ++i) {
    ASSERT_NO_FATAL_FAILURE(predict(0.0, 0.0, 0.0));
  }
  ASSERT_TRUE(particles_finite());
  const auto after = mean();
  EXPECT_NEAR(after[0], before[0], 1e-6);
  EXPECT_NEAR(after[1], before[1], 1e-6);
  EXPECT_NEAR(after[2], before[2], 1e-6);
}

TEST_F(NavmapAmclPredictTest, StraightMotionMovesThemAlongX)
{
  const auto before = mean();
  for (int i = 0; i < 5; ++i) {
    predict(0.1, 0.0, 0.0);   // dy, dz and the rotation are exactly zero
  }
  ASSERT_TRUE(particles_finite());
  const auto after = mean();
  EXPECT_NEAR(after[0] - before[0], 0.5, 0.1);
  EXPECT_NEAR(after[1], before[1], 0.1);
}

TEST_F(NavmapAmclPredictTest, TurningInPlaceRotatesThem)
{
  const auto before = mean();
  for (int i = 0; i < 5; ++i) {
    predict(0.0, 0.0, 0.1);   // no translation at all
  }
  ASSERT_TRUE(particles_finite());
  const auto after = mean();
  EXPECT_NEAR(after[0], before[0], 0.05);
  EXPECT_NEAR(after[1], before[1], 0.05);
  EXPECT_NEAR(std::remainder(after[2] - before[2] - 0.5, 2.0 * M_PI), 0.0, 0.15);
}

TEST_F(NavmapAmclPredictTest, AMixOfStillAndMovingCyclesStaysFinite)
{
  for (int i = 0; i < 20; ++i) {
    switch (i % 4) {
      case 0: predict(0.0, 0.0, 0.0); break;
      case 1: predict(0.05, 0.0, 0.0); break;
      case 2: predict(0.0, 0.0, -0.05); break;
      default: predict(0.0, 0.02, 0.0); break;
    }
  }
  EXPECT_TRUE(particles_finite());
}
