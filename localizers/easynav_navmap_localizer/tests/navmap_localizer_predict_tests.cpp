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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
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
      rclcpp::NodeOptions().parameter_overrides(
    {
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

  // Spread of the particles' yaw (circular standard deviation)
  double yaw_spread() const
  {
    double s = 0.0, c = 0.0;
    for (const auto & p : amcl_->particles_) {
      const double yaw = tf2::getYaw(p.pose.getRotation());
      s += std::sin(yaw);
      c += std::cos(yaw);
    }
    const double n = static_cast<double>(amcl_->particles_.size());
    const double r = std::hypot(s, c) / n;
    return std::sqrt(std::max(0.0, -2.0 * std::log(std::max(r, 1e-12))));
  }

  // An IMU reading with this orientation
  void imu(double roll, double pitch, double yaw)
  {
    easynav::IMUPerception p;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    p.data.orientation.x = q.x();
    p.data.orientation.y = q.y();
    p.data.orientation.z = q.z();
    p.data.orientation.w = q.w();
    p.valid = true;
    nav_state_.set("imu", p);
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

TEST_F(NavmapAmclPredictTest, EachParticleGetsItsOwnHeadingNoise)
{
  // Odometry turning: the particles must spread in yaw, so the correction can choose among them
  node_->set_parameter(rclcpp::Parameter("loc.noise_rotation", 0.3));
  amcl_->initialize(node_, "loc");
  const double before = yaw_spread();
  for (int i = 0; i < 20; ++i) {
    predict(0.0, 0.0, 0.1);
  }
  EXPECT_GT(yaw_spread(), 2.0 * before + 0.01);
}

TEST_F(NavmapAmclPredictTest, TheImuGivesRollPitchAndTurnsButNotItsAbsoluteYaw)
{
  // An IMU whose yaw is 1.2 rad off the map (no magnetometer) and tilted on a slope
  imu(0.1, -0.2, 1.2);
  const double yaw0 = mean()[2];
  predict(0.0, 0.0, 0.0);
  EXPECT_NEAR(std::remainder(mean()[2] - yaw0, 2.0 * M_PI), 0.0, 0.05);  // not snapped to 1.2
  for (const auto & p : amcl_->particles_) {
    double r, pi, y;
    tf2::Matrix3x3(p.pose.getRotation()).getRPY(r, pi, y);
    ASSERT_NEAR(r, 0.1, 1e-6);
    ASSERT_NEAR(pi, -0.2, 1e-6);
  }
  // The IMU turns 0.5 rad while the (slipping) odometry says 0.1: the particles follow the IMU
  for (int i = 1; i <= 5; ++i) {
    imu(0.1, -0.2, 1.2 + 0.1 * i);
    predict(0.0, 0.0, 0.02);
  }
  EXPECT_NEAR(std::remainder(mean()[2] - yaw0 - 0.5, 2.0 * M_PI), 0.0, 0.1);
  // Across the IMU's +-pi wrap too
  imu(0.0, 0.0, M_PI - 0.05);
  predict(0.0, 0.0, 0.0);
  const double y1 = mean()[2];
  imu(0.0, 0.0, -M_PI + 0.05);
  predict(0.0, 0.0, 0.0);
  EXPECT_NEAR(std::remainder(mean()[2] - y1 - 0.1, 2.0 * M_PI), 0.0, 0.05);
}

TEST_F(NavmapAmclPredictTest, TheEstimateCarriesTheImuRollAndPitch)
{
  // Tilted on a slope: the estimate (and so the TF) keeps the IMU's roll and pitch, with the
  // particles' yaw
  imu(0.1, -0.2, 1.2);
  predict(0.0, 0.0, 0.0);
  double r, p, y;
  tf2::Matrix3x3(amcl_->getEstimatedPose().getRotation()).getRPY(r, p, y);
  EXPECT_NEAR(r, 0.1, 1e-6);
  EXPECT_NEAR(p, -0.2, 1e-6);
  EXPECT_NEAR(std::remainder(y - mean()[2], 2.0 * M_PI), 0.0, 0.05);
  EXPECT_NEAR(amcl_->getEstimatedPose().getRotation().length(), 1.0, 1e-9);
  // The same in the pose the other components read
  const auto pose = amcl_->get_pose().pose.pose;
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  EXPECT_NEAR(r, 0.1, 1e-6);
  EXPECT_NEAR(p, -0.2, 1e-6);
  // Level again: level estimate
  imu(0.0, 0.0, 1.2);
  predict(0.0, 0.0, 0.0);
  tf2::Matrix3x3(amcl_->getEstimatedPose().getRotation()).getRPY(r, p, y);
  EXPECT_NEAR(r, 0.0, 1e-6);
  EXPECT_NEAR(p, 0.0, 1e-6);
}

TEST_F(NavmapAmclPredictTest, WithoutImuOnFlatGroundTheEstimateIsLevel)
{
  for (int i = 0; i < 5; ++i) {
    predict(0.1, 0.0, 0.05);
  }
  double r, p, y;
  tf2::Matrix3x3(amcl_->getEstimatedPose().getRotation()).getRPY(r, p, y);
  EXPECT_NEAR(r, 0.0, 1e-6);
  EXPECT_NEAR(p, 0.0, 1e-6);
  EXPECT_NEAR(std::remainder(y - mean()[2], 2.0 * M_PI), 0.0, 0.05);
}

TEST_F(NavmapAmclPredictTest, WithoutImuOnARampTheEstimateFollowsTheSurface)
{
  // A ramp rising 0.25 m per meter along x: without an IMU the attitude is the surface's
  ::navmap::NavMap nm;
  const auto surface = nm.create_surface("map");
  const int n = 9;  // 0.5 m spacing over [0, 4] m
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      nm.add_vertex({0.5f * i, 0.5f * j, 0.25f * 0.5f * i});
    }
  }
  for (int j = 0; j + 1 < n; ++j) {
    for (int i = 0; i + 1 < n; ++i) {
      const uint32_t v = j * n + i;
      nm.add_navcel_to_surface(surface, nm.add_navcel(v, v + 1, v + n + 1));
      nm.add_navcel_to_surface(surface, nm.add_navcel(v, v + n + 1, v + n));
    }
  }
  nm.rebuild_geometry_accels();
  nav_state_.set("map.navmap", nm);
  for (int i = 0; i < 3; ++i) {
    predict(0.05, 0.0, 0.0);
  }
  double r, p, y;
  tf2::Matrix3x3(amcl_->getEstimatedPose().getRotation()).getRPY(r, p, y);
  EXPECT_NEAR(std::abs(p), std::atan(0.25), 0.02);  // facing up the ramp
  EXPECT_NEAR(r, 0.0, 0.02);
  EXPECT_NEAR(amcl_->getEstimatedPose().getOrigin().z(),
    0.25 * amcl_->getEstimatedPose().getOrigin().x(), 0.02);
}

TEST_F(NavmapAmclPredictTest, MovingOnATiltedRobotStaysOnTheSurface)
{
  imu(0.0, -0.3, 0.0);  // pitched up 0.3 rad (an incline the flat map does not have)
  const auto before = mean();
  for (int i = 0; i < 5; ++i) {
    predict(0.1, 0.0, 0.0);
  }
  ASSERT_TRUE(particles_finite());
  const auto after = mean();
  // Along the tilted heading: about cos(0.3) of the distance in XY, z back on the map surface
  EXPECT_NEAR(after[0] - before[0], 0.5 * std::cos(0.3), 0.1);
  for (const auto & p : amcl_->particles_) {
    ASSERT_NEAR(p.pose.getOrigin().z(), 0.0, 1e-3);
  }
}
