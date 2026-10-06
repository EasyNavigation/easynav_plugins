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
/// \brief A local filter fusing only velocities (no absolute orientation) stays finite.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <thread>
#include <vector>

#include "easynav_fusion_localizer/FusionLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_common/types/NavState.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace
{

class FriendFusionLocalizer : public easynav::FusionLocalizer
{
public:
  using easynav::FusionLocalizer::update_rt;
};

bool finite(const nav_msgs::msg::Odometry & o)
{
  const auto & p = o.pose.pose;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w) &&
         std::isfinite(o.twist.twist.linear.x) && std::isfinite(o.twist.twist.angular.z);
}

}  // namespace

class FusionLocalFilterTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}

  // Runs the localizer for \p seconds while the odometry reports (vx, wz); returns the last
  // local estimate, if any.
  std::optional<nav_msgs::msg::Odometry> run(
    const std::vector<rclcpp::Parameter> & params, double vx, double wz, double seconds,
    bool * all_finite)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    auto node = std::make_shared<easynav::LocalizerNode>(options);
    node->declare_parameter<std::vector<std::string>>(
      "localizer_types", std::vector<std::string>{});
    auto localizer = std::make_shared<FriendFusionLocalizer>();
    localizer->initialize(node, "fusion");

    auto pub_node = std::make_shared<rclcpp::Node>("local_filter_odom_pub");
    auto pub = pub_node->create_publisher<nav_msgs::msg::Odometry>("test_odom", 10);
    auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>("test_imu", 10);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    // The filter's subscriptions are in the RT callback group.
    exec.add_callback_group(node->get_real_time_cbg(), node->get_node_base_interface());
    exec.add_node(pub_node->get_node_base_interface());

    easynav::NavState nav_state;
    std::optional<nav_msgs::msg::Odometry> last;
    *all_finite = true;
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = pub_node->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.angular.z = wz;
      for (int i = 0; i < 6; ++i) {
        odom.pose.covariance[i * 7] = 0.01;
        odom.twist.covariance[i * 7] = 0.001;
      }
      pub->publish(odom);
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = odom.header.stamp;
      imu.header.frame_id = "base_footprint";
      imu.orientation.w = 1.0;
      imu.angular_velocity.z = wz;
      imu.linear_acceleration.z = 9.81;
      for (int i = 0; i < 3; ++i) {
        imu.orientation_covariance[i * 4] = 0.01;
        imu.angular_velocity_covariance[i * 4] = 0.0001;
        imu.linear_acceleration_covariance[i * 4] = 0.01;
      }
      imu_pub->publish(imu);
      exec.spin_some();
      localizer->update_rt(nav_state);
      if (nav_state.has("robot_pose_local")) {
        last = nav_state.get<nav_msgs::msg::Odometry>("robot_pose_local");
        *all_finite = *all_finite && finite(*last);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return last;
  }

  static std::vector<rclcpp::Parameter> velocities_only()
  {
    return {
      {"fusion.local_filter.frequency", 30.0},
      {"fusion.local_filter.two_d_mode", true},
      {"fusion.local_filter.publish_tf", false},
      {"fusion.local_filter.map_frame", "map"},
      {"fusion.local_filter.odom_frame", "odom"},
      {"fusion.local_filter.base_link_frame", "base_footprint"},
      {"fusion.local_filter.world_frame", "odom"},
      {"fusion.local_filter.odom0", "test_odom"},
      {"fusion.local_filter.odom0_config", std::vector<bool>{
          false, false, false, false, false, false,
          true, true, false, false, false, true,
          false, false, false}},
    };
  }

  // Plus the IMU's yaw rate (as in the Summit playground).
  static std::vector<rclcpp::Parameter> with_imu_yaw_rate()
  {
    auto params = velocities_only();
    params.push_back({"fusion.local_filter.imu0", "test_imu"});
    params.push_back({"fusion.local_filter.imu0_config", std::vector<bool>{
          false, false, false, false, false, false,
          false, false, false, false, false, true,
          false, false, false}});
    params.push_back({"fusion.local_filter.imu0_remove_gravitational_acceleration", true});
    return params;
  }
};

TEST_F(FusionLocalFilterTest, WithTheImuYawRateItStaysFinite)
{
  for (const auto & [vx, wz] : std::vector<std::pair<double, double>>{{0.0, 0.0}, {0.5, 0.3}}) {
    bool all_finite = false;
    const auto last = run(with_imu_yaw_rate(), vx, wz, 2.0, &all_finite);
    ASSERT_TRUE(last.has_value());
    EXPECT_TRUE(all_finite) << vx << ", " << wz;
  }
}

TEST_F(FusionLocalFilterTest, BothFiltersStayFinite)
{
  auto params = with_imu_yaw_rate();
  for (const auto & p : std::vector<rclcpp::Parameter>{
    {"fusion.global_filter.frequency", 30.0},
    {"fusion.global_filter.two_d_mode", true},
    {"fusion.global_filter.publish_tf", false},
    {"fusion.global_filter.map_frame", "map"},
    {"fusion.global_filter.odom_frame", "odom"},
    {"fusion.global_filter.base_link_frame", "base_footprint"},
    {"fusion.global_filter.world_frame", "map"},
    {"fusion.global_filter.odom0", "test_odom"},
    {"fusion.global_filter.odom0_config", std::vector<bool>{
        false, false, false, false, false, false,
        true, true, false, false, false, true,
        false, false, false}},
    {"fusion.global_filter.imu0", "test_imu"},
    {"fusion.global_filter.imu0_config", std::vector<bool>{
        false, false, false, false, false, true,
        false, false, false, false, false, true,
        false, false, false}}})
  {
    params.push_back(p);
  }
  bool all_finite = false;
  const auto last = run(params, 0.5, 0.3, 2.0, &all_finite);
  ASSERT_TRUE(last.has_value());
  EXPECT_TRUE(all_finite);
}

// Nothing observes the yaw, so its variance grows (0.06 rad^2/s by default) past 2 rad^2,
// where the UKF's circular mean of the sigma points flips: here in ~1 s instead of ~33 s.
TEST_F(FusionLocalFilterTest, AnUnobservedYawWithAGrowingVarianceStaysFinite)
{
  auto params = with_imu_yaw_rate();
  const std::vector<double> noise{
    0.05, 0.05, 0.06, 0.03, 0.03, 3.0, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015};
  std::vector<double> process_noise(15 * 15, 0.0);
  for (int i = 0; i < 15; ++i) {
    process_noise[i * 16] = noise[i];
  }
  params.push_back({"fusion.local_filter.process_noise_covariance", process_noise});
  for (const auto & [vx, wz] : std::vector<std::pair<double, double>>{{0.0, 0.0}, {0.5, 0.3}}) {
    bool all_finite = false;
    const auto last = run(params, vx, wz, 3.0, &all_finite);
    ASSERT_TRUE(last.has_value());
    EXPECT_TRUE(all_finite) << vx << ", " << wz;
    // And the yaw did not jump by pi: it follows the turn rate.
    const double yaw = 2.0 * std::atan2(last->pose.pose.orientation.z,
      last->pose.pose.orientation.w);
    EXPECT_LT(std::abs(std::remainder(yaw - wz * 3.0, 2.0 * M_PI)), 0.6) << vx << ", " << wz;
  }
}

TEST_F(FusionLocalFilterTest, VelocitiesOnlyStayFiniteWhenStill)
{
  bool all_finite = false;
  const auto last = run(velocities_only(), 0.0, 0.0, 2.0, &all_finite);
  ASSERT_TRUE(last.has_value());
  EXPECT_TRUE(all_finite);
}

TEST_F(FusionLocalFilterTest, VelocitiesOnlyIntegrateTheMotion)
{
  bool all_finite = false;
  const auto last = run(velocities_only(), 0.5, 0.0, 2.0, &all_finite);
  ASSERT_TRUE(last.has_value());
  EXPECT_TRUE(all_finite);
  EXPECT_GT(last->pose.pose.position.x, 0.4);   // about 1 m in 2 s
  EXPECT_NEAR(last->twist.twist.linear.x, 0.5, 0.1);
}

TEST_F(FusionLocalFilterTest, VelocitiesOnlyStayFiniteWhenTurning)
{
  bool all_finite = false;
  const auto last = run(velocities_only(), 0.3, 0.5, 2.0, &all_finite);
  ASSERT_TRUE(last.has_value());
  EXPECT_TRUE(all_finite);
  EXPECT_NEAR(last->twist.twist.angular.z, 0.5, 0.15);
}

// The filters must not keep the node alive: the node owns the localizer, so a strong reference
// back would leak it, and its DDS threads would still run at exit (segfault on shutdown).
TEST_F(FusionLocalFilterTest, TheFiltersDoNotKeepTheNodeAlive)
{
  auto params = with_imu_yaw_rate();
  params.push_back({"fusion.global_filter.frequency", 30.0});
  params.push_back({"fusion.global_filter.odom0", "test_odom"});
  params.push_back({"fusion.global_filter.odom0_config", std::vector<bool>{
        false, false, false, false, false, false,
        true, true, false, false, false, true,
        false, false, false}});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  auto node = std::make_shared<easynav::LocalizerNode>(options);
  node->declare_parameter<std::vector<std::string>>(
    "localizer_types", std::vector<std::string>{});
  const auto before = node.use_count();
  auto localizer = std::make_shared<FriendFusionLocalizer>();
  localizer->initialize(node, "fusion");
  EXPECT_EQ(node.use_count(), before);

  std::weak_ptr<easynav::LocalizerNode> weak = node;
  node.reset();
  localizer.reset();
  EXPECT_TRUE(weak.expired());
}
