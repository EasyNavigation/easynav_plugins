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

/// \brief With a NavMap in the NavState, the global estimate is on its surface.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "easynav_common/types/NavState.hpp"
#include "easynav_fusion_localizer/FusionLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navmap_core/NavMap.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{

class FriendFusionLocalizer : public easynav::FusionLocalizer
{
public:
  using easynav::FusionLocalizer::update_rt;
};

/// A ramp z = 0.1 * x over [-5, 40] x [-10, 10].
::navmap::NavMap ramp()
{
  ::navmap::NavMap map;
  const auto s = map.create_surface("map");
  const auto a = map.add_vertex({-5.0f, -10.0f, -0.5f});
  const auto b = map.add_vertex({40.0f, -10.0f, 4.0f});
  const auto c = map.add_vertex({40.0f, 10.0f, 4.0f});
  const auto d = map.add_vertex({-5.0f, 10.0f, -0.5f});
  map.add_navcel_to_surface(s, map.add_navcel(a, b, c));
  map.add_navcel_to_surface(s, map.add_navcel(a, c, d));
  map.rebuild_geometry_accels();
  return map;
}


}  // namespace

class FusionTerrainTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}

  /// Drives forward at 1 m/s for \p seconds; returns the last global estimate.
  std::optional<nav_msgs::msg::Odometry> drive(
    bool with_map, bool with_imu, std::optional<bool> navmap_terrain, double seconds = 2.0)
  {
    std::vector<rclcpp::Parameter> params = {
      {"fusion.global_filter.frequency", 30.0},
      {"fusion.global_filter.two_d_mode", true},
      {"fusion.global_filter.publish_tf", false},
      {"fusion.global_filter.map_frame", "map"},
      {"fusion.global_filter.odom_frame", "odom"},
      {"fusion.global_filter.base_link_frame", "base_footprint"},
      {"fusion.global_filter.world_frame", "map"},
      {"fusion.global_filter.odom0", "terrain_test_odom"},
      {"fusion.global_filter.odom0_config", std::vector<bool>{
          false, false, false, false, false, false,
          true, true, false, false, false, true,
          false, false, false}},
    };
    if (navmap_terrain) {
      params.push_back({"fusion.navmap_terrain", *navmap_terrain});
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    auto node = std::make_shared<easynav::LocalizerNode>(options);
    node->declare_parameter<std::vector<std::string>>(
      "localizer_types", std::vector<std::string>{});
    auto localizer = std::make_shared<FriendFusionLocalizer>();
    localizer->initialize(node, "fusion");

    auto pub_node = std::make_shared<rclcpp::Node>("terrain_odom_pub");
    auto pub = pub_node->create_publisher<nav_msgs::msg::Odometry>("terrain_test_odom", 10);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.add_callback_group(node->get_real_time_cbg(), node->get_node_base_interface());
    exec.add_node(pub_node->get_node_base_interface());

    easynav::NavState nav_state;
    if (with_map) {
      nav_state.set("map.navmap", ramp());
    }
    if (with_imu) {
      easynav::IMUPerception imu;
      tf2::Quaternion q;
      q.setRPY(0.03, -0.1, 1.2);  // the IMU's own yaw is not used
      imu.data.orientation.x = q.x();
      imu.data.orientation.y = q.y();
      imu.data.orientation.z = q.z();
      imu.data.orientation.w = q.w();
      nav_state.set("imu", imu);
    }

    std::optional<nav_msgs::msg::Odometry> last;
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = pub_node->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = 1.0;
      for (int i = 0; i < 6; ++i) {
        odom.pose.covariance[i * 7] = 0.01;
        odom.twist.covariance[i * 7] = 0.001;
      }
      pub->publish(odom);
      exec.spin_some();
      localizer->update_rt(nav_state);
      if (nav_state.has("robot_pose")) {
        last = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return last;
  }
};

TEST_F(FusionTerrainTest, WithANavMapTheHeightIsTheSurfacesAndTheTiltTheImus)
{
  const auto pose = drive(true, true, std::nullopt);
  ASSERT_TRUE(pose);
  const auto & p = pose->pose.pose;
  EXPECT_GT(p.position.x, 0.5) << "it moved";
  EXPECT_NEAR(p.position.z, 0.1 * p.position.x, 1e-3) << "on the ramp, not at 0";
  double r, pitch, y;
  tf2::Matrix3x3(tf2::Quaternion(
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)).getRPY(r, pitch, y);
  EXPECT_NEAR(r, 0.03, 1e-4);
  EXPECT_NEAR(pitch, -0.1, 1e-4);
  EXPECT_NEAR(y, 0.0, 0.05) << "the filter's heading";
}

TEST_F(FusionTerrainTest, WithoutImuTheTiltIsTheSurfaces)
{
  const auto pose = drive(true, false, std::nullopt);
  ASSERT_TRUE(pose);
  const auto & p = pose->pose.pose;
  EXPECT_NEAR(p.position.z, 0.1 * p.position.x, 1e-3);
  double r, pitch, y;
  tf2::Matrix3x3(tf2::Quaternion(
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)).getRPY(r, pitch, y);
  EXPECT_NEAR(pitch, -std::atan(0.1), 1e-3);
  EXPECT_NEAR(r, 0.0, 1e-3);
}

TEST_F(FusionTerrainTest, WithoutANavMapNothingChanges)
{
  const auto pose = drive(false, true, std::nullopt);
  ASSERT_TRUE(pose);
  EXPECT_NEAR(pose->pose.pose.position.z, 0.0, 1e-9);
  EXPECT_NEAR(pose->pose.pose.orientation.x, 0.0, 1e-9) << "planar, as the filter gives it";
  EXPECT_NEAR(pose->pose.pose.orientation.y, 0.0, 1e-9);
}

TEST_F(FusionTerrainTest, ItCanBeTurnedOff)
{
  const auto pose = drive(true, true, false);
  ASSERT_TRUE(pose);
  EXPECT_GT(pose->pose.pose.position.x, 0.5);
  EXPECT_NEAR(pose->pose.pose.position.z, 0.0, 1e-9);
}
