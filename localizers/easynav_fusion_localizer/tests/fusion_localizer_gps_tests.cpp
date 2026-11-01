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

/// \file
/// \brief GNSS fixes in the map: the projection (UTM grid or east-north-up) and the antenna's
/// mounting on the robot.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "GeographicLib/LocalCartesian.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_fusion_localizer/FusionLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{

class FriendFusionLocalizer : public easynav::FusionLocalizer
{
public:
  using easynav::FusionLocalizer::update_rt;
};

// Sonoma Raceway: 0.54 deg east of UTM zone 10's central meridian, so the UTM grid is turned
// 0.34 deg from east-north there
constexpr double kLat0 = 38.162767;
constexpr double kLon0 = -122.457945;
constexpr double kAlt0 = 10.3;

}  // namespace

class FusionGpsTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override
  {
    localizer_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void make(std::optional<std::string> projection)
  {
    std::vector<rclcpp::Parameter> params = {
      {"fusion.latitude_origin", kLat0},
      {"fusion.longitude_origin", kLon0},
      {"fusion.altitude_origin", kAlt0},
      {"fusion.navmap_terrain", false},
      {"fusion.global_filter.frequency", 30.0},
      {"fusion.global_filter.two_d_mode", true},
      {"fusion.global_filter.publish_tf", false},
      {"fusion.global_filter.map_frame", "map"},
      {"fusion.global_filter.odom_frame", "odom"},
      {"fusion.global_filter.base_link_frame", "base_footprint"},
      {"fusion.global_filter.world_frame", "map"},
      {"fusion.global_filter.gps0", "gps_test_fix"},
      {"fusion.global_filter.gps0_config", std::vector<bool>{
          true, true, false, false, false, false,
          false, false, false, false, false, false,
          false, false, false}},
    };
    if (projection) {
      params.push_back({"fusion.projection", *projection});
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    node_ = std::make_shared<easynav::LocalizerNode>(options);
    node_->declare_parameter<std::vector<std::string>>(
      "localizer_types", std::vector<std::string>{});
    localizer_ = std::make_shared<FriendFusionLocalizer>();
    localizer_->initialize(node_, "fusion");
  }

  /// The fix of an antenna at (x, y) east-north-up from the origin, in \p frame.
  easynav::GNSSPerception fix(double x, double y, const std::string & frame)
  {
    GeographicLib::LocalCartesian enu(kLat0, kLon0, kAlt0);
    easynav::GNSSPerception p;
    enu.Reverse(x, y, 0.0, p.data.latitude, p.data.longitude, p.data.altitude);
    p.data.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    p.data.header.frame_id = frame;
    p.data.header.stamp = node_->now();
    return p;
  }

  void set_imu_yaw(double yaw)
  {
    easynav::IMUPerception imu;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    imu.data.orientation.x = q.x();
    imu.data.orientation.y = q.y();
    imu.data.orientation.z = q.z();
    imu.data.orientation.w = q.w();
    nav_state_.set("imu", imu);
  }

  /// Feeds the same antenna position for \p seconds (one fix per cycle); the last estimate.
  std::optional<nav_msgs::msg::Odometry> feed(
    double x, double y, const std::string & frame, double seconds)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    std::optional<nav_msgs::msg::Odometry> last;
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    do {
      nav_state_.set("gps", fix(x, y, frame));
      nav_state_.set_group("gnss", {"gps"});
      exec.spin_some();
      localizer_->update_rt(nav_state_);
      if (nav_state_.has("robot_pose")) {
        last = nav_state_.get<nav_msgs::msg::Odometry>("robot_pose");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < end);
    return last;
  }

  static void mount(const std::string & frame, double x, double y, double z)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base_footprint";
    t.child_frame_id = frame;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    t.transform.rotation.w = 1.0;
    easynav::RTTFBuffer::getInstance()->setTransform(t, "fusion_gps_test", true);
  }

  easynav::LocalizerNode::SharedPtr node_;
  std::shared_ptr<FriendFusionLocalizer> localizer_;
  easynav::NavState nav_state_;
};

TEST_F(FusionGpsTest, EnuPutsTheFixWhereTheMapHasIt)
{
  make("enu");
  const auto pose = feed(250.0, -150.0, "base_footprint", 0.5);
  ASSERT_TRUE(pose);
  EXPECT_NEAR(pose->pose.pose.position.x, 250.0, 0.05);
  EXPECT_NEAR(pose->pose.pose.position.y, -150.0, 0.05);
}

TEST_F(FusionGpsTest, UtmIsTheDefaultAndItsGridIsTurnedFromEastNorth)
{
  make(std::nullopt);
  const auto pose = feed(250.0, -150.0, "base_footprint", 0.5);
  ASSERT_TRUE(pose);
  const double error = std::hypot(
    pose->pose.pose.position.x - 250.0, pose->pose.pose.position.y + 150.0);
  // 0.34 deg at 290 m from the origin: 1.7 m (and the UTM scale, 0.1 m)
  EXPECT_GT(error, 1.0) << "the default keeps the UTM grid, as before";
  EXPECT_LT(error, 2.5);
}

TEST_F(FusionGpsTest, UnknownProjectionFailsToInitialize)
{
  EXPECT_THROW(make("mercator"), std::runtime_error);
}

TEST_F(FusionGpsTest, TheFirstFixPlacesTheRobotAndNotTheAntenna)
{
  // Antenna 1.5 m ahead and 1 m above; the robot faces north
  mount("gps_ahead", 1.5, 0.0, 1.0);
  make("enu");
  set_imu_yaw(M_PI / 2.0);
  const auto first = feed(100.0, 50.0, "gps_ahead", 0.0);
  ASSERT_TRUE(first);
  EXPECT_NEAR(first->pose.pose.position.x, 100.0, 0.02);
  EXPECT_NEAR(first->pose.pose.position.y, 48.5, 0.02) << "1.5 m behind the antenna";
}

TEST_F(FusionGpsTest, LaterFixesKeepTheRobotBehindTheAntenna)
{
  // Antenna 1.5 m ahead, 0.5 m to the left; the robot faces west
  mount("gps_left", 1.5, 0.5, 1.0);
  make("enu");
  set_imu_yaw(M_PI);
  const auto pose = feed(-40.0, 20.0, "gps_left", 1.5);
  ASSERT_TRUE(pose);
  EXPECT_NEAR(pose->pose.pose.position.x, -38.5, 0.1);
  EXPECT_NEAR(pose->pose.pose.position.y, 20.5, 0.1);
}

TEST_F(FusionGpsTest, WithoutTheAntennaMountingTheRobotIsAtTheAntenna)
{
  make("enu");
  set_imu_yaw(0.3);
  const auto first = feed(10.0, -5.0, "gps_unmounted", 0.0);
  ASSERT_TRUE(first);
  EXPECT_NEAR(first->pose.pose.position.x, 10.0, 0.02);
  EXPECT_NEAR(first->pose.pose.position.y, -5.0, 0.02);
}

TEST_F(FusionGpsTest, WithoutImuTheFirstFixIsTheAntennas)
{
  // No attitude to turn the mounting with: the antenna's position, as before
  mount("gps_no_imu", 1.5, 0.0, 1.0);
  make("enu");
  const auto first = feed(30.0, 30.0, "gps_no_imu", 0.0);
  ASSERT_TRUE(first);
  EXPECT_NEAR(first->pose.pose.position.x, 30.0, 0.02);
  EXPECT_NEAR(first->pose.pose.position.y, 30.0, 0.02);
}

TEST_F(FusionGpsTest, ThePublishedFixOfTheEstimateIsInTheSameProjection)
{
  make("enu");
  sensor_msgs::msg::NavSatFix::SharedPtr published;
  auto listener = std::make_shared<rclcpp::Node>("gps_filtered_listener");
  auto sub = listener->create_subscription<sensor_msgs::msg::NavSatFix>(
    "gps/filtered", 10, [&published](sensor_msgs::msg::NavSatFix::SharedPtr m) {published = m;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(listener);
  const auto in = fix(250.0, -150.0, "base_footprint");
  const auto pose = feed(250.0, -150.0, "base_footprint", 0.5);
  ASSERT_TRUE(pose);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!published && std::chrono::steady_clock::now() < end) {
    feed(250.0, -150.0, "base_footprint", 0.0);
    exec.spin_some();
  }
  ASSERT_TRUE(published);
  EXPECT_NEAR(published->latitude, in.data.latitude, 1e-6);   // ~0.1 m
  EXPECT_NEAR(published->longitude, in.data.longitude, 1e-6);
}
