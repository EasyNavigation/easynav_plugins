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

#include <cmath>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_maps_manager/NavMapMapsManager.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "navmap_core/NavMap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

class NavMapLocalModeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    static int count = 0;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "navmap_local_mode_test_" + std::to_string(count++));
    easynav::RTTFBuffer::getInstance()->clear();
  }

  void robot_at(double x)
  {
    auto buf = easynav::RTTFBuffer::getInstance();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = buf->get_tf_info().odom_frame;
    tf.child_frame_id = buf->get_tf_info().robot_frame;
    tf.transform.translation.x = x;
    tf.transform.rotation.w = 1.0;
    buf->setTransform(tf, "test", true);
  }

  // What the robot sees, in its own frame: flat ground 10 x 10 m around it, a 0.6 m box 2 m
  // ahead, and a far wall 20 m away (out of the window). Stamped dt s from now.
  void see(double dt)
  {
    easynav::PointPerception p;
    p.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_frame;
    p.stamp = node_->now() + rclcpp::Duration::from_seconds(dt);
    p.valid = true;
    for (float x = -5.0f; x <= 5.0f; x += 0.05f) {
      for (float y = -5.0f; y <= 5.0f; y += 0.05f) {
        const bool box = x > 1.7f && x<2.3f && y>-0.3f && y < 0.3f;
        p.data.push_back(pcl::PointXYZ(x, y, box ? 0.6f : 0.0f));
      }
    }
    for (float z = 0.0f; z < 2.0f; z += 0.1f) {p.data.push_back(pcl::PointXYZ(20.0f, 0.0f, z));}
    nav_state_.set("scan", p);
  }

  std::shared_ptr<easynav::navmap::NavMapMapsManager> make(bool enabled)
  {
    node_->declare_parameter("navmap.local.enabled", enabled);
    node_->declare_parameter("navmap.local.range", 4.0);
    node_->declare_parameter("navmap.local.freq", 0.0);            // rebuild on every update
    node_->declare_parameter("navmap.local.accumulate_time", 1e6);  // keep every scan
    auto m = std::make_shared<easynav::navmap::NavMapMapsManager>();
    m->initialize(node_, "navmap");
    return m;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

float area(const ::navmap::NavMap & nm)
{
  float a = 0.0f;
  for (const auto & c : nm.navcels) {
    a += c.area;
  }
  return a;
}

}  // namespace

TEST_F(NavMapLocalModeTest, BuildsANavMapAroundTheRobotInItsFrame)
{
  robot_at(0.0);
  auto m = make(true);
  see(0.0);
  m->update(nav_state_);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  ASSERT_FALSE(nm.navcels.empty());
  EXPECT_EQ(nm.surfaces[0].frame_id, easynav::RTTFBuffer::getInstance()->get_tf_info().robot_frame);
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.navcel_centroid(c);
    EXPECT_LE(std::abs(p.x()), 4.0f + 0.3f);  // the window, not the far wall
    EXPECT_LE(std::abs(p.y()), 4.0f + 0.3f);
  }
  EXPECT_GT(area(nm), 0.8f * 8.0f * 8.0f);
  // The ground under the robot is there, at its height; the box is not ground
  std::size_t s;
  ::navmap::NavCelId cid;
  Eigen::Vector3f bary, hit;
  ASSERT_TRUE(nm.locate_navcel({0.0f, 0.0f, 0.1f}, s, cid, bary, &hit));
  EXPECT_NEAR(hit.z(), 0.0f, 0.05f);
  const bool ground_at_box = nm.locate_navcel({2.0f, 0.0f, 0.1f}, s, cid, bary, &hit) &&
    std::abs(hit.z()) < 0.1f;
  if (ground_at_box) {
    for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
      const auto q = nm.navcel_centroid(c);
      if (std::abs(q.x() - 2.0f) < 0.5f && std::abs(q.y()) < 0.3f) {
        ADD_FAILURE() << "cell " << c << " at " << q.transpose() << " hit " << hit.transpose();
      }
    }
  }
  EXPECT_FALSE(ground_at_box);
}

TEST_F(NavMapLocalModeTest, FollowsTheRobotAndReplacesTheMapInNavState)
{
  robot_at(0.0);
  auto m = make(true);
  see(0.0);
  m->update(nav_state_);
  const auto first = nav_state_.get<::navmap::NavMap>("map.navmap").positions.x;
  ASSERT_FALSE(first.empty());
  // The robot moves 1 m forward and sees the same thing again (relative to itself)
  robot_at(1.0);
  see(0.5);
  m->update(nav_state_);
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  EXPECT_NE(nm.positions.x, first);  // a new map replaced the old one in NavState
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_LE(std::abs(nm.navcel_centroid(c).x()), 4.0f + 0.3f);  // still around the robot
  }
  // No new perception: rebuilt from the same scans, the same map
  const auto second = nm.positions.x;
  m->update(nav_state_);
  EXPECT_EQ(nav_state_.get<::navmap::NavMap>("map.navmap").positions.x, second);
}

TEST_F(NavMapLocalModeTest, DisabledByDefault)
{
  auto off = make(false);
  robot_at(0.0);
  see(0.0);
  off->update(nav_state_);
  EXPECT_TRUE(nav_state_.get<::navmap::NavMap>("map.navmap").navcels.empty());
}

TEST_F(NavMapLocalModeTest, SafeWithoutOdometry)
{
  auto on = make(true);  // no odom -> robot transform at all
  see(0.0);
  EXPECT_NO_THROW(on->update(nav_state_));
  EXPECT_TRUE(nav_state_.get<::navmap::NavMap>("map.navmap").navcels.empty());
}
