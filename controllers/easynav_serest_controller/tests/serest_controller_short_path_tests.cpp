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
/// \brief Serest with very short paths: a planner returns a single pose when the goal is in (or
/// next to) the robot's cell, which the controller indexed past its end.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_serest_controller/SerestController.hpp"

class SerestShortPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "serest_short_path_" + std::to_string(count++));
    serest_ = std::make_shared<easynav::SerestController>();
    serest_->initialize(node_, "serest");
    nav_state_.set("map", 0);  // only its presence is checked
  }

  void set_robot(double x, double y, double yaw)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = node_->now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    nav_state_.set("robot_pose", odom);
  }

  void set_path(const std::vector<std::vector<double>> & xy_yaw)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    for (const auto & p : xy_yaw) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p[0];
      ps.pose.position.y = p[1];
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, p[2]);
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      path.poses.push_back(ps);
    }
    nav_state_.set("path", path);
  }

  geometry_msgs::msg::Twist run(int cycles = 5)
  {
    for (int i = 0; i < cycles; ++i) {
      rclcpp::sleep_for(std::chrono::milliseconds(20));
      serest_->update_rt(nav_state_);
    }
    return nav_state_.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist;
  }

  static bool finite(const geometry_msgs::msg::Twist & t)
  {
    return std::isfinite(t.linear.x) && std::isfinite(t.angular.z);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::SerestController> serest_;
  easynav::NavState nav_state_;
};

TEST_F(SerestShortPathTest, ASinglePoseAheadDrivesTowardsIt)
{
  set_robot(0.0, 0.0, 0.0);
  set_path({{0.6, 0.0, 0.0}});
  const auto cmd = run();
  ASSERT_TRUE(finite(cmd));
  EXPECT_GE(cmd.linear.x, 0.0);
  EXPECT_NEAR(cmd.angular.z, 0.0, 0.3);
}

TEST_F(SerestShortPathTest, ASinglePoseToTheLeftTurnsLeft)
{
  set_robot(0.0, 0.0, 0.0);
  set_path({{0.0, 0.6, M_PI / 2.0}});
  const auto cmd = run();
  ASSERT_TRUE(finite(cmd));
  EXPECT_GT(cmd.angular.z, 0.0);
}

TEST_F(SerestShortPathTest, ASinglePoseOnTheRobotStaysFinite)
{
  // The user's case: the goal (0.9, 4.9, yaw 1.49) in the robot's cell.
  set_robot(0.9, 4.9, 0.0);
  set_path({{0.9, 4.9, 1.49}});
  const auto cmd = run();
  ASSERT_TRUE(finite(cmd));
  EXPECT_NEAR(cmd.linear.x, 0.0, 0.05);
}

TEST_F(SerestShortPathTest, TwoIdenticalPosesStayFinite)
{
  set_robot(0.0, 0.0, 0.0);
  set_path({{0.5, 0.5, 0.0}, {0.5, 0.5, 0.0}});
  EXPECT_TRUE(finite(run()));
}

TEST_F(SerestShortPathTest, APathShrinkingToOnePoseAsTheRobotArrives)
{
  set_robot(0.0, 0.0, 0.0);
  set_path({{0.2, 0.0, 0.0}, {0.4, 0.0, 0.0}, {0.6, 0.0, 0.0}});
  EXPECT_TRUE(finite(run(3)));
  set_robot(0.3, 0.0, 0.0);
  set_path({{0.4, 0.0, 0.0}, {0.6, 0.0, 0.0}});
  EXPECT_TRUE(finite(run(3)));
  set_robot(0.5, 0.0, 0.0);
  set_path({{0.6, 0.0, 0.0}});
  EXPECT_TRUE(finite(run(3)));
}
