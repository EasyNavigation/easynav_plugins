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
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_regulated_pp_controller/RegulatedPurePursuitController.hpp"

namespace
{

// Path along +x ending at the goal (3, 0), whose heading is +y (90 deg).
nav_msgs::msg::Path path_to(double goal_x, double goal_yaw)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal_yaw);
  for (double x = 0.0; x <= goal_x + 1e-9; x += 0.1) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    path.poses.push_back(p);
  }
  return path;
}

class RppGoalApproachTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  // The playgrounds' tolerances: the controller's own equal to the system's.
  void SetUp() override
  {
    make({rclcpp::Parameter("rpp.xy_goal_tolerance", 0.3),
        rclcpp::Parameter("rpp.yaw_goal_tolerance", 0.15)});
  }

  void make(const std::vector<rclcpp::Parameter> & overrides)
  {
    static int n = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "rpp_goal_node_" + std::to_string(n++), rclcpp::NodeOptions().parameter_overrides(overrides));
    rpp_ = std::make_unique<easynav::RegulatedPurePursuitController>();
    rpp_->initialize(node_, "rpp");
    nav_state_ = std::make_unique<easynav::NavState>();
    nav_state_->set("goal_tolerance.position", 0.3);
    nav_state_->set("goal_tolerance.yaw", 0.15);
    nav_state_->set("path", path_to(3.0, M_PI_2));
  }

  // Robot \p dist_to_goal m before the goal on the path, heading +x (the goal wants +y).
  geometry_msgs::msg::Twist step(double dist_to_goal, double yaw = 0.0)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = 3.0 - dist_to_goal;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    nav_state_->set("robot_pose", odom);
    rpp_->update_rt(*nav_state_);
    return nav_state_->get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist;
  }

  static bool turning_in_place(const geometry_msgs::msg::Twist & t)
  {
    return t.linear.x == 0.0 && t.angular.z != 0.0;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<easynav::RegulatedPurePursuitController> rpp_;
  std::unique_ptr<easynav::NavState> nav_state_;
};

}  // namespace

TEST_F(RppGoalApproachTest, InsideTheToleranceButNotHalfItKeepsDriving)
{
  const auto t = step(0.25);
  EXPECT_GT(t.linear.x, 0.0);
}

TEST_F(RppGoalApproachTest, WithinHalfTheToleranceTurnsToTheGoalHeading)
{
  const auto t = step(0.1);
  EXPECT_TRUE(turning_in_place(t));
  EXPECT_GT(t.angular.z, 0.0);  // towards +y
}

TEST_F(RppGoalApproachTest, OnceTurningNoiseInsideTheToleranceDoesNotMakeItDriveAgain)
{
  EXPECT_TRUE(turning_in_place(step(0.1)));
  for (double d : {0.2, 0.28, 0.12, 0.29, 0.25}) {
    EXPECT_TRUE(turning_in_place(step(d))) << d;
  }
}

TEST_F(RppGoalApproachTest, LeavingTheWholeToleranceTracksThePathAgain)
{
  EXPECT_TRUE(turning_in_place(step(0.1)));
  const auto out = step(0.4);
  EXPECT_GT(out.linear.x, 0.0);
  EXPECT_GT(step(0.25).linear.x, 0.0);  // back inside, but not at half: still driving
  EXPECT_TRUE(turning_in_place(step(0.1)));
}

TEST_F(RppGoalApproachTest, ANewGoalStartsDrivingAgain)
{
  EXPECT_TRUE(turning_in_place(step(0.1)));
  nav_state_->set("path", path_to(3.0, -M_PI_2));  // same place, another heading
  EXPECT_TRUE(turning_in_place(step(0.1)));         // still within half: turns, now to -y
  nav_state_->set("path", path_to(3.5, M_PI_2));    // a goal 0.6 m farther
  EXPECT_GT(step(0.25 + 0.5).linear.x, 0.0);
}

TEST_F(RppGoalApproachTest, AtTheGoalWithTheRightHeadingItStops)
{
  const auto t = step(0.25, M_PI_2);  // inside the tolerance, heading within 0.15 rad
  EXPECT_EQ(t.linear.x, 0.0);
  EXPECT_EQ(t.angular.z, 0.0);
}

TEST_F(RppGoalApproachTest, AStricterOwnToleranceIsUsed)
{
  make({rclcpp::Parameter("rpp.xy_goal_tolerance", 0.1)});
  EXPECT_GT(step(0.08).linear.x, 0.0);         // within half the system's, not half its own
  EXPECT_TRUE(turning_in_place(step(0.04)));
  // The system would accept it; the controller does not stop there.
  const auto t = step(0.25, M_PI_2);
  EXPECT_FALSE(t.linear.x == 0.0 && t.angular.z == 0.0);
}

TEST_F(RppGoalApproachTest, WithoutSystemTolerancesItsOwnApply)
{
  make({rclcpp::Parameter("rpp.xy_goal_tolerance", 0.4)});
  nav_state_ = std::make_unique<easynav::NavState>();
  nav_state_->set("path", path_to(3.0, M_PI_2));
  EXPECT_GT(step(0.3).linear.x, 0.0);
  EXPECT_TRUE(turning_in_place(step(0.15)));
}
