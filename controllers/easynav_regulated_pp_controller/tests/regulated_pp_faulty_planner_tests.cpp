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
/// \brief The robot never receives a NaN when the planner produces NaN paths (FaultyPlanner),
/// through this controller and ControllerNode's velocity output.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_planner/fault_injection/FaultyPlanner.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class RegulatedPpFaultyPlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void start(const std::string & fault, int fault_after)
  {
    controller_node_ = std::make_shared<easynav::ControllerNode>(
      rclcpp::NodeOptions().parameter_overrides(
    {
      {"controller_types", std::vector<std::string>{"rpp"}},
      {"rpp.plugin", "easynav_regulated_pp_controller/RegulatedPurePursuitController"},
      {"rpp.rt_freq", 200.0},
      {"robot_limits.max_linear_vel", 0.5},
      {"robot_limits.max_linear_acc", 10.0},
      {"use_cmd_vel_stamped", true},
      {"cmd_timeout", 0.3}}));
    ASSERT_EQ(
      controller_node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      controller_node_->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
      State::PRIMARY_STATE_ACTIVE);

    planner_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "faulty_planner_node", rclcpp::NodeOptions().parameter_overrides(
        {{"plan.fault", fault}, {"plan.fault_after", fault_after}}));
    planner_ = std::make_shared<easynav::FaultyPlanner>();
    planner_->initialize(planner_node_, "plan");

    // Robot at the origin, goal 3 m ahead.
    nav_state_->set("robot_pose", nav_msgs::msg::Odometry());
    nav_msgs::msg::Goals goals;
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 3.0;
    goal.pose.orientation.w = 1.0;
    goals.goals.push_back(goal);
    nav_state_->set("goals", goals);

    listener_ = rclcpp::Node::make_shared("rpp_faulty_planner_listener");
    sub_ = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", 1000,
      [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {received_.push_back(msg->twist);});
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(listener_);
    const auto begin = std::chrono::steady_clock::now();
    while (sub_->get_publisher_count() == 0 && std::chrono::steady_clock::now() - begin < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub_->get_publisher_count(), 0u);
  }

  void TearDown() override
  {
    exe_.reset();
    sub_.reset();
    listener_.reset();
    planner_.reset();
    planner_node_.reset();
    controller_node_.reset();
  }

  // A planning cycle every 10 control cycles (200 Hz), for \p duration.
  void run_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    for (int i = 0; std::chrono::steady_clock::now() < end; ++i) {
      if (i % 10 == 0) {
        planner_->update(*nav_state_);
      }
      controller_node_->cycle_rt(nav_state_);
      controller_node_->publish_cmd_vel_rt(nav_state_);
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
    const auto spin_end = std::chrono::steady_clock::now() + 50ms;
    while (std::chrono::steady_clock::now() < spin_end) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  void expect_all_finite() const
  {
    for (size_t i = 0; i < received_.size(); ++i) {
      EXPECT_TRUE(
        std::isfinite(received_[i].linear.x) && std::isfinite(received_[i].angular.z)) << i;
    }
  }

  easynav::ControllerNode::SharedPtr controller_node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr planner_node_;
  std::shared_ptr<easynav::FaultyPlanner> planner_;
  std::shared_ptr<easynav::NavState> nav_state_ = std::make_shared<easynav::NavState>();
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::vector<geometry_msgs::msg::Twist> received_;
};

TEST_F(RegulatedPpFaultyPlannerTest, AHealthyPathMovesTheRobot)
{
  start("none", 0);
  run_for(400ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_GT(received_.back().linear.x, 0.1) << "the setup works: the robot follows the path";
  expect_all_finite();
}

TEST_F(RegulatedPpFaultyPlannerTest, NanPathsFromTheStartNeverMoveTheRobot)
{
  start("nan", 0);
  run_for(500ms);
  expect_all_finite();
  for (const auto & twist : received_) {
    EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  }
}

TEST_F(RegulatedPpFaultyPlannerTest, NanPathsWhileMovingStopTheRobotWithoutANan)
{
  start("nan", 4);  // Healthy for the first 4 plans (~0.2 s).
  run_for(150ms);
  ASSERT_FALSE(received_.empty());
  ASSERT_GT(received_.back().linear.x, 0.0) << "moving before the fault";

  run_for(800ms);
  expect_all_finite();
  EXPECT_DOUBLE_EQ(received_.back().linear.x, 0.0) << "stopped";
  EXPECT_DOUBLE_EQ(received_.back().angular.z, 0.0);
}
