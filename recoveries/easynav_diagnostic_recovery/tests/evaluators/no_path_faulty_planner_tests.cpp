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
/// \brief NoPathEvaluator against a planner that misbehaves (FaultyPlanner).

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_diagnostic_recovery/evaluators/NoPathEvaluator.hpp"
#include "easynav_planner/fault_injection/FaultyPlanner.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;

class NoPathWithFaultyPlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void make(const std::string & fault, int fault_after)
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "no_path_faulty_planner", rclcpp::NodeOptions().parameter_overrides(
        {{"plan.fault", fault}, {"plan.fault_after", fault_after}, {"no_path.freq", 1000.0}}));
    planner_ = std::make_shared<easynav::FaultyPlanner>();
    planner_->initialize(node_, "plan");
    evaluator_ = std::make_shared<easynav::NoPathEvaluator>();
    evaluator_->initialize(node_, "no_path");

    // Robot at the origin, goal 2 m ahead.
    nav_state_.set("robot_pose", nav_msgs::msg::Odometry());
    nav_msgs::msg::Goals goals;
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 2.0;
    goal.pose.orientation.w = 1.0;
    goals.goals.push_back(goal);
    nav_state_.set("goals", goals);
  }

  // One planning cycle, then the evaluator.
  uint8_t cycle()
  {
    planner_->update(nav_state_);
    rclcpp::sleep_for(std::chrono::milliseconds(2));  // Evaluators run at most at "freq".
    evaluator_->internal_update(nav_state_);
    return nav_state_.get<DiagnosticStatus>("diagnostics.no_path").level;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::FaultyPlanner> planner_;
  std::shared_ptr<easynav::NoPathEvaluator> evaluator_;
  easynav::NavState nav_state_;
};

TEST_F(NoPathWithFaultyPlannerTest, AHealthyPlannerIsOk)
{
  make("none", 0);
  EXPECT_EQ(cycle(), DiagnosticStatus::OK);
  EXPECT_EQ(cycle(), DiagnosticStatus::OK);
}

TEST_F(NoPathWithFaultyPlannerTest, APlannerThatStartsReturningEmptyPathsIsAnError)
{
  make("empty_path", 2);
  ASSERT_EQ(cycle(), DiagnosticStatus::OK);
  ASSERT_EQ(cycle(), DiagnosticStatus::OK);
  EXPECT_EQ(cycle(), DiagnosticStatus::ERROR);
  EXPECT_EQ(
    nav_state_.get<DiagnosticStatus>("diagnostics.no_path").hardware_id, "planner")
    << "what the recovery mitigations handle";
}

TEST_F(NoPathWithFaultyPlannerTest, AnEmptyPathFromTheStartIsAnError)
{
  make("empty_path", 0);
  EXPECT_EQ(cycle(), DiagnosticStatus::ERROR);
}
