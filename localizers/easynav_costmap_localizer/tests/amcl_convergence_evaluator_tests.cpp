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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_costmap_localizer/AmclConvergenceEvaluator.hpp"

class AmclConvergenceEvaluatorTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  std::shared_ptr<easynav::AmclConvergenceEvaluator> make_ready_evaluator(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto eval = std::make_shared<easynav::AmclConvergenceEvaluator>();
    eval->initialize(node, name);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    return eval;
  }
};

TEST_F(AmclConvergenceEvaluatorTestCase, OkWithoutCovarianceData)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_data_node");
  auto eval = make_ready_evaluator(node, "amcl1");

  easynav::NavState nav_state;
  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.amcl1");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(AmclConvergenceEvaluatorTestCase, OkWhenCovarianceBelowThreshold)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_below_threshold_node");
  auto eval = make_ready_evaluator(node, "amcl2");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 0.1);
  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.amcl2");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(AmclConvergenceEvaluatorTestCase, ErrorWhenCovarianceAboveThreshold)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_above_threshold_node",
    rclcpp::NodeOptions().append_parameter_override("amcl3.covariance_threshold", 0.5));
  auto eval = make_ready_evaluator(node, "amcl3");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 2.5);
  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.amcl3");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.hardware_id, "localizer.amcl");
  ASSERT_EQ(status.values.size(), 1u);
  EXPECT_EQ(status.values[0].key, "covariance_trace");
  EXPECT_NEAR(std::stod(status.values[0].value), 2.5, 1e-3);
}
