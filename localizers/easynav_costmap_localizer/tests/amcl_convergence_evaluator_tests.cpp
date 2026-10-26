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
#include <memory>
#include <string>
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

namespace
{

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> scored_node(
  const std::string & node_name, const std::string & plugin, double match_threshold)
{
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    node_name,
    rclcpp::NodeOptions()
    .append_parameter_override(plugin + ".match_threshold", match_threshold)
    .append_parameter_override(plugin + ".debounce_duration", 0.3)
    .append_parameter_override(plugin + ".freq", 50.0));
}

const diagnostic_msgs::msg::DiagnosticStatus & status_of(
  easynav::NavState & nav_state, const std::string & plugin)
{
  return nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics." + plugin);
}

void step(easynav::AmclConvergenceEvaluator & eval, easynav::NavState & nav_state)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.internal_update(nav_state);
}

}  // namespace

TEST_F(AmclConvergenceEvaluatorTestCase, ACompactCloudThatDoesNotMatchTheMapIsLost)
{
  auto node = scored_node("test_lost_node", "amcl4", 0.3);
  auto eval = make_ready_evaluator(node, "amcl4");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 0.01);  // compact
  nav_state.set("localizer.amcl.match_score", 0.05);       // but wrong
  nav_state.set("localizer.amcl.quality_ratio", 1.0);
  step(*eval, nav_state);
  EXPECT_EQ(status_of(nav_state, "amcl4").level, diagnostic_msgs::msg::DiagnosticStatus::OK)
    << "not before the debounce";
  step(*eval, nav_state);
  step(*eval, nav_state);
  step(*eval, nav_state);
  const auto status = status_of(nav_state, "amcl4");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("lost"), std::string::npos) << status.message;
  bool has_match = false;
  for (const auto & kv : status.values) {
    has_match = has_match || kv.key == "match_score";
  }
  EXPECT_TRUE(has_match);
}

TEST_F(AmclConvergenceEvaluatorTestCase, AGoodMatchClearsItAndRestartsTheDebounce)
{
  auto node = scored_node("test_clear_node", "amcl5", 0.3);
  auto eval = make_ready_evaluator(node, "amcl5");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 0.01);
  nav_state.set("localizer.amcl.match_score", 0.05);
  for (int i = 0; i < 4; ++i) {
    step(*eval, nav_state);
  }
  EXPECT_EQ(status_of(nav_state, "amcl5").level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  nav_state.set("localizer.amcl.match_score", 0.6);
  step(*eval, nav_state);
  EXPECT_EQ(status_of(nav_state, "amcl5").level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  // A short dip is not an error.
  nav_state.set("localizer.amcl.match_score", 0.1);
  step(*eval, nav_state);
  EXPECT_EQ(status_of(nav_state, "amcl5").level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  nav_state.set("localizer.amcl.match_score", 0.6);
  step(*eval, nav_state);
  step(*eval, nav_state);
  step(*eval, nav_state);
  EXPECT_EQ(status_of(nav_state, "amcl5").level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(AmclConvergenceEvaluatorTestCase, AMatchThresholdOfZeroDoesNotCheckTheMatch)
{
  auto node = scored_node("test_disabled_node", "amcl6", 0.0);
  auto eval = make_ready_evaluator(node, "amcl6");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 0.01);
  nav_state.set("localizer.amcl.match_score", 0.0);
  nav_state.set("localizer.amcl.quality_ratio", 1.0);
  for (int i = 0; i < 5; ++i) {
    step(*eval, nav_state);
  }
  EXPECT_EQ(status_of(nav_state, "amcl6").level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(AmclConvergenceEvaluatorTestCase, ASuddenDropOfTheMatchIsAKidnapping)
{
  auto node = scored_node("test_drop_node", "amcl7", 0.0);
  auto eval = make_ready_evaluator(node, "amcl7");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 0.01);
  nav_state.set("localizer.amcl.match_score", 0.5);
  nav_state.set("localizer.amcl.quality_ratio", 0.2);
  for (int i = 0; i < 4; ++i) {
    step(*eval, nav_state);
  }
  const auto status = status_of(nav_state, "amcl7");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("dropped"), std::string::npos) << status.message;

  nav_state.set("localizer.amcl.quality_ratio", 0.9);
  step(*eval, nav_state);
  EXPECT_EQ(status_of(nav_state, "amcl7").level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(AmclConvergenceEvaluatorTestCase, ASpreadCloudIsStillAnErrorAtOnce)
{
  auto node = scored_node("test_spread_node", "amcl8", 0.3);
  auto eval = make_ready_evaluator(node, "amcl8");

  easynav::NavState nav_state;
  nav_state.set("localizer.amcl.covariance_trace", 5.0);
  nav_state.set("localizer.amcl.match_score", 0.9);
  nav_state.set("localizer.amcl.quality_ratio", 1.0);
  step(*eval, nav_state);
  const auto status = status_of(nav_state, "amcl8");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "particle filter diverged");
}
