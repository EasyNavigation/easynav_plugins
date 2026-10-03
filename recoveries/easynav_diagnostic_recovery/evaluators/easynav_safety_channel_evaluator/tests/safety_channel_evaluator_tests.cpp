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
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_core/SafetyChannel.hpp"
#include "easynav_safety_channel_evaluator/SafetyChannelEvaluator.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;

class SafetyChannelEvaluatorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<easynav::SafetyChannelEvaluator> make_evaluator(double max_stop_time = 0.0)
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "safety_channel_evaluator_test", rclcpp::NodeOptions()
      .append_parameter_override("channel.max_stop_time", max_stop_time)
      .append_parameter_override("channel.freq", 1000.0));
    auto eval = std::make_shared<easynav::SafetyChannelEvaluator>();
    eval->initialize(node_, "channel");
    return eval;
  }

  // Evaluators run at most at "freq": space the calls.
  static void update(
    const std::shared_ptr<easynav::SafetyChannelEvaluator> & eval, easynav::NavState & nav_state)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    eval->internal_update(nav_state);
  }

  static DiagnosticStatus diagnostic(const easynav::NavState & nav_state)
  {
    return nav_state.get<DiagnosticStatus>("diagnostics.channel");
  }

  static void set_state(easynav::NavState & nav_state, bool stop, bool lost = false)
  {
    easynav::SafetyChannelState state;
    state.protective_stop = stop;
    state.status_lost = lost;
    nav_state.set(easynav::kSafetyStatusKey, state);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

TEST_F(SafetyChannelEvaluatorTest, OkWithoutSafetyStatus)
{
  auto eval = make_evaluator();
  easynav::NavState nav_state;
  update(eval, nav_state);

  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic(nav_state).hardware_id, "safety_channel");
  EXPECT_EQ(diagnostic(nav_state).message, "no safety status");
}

TEST_F(SafetyChannelEvaluatorTest, OkWithoutProtectiveStop)
{
  auto eval = make_evaluator();
  easynav::NavState nav_state;
  set_state(nav_state, false);
  update(eval, nav_state);

  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic(nav_state).message, "no protective stop");
}

TEST_F(SafetyChannelEvaluatorTest, ASpeedLimitIsOkAndShown)
{
  auto eval = make_evaluator();
  easynav::NavState nav_state;
  easynav::SafetyChannelState state;
  state.max_linear_vel = 0.3;
  state.max_angular_vel = 0.5;
  nav_state.set(easynav::kSafetyStatusKey, state);
  update(eval, nav_state);

  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic(nav_state).message, "no protective stop, speed limited to 0.3 m/s");
}

TEST_F(SafetyChannelEvaluatorTest, AProtectiveStopIsAWarning)
{
  auto eval = make_evaluator();
  easynav::NavState nav_state;
  set_state(nav_state, true);
  update(eval, nav_state);

  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::WARN);
  EXPECT_EQ(diagnostic(nav_state).message, "protective stop by the safety channel");
  ASSERT_EQ(diagnostic(nav_state).values.size(), 1u);
  EXPECT_EQ(diagnostic(nav_state).values[0].key, "stop_duration");
}

TEST_F(SafetyChannelEvaluatorTest, ALostStatusIsReportedAsSuch)
{
  auto eval = make_evaluator();
  easynav::NavState nav_state;
  set_state(nav_state, true, true);
  update(eval, nav_state);

  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::WARN);
  EXPECT_EQ(diagnostic(nav_state).message, "safety status lost: robot stopped");
}

TEST_F(SafetyChannelEvaluatorTest, WithoutMaxStopTimeAStopNeverEscalates)
{
  auto eval = make_evaluator(0.0);
  easynav::NavState nav_state;
  set_state(nav_state, true);
  update(eval, nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  update(eval, nav_state);

  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::WARN);
}

TEST_F(SafetyChannelEvaluatorTest, ALongStopEscalatesToError)
{
  auto eval = make_evaluator(0.05);
  easynav::NavState nav_state;
  set_state(nav_state, true);
  update(eval, nav_state);
  ASSERT_EQ(diagnostic(nav_state).level, DiagnosticStatus::WARN) << "not yet";

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  update(eval, nav_state);
  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic(nav_state).message, "protective stop by the safety channel for too long");
}

TEST_F(SafetyChannelEvaluatorTest, EachStopCountsFromItsOwnStart)
{
  auto eval = make_evaluator(0.05);
  easynav::NavState nav_state;
  set_state(nav_state, true);
  update(eval, nav_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  update(eval, nav_state);
  ASSERT_EQ(diagnostic(nav_state).level, DiagnosticStatus::WARN);

  set_state(nav_state, false);  // Released before max_stop_time...
  update(eval, nav_state);
  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::OK);

  set_state(nav_state, true);  // ...so a new stop starts its own count.
  update(eval, nav_state);
  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::WARN);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  update(eval, nav_state);
  EXPECT_EQ(diagnostic(nav_state).level, DiagnosticStatus::ERROR);
}

TEST_F(SafetyChannelEvaluatorTest, AnInvalidMaxStopTimeFailsToInitialize)
{
  for (const double value : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(make_evaluator(value), std::runtime_error) << value;
  }
}
