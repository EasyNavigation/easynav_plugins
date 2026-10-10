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
/// \brief Tests for the plugin interfaces DiagnosticRecoveryManager is composed of: safety
/// reflexes, evaluators and mitigations (exception safety, diagnostics, lifecycle forwarding).

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/VelocityCommand.hpp"
#include "easynav_diagnostic_recovery/SafetyReflexBase.hpp"
#include "easynav_diagnostic_recovery/RecoveryEvaluatorBase.hpp"
#include "easynav_diagnostic_recovery/RecoveryMitigationBase.hpp"

class RecoveryBasesTestCase : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};


// ─────────────────────────────────────────────────────────────────────────────
// SafetyReflexBase mocks (level 0).
// ─────────────────────────────────────────────────────────────────────────────

class TrackingReflex : public easynav_diagnostic_recovery::SafetyReflexBase
{
public:
  bool should_trigger {false};
  bool throw_on_mitigate {false};
  int check_calls {0};
  int mitigate_calls {0};

  void on_initialize() override {}

  bool check(easynav::NavState &) override
  {
    check_calls++;
    return should_trigger;
  }

  void mitigate(easynav::NavState & nav_state) override
  {
    mitigate_calls++;
    if (throw_on_mitigate) {
      throw std::runtime_error("boom in mitigate (toggled)");
    }
    geometry_msgs::msg::TwistStamped applied;
    applied.twist.linear.x = 42.0;  // sentinel value, distinguishable from a fail-safe stop
    nav_state.set("cmd_vel", applied);
  }
};

class ThrowingCheckReflex : public easynav_diagnostic_recovery::SafetyReflexBase
{
public:
  void on_initialize() override {}
  bool check(easynav::NavState &) override {throw std::runtime_error("boom in check");}
  void mitigate(easynav::NavState &) override {}
};

class ThrowingMitigateReflex : public easynav_diagnostic_recovery::SafetyReflexBase
{
public:
  void on_initialize() override {}
  bool check(easynav::NavState &) override {return true;}
  void mitigate(easynav::NavState &) override {throw std::runtime_error("boom in mitigate");}
};

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryEvaluatorBase mocks (level 1).
// ─────────────────────────────────────────────────────────────────────────────

class TrackingEvaluator : public easynav_diagnostic_recovery::RecoveryEvaluatorBase
{
public:
  int call_count {0};

  void on_initialize() override {}
  void update(easynav::NavState &) override {call_count++;}
};

class ThrowingEvaluator : public easynav_diagnostic_recovery::RecoveryEvaluatorBase
{
public:
  void on_initialize() override {}
  void update(easynav::NavState &) override {throw std::runtime_error("boom in evaluator");}
};

class PublishingEvaluator : public easynav_diagnostic_recovery::RecoveryEvaluatorBase
{
public:
  int8_t level_to_publish {diagnostic_msgs::msg::DiagnosticStatus::OK};

  void on_initialize() override {}

  void update(easynav::NavState & nav_state) override
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level_to_publish;
    status.name = get_plugin_name();
    publish_diagnostic(nav_state, status);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryMitigationBase mocks (level 1).
// ─────────────────────────────────────────────────────────────────────────────

class TrackingMitigation : public easynav_diagnostic_recovery::RecoveryMitigationBase
{
public:
  int start_calls {0};
  int cycle_calls {0};
  int stop_calls {0};
  easynav_diagnostic_recovery::RecoveryStatus status_to_return {easynav_diagnostic_recovery::
    RecoveryStatus::RUNNING};

  void on_initialize() override {}
  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus &) const override {return true;}
  bool requires_control() const override {return true;}

  void on_start(easynav::NavState &) override {start_calls++;}
  easynav_diagnostic_recovery::RecoveryStatus on_cycle(easynav::NavState &) override
  {
    cycle_calls++;
    return status_to_return;
  }
  void on_stop(easynav::NavState &) override {stop_calls++;}
};

class ThrowingCycleMitigation : public easynav_diagnostic_recovery::RecoveryMitigationBase
{
public:
  void on_initialize() override {}
  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus &) const override {return true;}
  easynav_diagnostic_recovery::RecoveryStatus on_cycle(easynav::NavState &) override
  {
    throw std::runtime_error("boom in mitigation on_cycle");
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// SafetyReflexBase: internal_check_and_mitigate (level 0).
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(RecoveryBasesTestCase, ReflexDoesNotMitigateWhenCheckReturnsFalse)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_no_trigger_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_p");
  reflex.should_trigger = false;

  easynav::NavState nav_state;
  bool result = reflex.internal_check_and_mitigate(nav_state);

  EXPECT_FALSE(result);
  EXPECT_EQ(reflex.check_calls, 1);
  EXPECT_EQ(reflex.mitigate_calls, 0);
}

TEST_F(RecoveryBasesTestCase, ReflexMitigatesWhenCheckReturnsTrue)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_trigger_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_p2");
  reflex.should_trigger = true;

  easynav::NavState nav_state;
  bool result = reflex.internal_check_and_mitigate(nav_state);

  EXPECT_TRUE(result);
  EXPECT_EQ(reflex.mitigate_calls, 1);
  EXPECT_DOUBLE_EQ(
    nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x, 42.0);
}

TEST_F(RecoveryBasesTestCase, ReflexFailsSafeWhenCheckThrows)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_check_throw_node");
  ThrowingCheckReflex reflex;
  reflex.initialize(node, "reflex_p3");

  easynav::NavState nav_state;
  bool result = false;
  EXPECT_NO_THROW(result = reflex.internal_check_and_mitigate(nav_state));

  EXPECT_TRUE(result);
  // Fail-safe: the reflex overrides the command with a zero velocity.
  const auto override_cmd =
    easynav::velocity_command::peek(nav_state, easynav::VelocitySource::OVERRIDE);
  ASSERT_TRUE(override_cmd.has_value());
  const auto & applied = *override_cmd;
  EXPECT_DOUBLE_EQ(applied.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(applied.twist.angular.z, 0.0);
}

TEST_F(RecoveryBasesTestCase, ReflexFailsSafeWhenMitigateThrows)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_mitigate_throw_node");
  ThrowingMitigateReflex reflex;
  reflex.initialize(node, "reflex_p4");

  easynav::NavState nav_state;
  bool result = false;
  EXPECT_NO_THROW(result = reflex.internal_check_and_mitigate(nav_state));

  EXPECT_TRUE(result);
  // Fail-safe: the reflex overrides the command with a zero velocity.
  const auto override_cmd =
    easynav::velocity_command::peek(nav_state, easynav::VelocitySource::OVERRIDE);
  ASSERT_TRUE(override_cmd.has_value());
  const auto & applied = *override_cmd;
  EXPECT_DOUBLE_EQ(applied.twist.linear.x, 0.0);
}

// Reflexes also report into NavState's shared "diagnostics" group, so a future level-1
// evaluator could notice one triggering repeatedly without depending on the non-RT cycle.

TEST_F(RecoveryBasesTestCase, ReflexNotTriggeredPublishesOkDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_diag_ok_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_diag_ok");
  reflex.should_trigger = false;

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  ASSERT_TRUE(nav_state.has("diagnostics.reflex_diag_ok"));
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.reflex_diag_ok").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(
    std::find(members.begin(), members.end(), "diagnostics.reflex_diag_ok"), members.end());
}

TEST_F(RecoveryBasesTestCase, ReflexTriggeredPublishesWarnDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_diag_warn_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_diag_warn");
  reflex.should_trigger = true;

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.reflex_diag_warn").level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST_F(RecoveryBasesTestCase, ReflexCheckThrowPublishesErrorDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_diag_check_throw_node");
  ThrowingCheckReflex reflex;
  reflex.initialize(node, "reflex_diag_check_throw");

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(
      "diagnostics.reflex_diag_check_throw").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(RecoveryBasesTestCase, ReflexMitigateThrowPublishesErrorDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_diag_mitigate_throw_node");
  ThrowingMitigateReflex reflex;
  reflex.initialize(node, "reflex_diag_mitigate_throw");

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(
      "diagnostics.reflex_diag_mitigate_throw").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(RecoveryBasesTestCase, ReflexDiagnosticTracksLevelAcrossCycles)
{
  // Reproduces the exact bug edge-triggering-on-a-plain-bool would have: going from
  // "triggered, mitigate() succeeded" (WARN) to "triggered, mitigate() throws" (ERROR) must
  // still be reported even though "triggered" itself does not change between those two calls.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_diag_transitions_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_diag_transitions");
  easynav::NavState nav_state;
  const std::string key = "diagnostics.reflex_diag_transitions";

  reflex.should_trigger = false;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  reflex.should_trigger = true;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);

  reflex.throw_on_mitigate = true;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  reflex.throw_on_mitigate = false;
  reflex.should_trigger = false;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  // No duplicate membership, regardless of how many cycles were reported above.
  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_EQ(std::count(members.begin(), members.end(), key), 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryEvaluatorBase: internal_update and publish_diagnostic (level 1).
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(RecoveryBasesTestCase, EvaluatorInternalUpdateDoesNotRunTooSoon)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_node");
  TrackingEvaluator eval;
  eval.initialize(node, "eval_p");

  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  EXPECT_EQ(eval.call_count, 0);
}

TEST_F(RecoveryBasesTestCase, EvaluatorInternalUpdateRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval2_node");
  TrackingEvaluator eval;
  eval.initialize(node, "eval_p2");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  EXPECT_EQ(eval.call_count, 1);
}

TEST_F(RecoveryBasesTestCase, EvaluatorUpdateExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_throw_node");
  ThrowingEvaluator eval;
  eval.initialize(node, "eval_throw");

  easynav::NavState nav_state;
  EXPECT_NO_THROW(eval.internal_update(nav_state));
}

TEST_F(RecoveryBasesTestCase, PublishDiagnosticCreatesGroupAndEntry)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_pub_node");
  PublishingEvaluator eval;
  eval.initialize(node, "my_eval");
  eval.level_to_publish = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  ASSERT_TRUE(nav_state.has("diagnostics.my_eval"));
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.my_eval").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(
    std::find(members.begin(), members.end(), "diagnostics.my_eval"), members.end());
}

TEST_F(RecoveryBasesTestCase, PublishDiagnosticOverwritesInsteadOfDuplicating)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_pub2_node");
  PublishingEvaluator eval;
  eval.initialize(node, "my_eval2");

  easynav::NavState nav_state;
  eval.level_to_publish = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  eval.internal_update(nav_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.level_to_publish = diagnostic_msgs::msg::DiagnosticStatus::OK;
  eval.internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.my_eval2").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_EQ(
    std::count(members.begin(), members.end(), "diagnostics.my_eval2"), 1);
}

TEST_F(RecoveryBasesTestCase, PublishDiagnosticFromTwoEvaluatorsBothAppearInGroup)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_pub3_node");
  PublishingEvaluator eval_a;
  eval_a.initialize(node, "eval_a");
  PublishingEvaluator eval_b;
  eval_b.initialize(node, "eval_b");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval_a.internal_update(nav_state);
  eval_b.internal_update(nav_state);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(std::find(members.begin(), members.end(), "diagnostics.eval_a"), members.end());
  EXPECT_NE(std::find(members.begin(), members.end(), "diagnostics.eval_b"), members.end());
}

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryMitigationBase: internal_start/internal_cycle/internal_stop (level 1).
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(RecoveryBasesTestCase, MitigationLifecycleCallsAreForwarded)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mit_node");
  TrackingMitigation mit;
  mit.initialize(node, "mit_p");

  easynav::NavState nav_state;
  mit.internal_start(nav_state);
  auto status = mit.internal_cycle(nav_state);
  mit.internal_stop(nav_state);

  EXPECT_EQ(mit.start_calls, 1);
  EXPECT_EQ(mit.cycle_calls, 1);
  EXPECT_EQ(mit.stop_calls, 1);
  EXPECT_EQ(status, easynav_diagnostic_recovery::RecoveryStatus::RUNNING);
}

TEST_F(RecoveryBasesTestCase, MitigationCycleExceptionFailsSafe)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mit_throw_node");
  ThrowingCycleMitigation mit;
  mit.initialize(node, "mit_throw");

  easynav::NavState nav_state;
  easynav_diagnostic_recovery::RecoveryStatus status =
    easynav_diagnostic_recovery::RecoveryStatus::RUNNING;
  EXPECT_NO_THROW(status = mit.internal_cycle(nav_state));

  EXPECT_EQ(status, easynav_diagnostic_recovery::RecoveryStatus::FAILED);
  // Fail-safe: the mitigation proposes a zero velocity.
  const auto proposed =
    easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
  ASSERT_TRUE(proposed.has_value());
  const auto & applied = *proposed;
  EXPECT_DOUBLE_EQ(applied.twist.linear.x, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
