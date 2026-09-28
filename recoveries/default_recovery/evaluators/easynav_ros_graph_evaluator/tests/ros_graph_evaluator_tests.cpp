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
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_ros_graph_evaluator/RosGraphEvaluator.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using namespace std::chrono_literals;

class RosGraphEvaluatorTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    // Each test gets its own namespace so graph entities from other tests never interfere.
    ns_ = "/rg_test_" + std::to_string(counter_++);
  }

  // The evaluator runs on "recovery_node"; nodes named like EasyNav's ("system_node",
  // "controller_node", ...) play the rest of EasyNav. Everything else is "outside" EasyNav.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_recovery_node(
    double debounce = 0.0, double startup_grace = 0.0,
    const std::vector<std::string> & ignored_topics = {})
  {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides{
      {"graph.error_debounce", debounce},
      {"graph.startup_grace", startup_grace},
    };
    if (!ignored_topics.empty()) {
      overrides.emplace_back("graph.ignored_topics", ignored_topics);
    }
    options.parameter_overrides(overrides);
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>("recovery_node", ns_, options);
  }

  rclcpp::Node::SharedPtr make_node(const std::string & name)
  {
    return std::make_shared<rclcpp::Node>(name, ns_);
  }

  // Graph discovery is asynchronous: keep evaluating until the expected level shows up.
  static DiagnosticStatus wait_for_level(
    easynav::RosGraphEvaluator & eval, easynav::NavState & nav_state, uint8_t level)
  {
    DiagnosticStatus status;
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(120ms);
      eval.internal_update(nav_state);
      if (nav_state.has("diagnostics.graph")) {
        status = nav_state.get<DiagnosticStatus>("diagnostics.graph");
        if (status.level == level) {break;}
      }
    }
    return status;
  }

  static std::string value_of(const DiagnosticStatus & status, const std::string & key)
  {
    for (const auto & kv : status.values) {
      if (kv.key == key) {return kv.value;}
    }
    return "";
  }

  std::string ns_;
  static inline int counter_ {0};
};

TEST_F(RosGraphEvaluatorTestCase, OkWhenWiredAndReportsUnstamped)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto base = make_node("base_driver");
  auto odom_src = make_node("odom_source");

  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto odom_sub = system->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, [](nav_msgs::msg::Odometry::SharedPtr) {});
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});
  auto odom_pub = odom_src->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;
  EXPECT_EQ(status.hardware_id, "ros_graph");
  EXPECT_EQ(value_of(status, "velocity_type"), "unstamped");
  EXPECT_EQ(value_of(status, "velocity_topic"), ns_ + "/cmd_vel");
  // OK only says where the velocity goes.
  EXPECT_EQ(status.message, "velocity: " + ns_ + "/cmd_vel [unstamped]");
  EXPECT_EQ(status.values.size(), 2u);
}

TEST_F(RosGraphEvaluatorTestCase, ReportsStampedVelocityOutput)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto base = make_node("base_driver");

  auto vel_pub = system->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_stamped", 10, [](geometry_msgs::msg::TwistStamped::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;
  EXPECT_EQ(value_of(status, "velocity_type"), "stamped");
}

TEST_F(RosGraphEvaluatorTestCase, ErrorWhenVelocityOutputHasNoSubscriber)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("no subscriber for velocity output"),
    std::string::npos) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, EasyNavOwnSubscriberIsNotAConsumer)
{
  // A plugin inside EasyNav listening to its own cmd_vel does not move the robot.
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto controller = make_node("controller_node");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = controller->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  // Give discovery time to see the internal subscriber before judging.
  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, ErrorWhenSubscriptionHasNoPublisher)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto controller = make_node("controller_node");
  auto base = make_node("base_driver");

  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});
  auto scan_sub = controller->create_subscription<nav_msgs::msg::Odometry>(
    "missing_odom", 10, [](nav_msgs::msg::Odometry::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(value_of(status, "unfed_subscriptions"), "1");

  // The message must say which topic, who needs it, and how to declare it optional.
  EXPECT_NE(
    status.message.find(
      ns_ + "/missing_odom [nav_msgs/Odometry], needed by " + ns_ + "/controller_node"),
    std::string::npos) << status.message;
  EXPECT_NE(
    status.message.find(
      "if optional, add to graph.ignored_topics of " + ns_ + "/recovery_node: \"missing_odom\""),
    std::string::npos) << status.message;
  EXPECT_EQ(value_of(status, "suggested_ignored_topic"), "missing_odom");
}

TEST_F(RosGraphEvaluatorTestCase, FollowingTheSuggestionDeclaresTheTopicOptional)
{
  // A new plugin with an optional input: adding it to ignored_topics, as the ERROR message
  // suggests, makes it acceptable.
  auto recovery = make_recovery_node(0.0, 0.0, {"initialpose", "optional_input"});
  auto system = make_node("system_node");
  auto controller = make_node("controller_node");
  auto base = make_node("base_driver");

  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});
  auto optional_sub = controller->create_subscription<nav_msgs::msg::Odometry>(
    "optional_input", 10, [](nav_msgs::msg::Odometry::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, IgnoredTopicsDoNotCount)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto base = make_node("base_driver");

  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});
  auto init_sub = system->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10, [](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {});
  auto goal_sub = system->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10, [](geometry_msgs::msg::PoseStamped::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, ErrorsEvenWithoutAnActiveGoal)
{
  // A miswired EasyNav cannot navigate correctly whether or not it has a goal right now.
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, WarnsDuringStartupGrace)
{
  auto recovery = make_recovery_node(0.0, 60.0);
  auto system = make_node("system_node");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_NE(status.message.find("startup grace"), std::string::npos) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, WarnsWhileDebouncing)
{
  auto recovery = make_recovery_node(60.0);
  auto system = make_node("system_node");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::WARN);
  EXPECT_EQ(status.level, DiagnosticStatus::WARN) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, WarnsUntilActivated)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto base = make_node("base_driver");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  easynav::NavState nav_state;

  std::this_thread::sleep_for(120ms);
  eval.internal_update(nav_state);
  EXPECT_EQ(
    nav_state.get<DiagnosticStatus>("diagnostics.graph").level, DiagnosticStatus::WARN);

  eval.on_activate();
  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, TopologyIsFixedUntilReactivation)
{
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto base = make_node("base_driver");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  ASSERT_EQ(status.level, DiagnosticStatus::OK) << status.message;

  // A subscription created after activation is not part of the discovered topology...
  auto late_sub = system->create_subscription<nav_msgs::msg::Odometry>(
    "late_odom", 10, [](nav_msgs::msg::Odometry::SharedPtr) {});
  status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;

  // ...until the node is activated again.
  eval.on_deactivate();
  eval.on_activate();
  status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find(ns_ + "/late_odom"), std::string::npos) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, ReportsMissingClockEvenThoughSimTimeIsStopped)
{
  // EasyNav launched with use_sim_time but without the simulator: /clock never arrives, so the
  // nodes' clocks stay at 0. The evaluator must still run (wall time) and say why.
  rclcpp::NodeOptions sim_options;
  sim_options.parameter_overrides({
    {"use_sim_time", true},
    {"graph.error_debounce", 0.0},
    {"graph.startup_grace", 0.0},
  });
  auto recovery = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "recovery_node", ns_, sim_options);
  auto system = std::make_shared<rclcpp::Node>(
    "system_node", ns_, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  auto base = make_node("base_driver");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::ERROR);
  ASSERT_EQ(status.level, DiagnosticStatus::ERROR) << status.message;
  EXPECT_NE(status.message.find("/clock is not published"), std::string::npos) << status.message;
  // One clear cause, not one "no publisher" entry per node.
  EXPECT_EQ(status.message.find("no publisher for"), std::string::npos) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, IgnoredTopicsMatchInTheRootNamespace)
{
  // Without a namespace, "goal_pose" must still match "/goal_pose".
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"graph.error_debounce", 0.0},
    {"graph.startup_grace", 0.0},
    {"graph.ignored_topics", std::vector<std::string>{"root_optional_input"}},
  });
  auto recovery = std::make_shared<rclcpp_lifecycle::LifecycleNode>("recovery_node", options);
  auto system = std::make_shared<rclcpp::Node>("system_node");
  auto base = std::make_shared<rclcpp::Node>("root_base_driver");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("root_cmd_vel", 10);
  auto vel_sub = base->create_subscription<geometry_msgs::msg::Twist>(
    "root_cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});
  auto optional_sub = system->create_subscription<nav_msgs::msg::Odometry>(
    "root_optional_input", 10, [](nav_msgs::msg::Odometry::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.message;
}

TEST_F(RosGraphEvaluatorTestCase, MonitoringToolsDoNotCountAsVelocityConsumers)
{
  // Only EasyNav's TUI and a `ros2 topic echo` listen to cmd_vel: nothing drives the robot.
  auto recovery = make_recovery_node();
  auto system = make_node("system_node");
  auto tui = make_node("easynav_tui_status_commanding");
  auto echo = make_node("_ros2cli_12345");
  auto vel_pub = system->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto tui_sub = tui->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});
  auto echo_sub = echo->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](geometry_msgs::msg::Twist::SharedPtr) {});

  easynav::RosGraphEvaluator eval;
  eval.initialize(recovery, "graph");
  eval.on_activate();
  easynav::NavState nav_state;

  // Give discovery time to see both tools before judging.
  const auto status = wait_for_level(eval, nav_state, DiagnosticStatus::OK);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR) << status.message;
  EXPECT_NE(status.message.find("no subscriber for velocity output"), std::string::npos) <<
    status.message;
}
