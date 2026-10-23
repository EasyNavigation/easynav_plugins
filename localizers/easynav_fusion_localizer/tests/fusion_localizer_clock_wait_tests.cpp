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
/// \brief With use_sim_time, the filters wait for /clock while configuring, but a shutdown
/// (a second signal to system_main) stops the wait.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "easynav_fusion_localizer/FusionLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

namespace
{

std::atomic<int> g_clock_warnings{0};

void count_clock_warnings(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char msg[512];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(msg, sizeof(msg), format, copy);
  va_end(copy);
  if (severity == RCUTILS_LOG_SEVERITY_WARN &&
    std::string(msg).find("Waiting for the ROS clock to start") != std::string::npos)
  {
    ++g_clock_warnings;
  }
}

}  // namespace

class FusionClockWaitTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override
  {
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }

  // Initializes a local filter in another thread; the node is kept in node_.
  std::future<void> initialize(bool use_sim_time)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      {"use_sim_time", use_sim_time},
      {"fusion.local_filter.frequency", 30.0},
      {"fusion.local_filter.two_d_mode", true},
      {"fusion.local_filter.publish_tf", false},
      {"fusion.local_filter.world_frame", "odom"},
      {"fusion.local_filter.odom0", "clock_wait_odom"},
    });
    node_ = std::make_shared<easynav::LocalizerNode>(options);
    node_->declare_parameter<std::vector<std::string>>(
      "localizer_types", std::vector<std::string>{});
    localizer_ = std::make_shared<easynav::FusionLocalizer>();
    return std::async(
      std::launch::async, [this]() {localizer_->initialize(node_, "fusion");});
  }

  std::shared_ptr<easynav::LocalizerNode> node_;
  std::shared_ptr<easynav::FusionLocalizer> localizer_;
};

TEST_F(FusionClockWaitTest, WithoutSimTimeItDoesNotWait)
{
  auto done = initialize(false);
  ASSERT_EQ(done.wait_for(5s), std::future_status::ready);
  EXPECT_NO_THROW(done.get());
}

TEST_F(FusionClockWaitTest, WithSimTimeItWaitsUntilTheClockStarts)
{
  auto done = initialize(true);
  EXPECT_EQ(done.wait_for(1s), std::future_status::timeout);  // no /clock yet

  auto clock_node = std::make_shared<rclcpp::Node>("clock_wait_publisher");
  auto pub = clock_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  rosgraph_msgs::msg::Clock clock;
  clock.clock.sec = 100;
  const auto end = std::chrono::steady_clock::now() + 10s;
  while (done.wait_for(50ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < end)
  {
    pub->publish(clock);
  }
  ASSERT_EQ(done.wait_for(0s), std::future_status::ready);
  EXPECT_NO_THROW(done.get());
  EXPECT_EQ(node_->now().seconds(), 100.0);
}

TEST_F(FusionClockWaitTest, WhileItWaitsItSaysWhy)
{
  const auto handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(count_clock_warnings);
  g_clock_warnings = 0;
  auto done = initialize(true);
  EXPECT_EQ(done.wait_for(4s), std::future_status::timeout);
  EXPECT_EQ(g_clock_warnings, 0);  // every 5 s
  EXPECT_EQ(done.wait_for(2s), std::future_status::timeout);
  EXPECT_EQ(g_clock_warnings, 1);
  rclcpp::shutdown();
  ASSERT_EQ(done.wait_for(5s), std::future_status::ready);
  rcutils_logging_set_output_handler(handler);
  try {
    done.get();
  } catch (const std::exception &) {
  }
}

TEST_F(FusionClockWaitTest, AShutdownStopsTheWait)
{
  auto done = initialize(true);
  EXPECT_EQ(done.wait_for(1s), std::future_status::timeout);
  rclcpp::shutdown();
  // Stops waiting; what is left of the initialization may fail without a context
  ASSERT_EQ(done.wait_for(5s), std::future_status::ready);
  try {
    done.get();
  } catch (const std::exception &) {
  }
}

// Before or while it waits, whichever comes first
TEST_F(FusionClockWaitTest, AShutdownRightAwayDoesNotHang)
{
  auto done = initialize(true);
  rclcpp::shutdown();
  ASSERT_EQ(done.wait_for(5s), std::future_status::ready);
  try {
    done.get();
  } catch (const std::exception &) {
  }
}
