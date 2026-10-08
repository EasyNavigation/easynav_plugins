// Copyright 2025 Intelligent Robotics Lab
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
/// \brief The deprecated limit parameters of this controller still work, and "robot_limits.*"
/// in controller_node takes precedence over them.

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_mpc_controller/MPCController.hpp"

class MpcRobotLimitsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Limits the node enforces once the controller is initialized with \p overrides.
  static easynav::RobotLimits limits_with(std::vector<rclcpp::Parameter> overrides)
  {
    auto node = std::make_shared<easynav::ControllerNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    auto controller = std::make_shared<easynav::MPCController>();
    controller->initialize(node, "ctrl");
    return node->get_robot_limits();
  }
};

TEST_F(MpcRobotLimitsTest, DeprecatedParametersStillApply)
{
  const auto limits = limits_with(
  {
    {"ctrl.max_linear_velocity", 0.7},
    {"ctrl.max_angular_velocity", 1.3}});
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 0.7) << "max_linear_velocity";
  EXPECT_DOUBLE_EQ(limits.max_angular_vel, 1.3) << "max_angular_velocity";
}

TEST_F(MpcRobotLimitsTest, RobotLimitsTakePrecedence)
{
  const auto limits = limits_with(
  {
    {"robot_limits.max_linear_vel", 0.25},
    {"ctrl.max_linear_velocity", 0.7}});
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 0.25);
}

TEST_F(MpcRobotLimitsTest, WithoutDeprecatedParametersTheRobotLimitsApply)
{
  const auto limits = limits_with({{"robot_limits.max_linear_vel", 0.25}});
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 0.25);
  EXPECT_DOUBLE_EQ(limits.max_linear_decel, easynav::RobotLimits{}.max_linear_decel);
}
