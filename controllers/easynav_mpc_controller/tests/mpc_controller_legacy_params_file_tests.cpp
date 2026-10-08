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
/// \brief A real parameter file from before robot_limits (easynav_indoor_testcase,
/// bonxai.amcl.params.urjc_alt_imu.yaml) keeps working: no limits of its own, and those of inactive controllers are ignored.

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_controller/ControllerNode.hpp"

class MpcLegacyParamsFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(MpcLegacyParamsFileTest, LegacyParamsFileStillConfiguresTheLimits)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file",
        std::string(TEST_DATA_DIR) + "/legacy_bonxai.amcl.params.urjc_alt_imu.yaml"}));
  ASSERT_EQ(
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto limits = node->get_robot_limits();
  const easynav::RobotLimits defaults;
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, defaults.max_linear_vel) << "max_linear_vel";
  EXPECT_DOUBLE_EQ(limits.min_linear_vel, defaults.min_linear_vel) << "min_linear_vel";
  EXPECT_DOUBLE_EQ(limits.max_angular_vel, defaults.max_angular_vel) << "max_angular_vel";
  EXPECT_DOUBLE_EQ(limits.max_linear_acc, defaults.max_linear_acc) << "max_linear_acc";
  EXPECT_DOUBLE_EQ(limits.max_linear_decel, defaults.max_linear_decel) << "max_linear_decel";
  EXPECT_DOUBLE_EQ(limits.max_angular_acc, defaults.max_angular_acc) << "max_angular_acc";
  EXPECT_DOUBLE_EQ(limits.max_angular_decel, defaults.max_angular_decel) << "max_angular_decel";
}
