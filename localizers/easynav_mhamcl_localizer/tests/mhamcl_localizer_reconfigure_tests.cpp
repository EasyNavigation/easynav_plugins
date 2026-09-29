// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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
/// \brief Regression test: initialize() may run several times on the same node (cleanup/configure).

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_mhamcl_localizer/MHAMCLLocalizer.hpp"

class MHAMCLLocalizerReconfigureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(MHAMCLLocalizerReconfigureTest, InitializeRepeatedlyOnSameNodeDoesNotThrow)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  for (int i = 0; i < 3; ++i) {
    auto plugin = std::make_shared<easynav::mhamcl::MHAMCLLocalizer>();
    ASSERT_NO_THROW(plugin->initialize(node, "test_localizer")) << "initialize #" << i;
  }
}

TEST_F(MHAMCLLocalizerReconfigureTest, ConfiguredValuesSurviveReinitialization)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions().append_parameter_override("test_localizer.initial_pose.x", 2.5));

  auto plugin1 = std::make_shared<easynav::mhamcl::MHAMCLLocalizer>();
  plugin1->initialize(node, "test_localizer");
  node->set_parameter(rclcpp::Parameter("test_localizer.initial_pose.y", -1.5));

  auto plugin2 = std::make_shared<easynav::mhamcl::MHAMCLLocalizer>();
  ASSERT_NO_THROW(plugin2->initialize(node, "test_localizer"));
  EXPECT_DOUBLE_EQ(node->get_parameter("test_localizer.initial_pose.x").as_double(), 2.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("test_localizer.initial_pose.y").as_double(), -1.5);
}
