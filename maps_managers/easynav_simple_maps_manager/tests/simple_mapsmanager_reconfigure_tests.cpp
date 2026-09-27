// Copyright 2025 Intelligent Robotics Lab
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
/// \brief Regression test: a plugin must tolerate initialize() being
/// called twice on the same node (as happens across a cleanup/reconfigure
/// cycle) without throwing.

#include "easynav_simple_maps_manager/SimpleMapsManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class SimpleMapsManagerReconfigureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(SimpleMapsManagerReconfigureTest, InitializeTwiceOnSameNodeDoesNotThrow)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "simple_maps_manager_reconfigure_test");

  auto plugin1 = std::make_shared<easynav::SimpleMapsManager>();
  ASSERT_NO_THROW(plugin1->initialize(node, "test_maps_manager"));

  auto plugin2 = std::make_shared<easynav::SimpleMapsManager>();
  ASSERT_NO_THROW(plugin2->initialize(node, "test_maps_manager"));
}
