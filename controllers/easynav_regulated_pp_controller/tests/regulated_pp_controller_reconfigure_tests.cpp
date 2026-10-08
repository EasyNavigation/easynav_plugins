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
/// \brief Regression test: a plugin must tolerate initialize() being
/// called twice on the same node (as happens across a cleanup/reconfigure
/// cycle) without throwing.

#include <vector>
#include <string>
#include "easynav_regulated_pp_controller/RegulatedPurePursuitController.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class RegulatedPurePursuitControllerReconfigureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(RegulatedPurePursuitControllerReconfigureTest, InitializeTwiceOnSameNodeDoesNotThrow)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "regulated_pp_controller_reconfigure_test");

  auto plugin1 = std::make_shared<easynav::RegulatedPurePursuitController>();
  ASSERT_NO_THROW(plugin1->initialize(node, "test_controller"));

  auto plugin2 = std::make_shared<easynav::RegulatedPurePursuitController>();
  ASSERT_NO_THROW(plugin2->initialize(node, "test_controller"));
}

TEST_F(RegulatedPurePursuitControllerReconfigureTest,
  InitializeWhenAnotherPluginLeftSomeOfItsParameters)
{
  // A plugin of another type under the same name may have left any subset of them declared.
  std::vector<std::string> names;
  std::vector<rclcpp::ParameterValue> values;
  {
    auto node = rclcpp_lifecycle::LifecycleNode::make_shared(
      "regulated_pp_controller_reconfigure_test");

    auto plugin = std::make_shared<easynav::RegulatedPurePursuitController>();
    plugin->initialize(node, "test_controller");
    for (const auto & n : node->list_parameters({"test_controller"}, 10).names) {
      try {
        values.push_back(node->get_parameter(n).get_parameter_value());
        names.push_back(n);
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        // Declared by type only, without a value: nothing another plugin could have left.
      }
    }
  }
  ASSERT_FALSE(names.empty());

  for (std::size_t i = 0; i < names.size(); ++i) {
    auto node = rclcpp_lifecycle::LifecycleNode::make_shared(
      "regulated_pp_controller_reconfigure_test");

    node->declare_parameter(names[i], values[i]);
    auto plugin = std::make_shared<easynav::RegulatedPurePursuitController>();
    ASSERT_NO_THROW(plugin->initialize(node, "test_controller")) << "left declared: " << names[i];
    for (const auto & n : names) {
      EXPECT_TRUE(node->has_parameter(n)) << "left declared: " << names[i] << ", missing: " << n;
    }
  }
}
