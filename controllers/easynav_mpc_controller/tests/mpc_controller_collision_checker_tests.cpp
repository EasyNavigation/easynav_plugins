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
/// \brief MPC's in-loop obstacle constraint is enabled by its own "use_collision_checker" (no
/// longer by the removed "colision_checker.active").

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_mpc_controller/MPCController.hpp"

namespace
{

class TestMpc : public easynav::MPCController
{
public:
  bool uses_collision_checker() const {return use_collision_checker_;}
};

}  // namespace

class MpcCollisionCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static bool uses_collision_checker(const std::vector<rclcpp::Parameter> & overrides)
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "controller_node", rclcpp::NodeOptions().parameter_overrides(overrides));
    TestMpc mpc;
    mpc.initialize(node, "mpc");
    return mpc.uses_collision_checker();
  }
};

TEST_F(MpcCollisionCheckerTest, OffByDefault)
{
  EXPECT_FALSE(uses_collision_checker({}));
}

TEST_F(MpcCollisionCheckerTest, EnabledByItsOwnParameter)
{
  EXPECT_TRUE(uses_collision_checker({{"mpc.use_collision_checker", true}}));
  EXPECT_FALSE(uses_collision_checker({{"mpc.use_collision_checker", false}}));
}

TEST_F(MpcCollisionCheckerTest, TheRemovedCollisionCheckerParameterDoesNotApply)
{
  EXPECT_FALSE(uses_collision_checker({{"colision_checker.active", true}}));
}
