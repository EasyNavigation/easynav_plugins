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

#include <memory>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_collision_safety_reflex/CollisionSafetyReflex.hpp"
#include "easynav_core/VelocityCommand.hpp"

class CollisionSafetyReflexTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(CollisionSafetyReflexTest, InitializeTwiceOnSameNodeDoesNotThrow)
{
  // As happens across a cleanup/configure cycle of the node that loads it.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("reflex_reconfigure_node");

  auto reflex1 = std::make_shared<easynav::CollisionSafetyReflex>();
  ASSERT_NO_THROW(reflex1->initialize(node, "collision"));

  auto reflex2 = std::make_shared<easynav::CollisionSafetyReflex>();
  ASSERT_NO_THROW(reflex2->initialize(node, "collision"));
}

TEST_F(CollisionSafetyReflexTest, DoesNotInterveneWithoutCommandedMotion)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("reflex_idle_node");
  auto reflex = std::make_shared<easynav::CollisionSafetyReflex>();
  reflex->initialize(node, "collision");

  easynav::NavState nav_state;
  EXPECT_FALSE(reflex->internal_check_and_mitigate(nav_state));

  easynav::velocity_command::propose(
    nav_state, easynav::VelocitySource::CONTROLLER, geometry_msgs::msg::TwistStamped());
  EXPECT_FALSE(reflex->internal_check_and_mitigate(nav_state));
}
