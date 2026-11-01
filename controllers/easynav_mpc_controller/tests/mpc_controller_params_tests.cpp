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
/// \brief MPCController parameters: motion models, validation, deprecated and removed ones, and
/// the topics under the node's name.

#include <memory>
#include <string>
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
  double xy_tolerance() const {return xy_goal_tolerance_;}
  double yaw_tolerance() const {return yaw_goal_tolerance_;}
  int horizon() const {return optimizer_.params().horizon_steps;}
  easynav::mpc::MotionModel model() const {return model_;}
};

int count = 0;

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_node(
  const std::vector<rclcpp::Parameter> & overrides, const std::string & ns = "")
{
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "controller_node_" + std::to_string(count++), ns,
    rclcpp::NodeOptions().parameter_overrides(overrides));
}

}  // namespace

class MpcParamsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(MpcParamsTest, EveryMotionModelAndDiffByDefault)
{
  using easynav::mpc::MotionModel;
  for (const auto & [name, model] : std::vector<std::pair<std::string, MotionModel>>{
    {"", MotionModel::Diff}, {"diff", MotionModel::Diff}, {"omni", MotionModel::Omni},
    {"ackermann", MotionModel::Ackermann}, {"aerial", MotionModel::Aerial}})
  {
    std::vector<rclcpp::Parameter> p;
    if (!name.empty()) {p.emplace_back("mpc.motion_model", name);}
    auto node = make_node(p);
    TestMpc mpc;
    ASSERT_NO_THROW(mpc.initialize(node, "mpc")) << name;
    EXPECT_EQ(mpc.model(), model) << name;
  }
}

TEST_F(MpcParamsTest, InvalidParametersFailToInitialize)
{
  for (const auto & param : std::vector<rclcpp::Parameter>{
    {"mpc.motion_model", "bicycle"},
    {"mpc.horizon_steps", 0},
    {"mpc.dt", 0.0},
    {"mpc.dt", -0.1},
    {"mpc.time_constant", -0.2},
    {"mpc.latency", -0.05},
    {"mpc.max_iterations", 0},
    {"mpc.tolerance", 0.0},
    {"mpc.curvature_window", 0.0},
    {"mpc.max_lateral_acc", 0.0},
    {"mpc.near_weight", 0.0},
    {"mpc.near_weight", 1.5}})
  {
    auto node = make_node({param});
    TestMpc mpc;
    EXPECT_THROW(mpc.initialize(node, "mpc"), std::runtime_error) << param.get_name();
  }
}

TEST_F(MpcParamsTest, RemovedParametersAreHarmless)
{
  const std::vector<rclcpp::Parameter> removed = {
    {"mpc.safety_radius", 0.5}, {"mpc.use_collision_checker", true}, {"mpc.verbose", true}};
  auto node = make_node(removed);
  TestMpc mpc;
  EXPECT_NO_THROW(mpc.initialize(node, "mpc"));
  EXPECT_EQ(mpc.horizon(), 20);
}

TEST_F(MpcParamsTest, DeprecatedGoalTolerancesApplyUnlessTheNewOnesAreGiven)
{
  {
    auto node = make_node({{"mpc.fallback_goal_pos_tol", 0.3}, {"mpc.fallback_goal_yaw_tol", 0.4}});
    TestMpc mpc;
    mpc.initialize(node, "mpc");
    EXPECT_DOUBLE_EQ(mpc.xy_tolerance(), 0.3);
    EXPECT_DOUBLE_EQ(mpc.yaw_tolerance(), 0.4);
  }
  {
    auto node = make_node({{"mpc.fallback_goal_pos_tol", 0.3}, {"mpc.xy_goal_tolerance", 0.15}});
    TestMpc mpc;
    mpc.initialize(node, "mpc");
    EXPECT_DOUBLE_EQ(mpc.xy_tolerance(), 0.15) << "the new one wins";
  }
}

TEST_F(MpcParamsTest, TopicsAreUnderTheNodeSoRobotsDoNotShareThem)
{
  auto node = make_node({}, "robot1");
  TestMpc mpc;
  mpc.initialize(node, "mpc");
  const std::string prefix = std::string("/robot1/") + node->get_name() + "/mpc/";
  const auto topics = node->get_topic_names_and_types();
  for (const char * t : {"predicted", "reference", "obstacles"}) {
    EXPECT_TRUE(topics.count(prefix + t)) << prefix + t;
  }
  EXPECT_FALSE(topics.count("/mpc/path")) << "no global topics any more";
}
