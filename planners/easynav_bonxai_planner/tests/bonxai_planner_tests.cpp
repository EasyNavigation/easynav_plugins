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
/// \brief BonxaiPlanner in the NavState: the path to the goal in 3D, and when there is none.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "bonxai/probabilistic_map.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_bonxai_planner/BonxaiPlanner.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"

namespace
{

/// A building: a closed 10 x 10 x 20 m box at the origin, on a floor at z = 0.
std::shared_ptr<Bonxai::ProbabilisticMap> town()
{
  auto map = std::make_shared<Bonxai::ProbabilisticMap>(0.5);
  for (double a = -5.0; a <= 5.0; a += 0.5) {
    for (double z = 0.0; z <= 20.0; z += 0.5) {
      for (double s : {-5.0, 5.0}) {
        map->addHitPoint(Eigen::Vector3d(s, a, z));
        map->addHitPoint(Eigen::Vector3d(a, s, z));
      }
    }
    for (double b = -5.0; b <= 5.0; b += 0.5) {
      map->addHitPoint(Eigen::Vector3d(a, b, 20.0));
    }
  }
  for (double x = -30.0; x <= 30.0; x += 0.5) {
    for (double y = -30.0; y <= 30.0; y += 0.5) {
      map->addHitPoint(Eigen::Vector3d(x, y, 0.0));
    }
  }
  return map;
}

nav_msgs::msg::Goals goal_at(double x, double y, double z, const std::string & frame)
{
  nav_msgs::msg::Goals goals;
  goals.header.frame_id = frame;
  geometry_msgs::msg::PoseStamped g;
  g.header.frame_id = frame;
  g.pose.position.x = x;
  g.pose.position.y = y;
  g.pose.position.z = z;
  g.pose.orientation.z = std::sin(0.5);
  g.pose.orientation.w = std::cos(0.5);
  goals.goals.push_back(g);
  return goals;
}

}  // namespace

class BonxaiPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    const std::vector<rclcpp::Parameter> overrides = {
      {"bonxai.resolution", 1.0}, {"bonxai.min_clearance", 2.0},
      {"bonxai.preferred_clearance", 0.0}, {"bonxai.margin", 5.0},
      {"bonxai.max_altitude", 30.0}};
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "planner_node_" + std::to_string(count_++),
      rclcpp::NodeOptions().parameter_overrides(overrides));
    planner_.initialize(node_, "bonxai");
    nav_msgs::msg::Odometry robot;
    robot.pose.pose.position.x = -15.0;
    robot.pose.pose.position.z = 5.0;
    robot.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", robot);
  }

  std::string map_frame() const
  {
    return easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame;
  }

  static int count_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::BonxaiPlanner planner_;
  easynav::NavState nav_state_;
};

int BonxaiPlannerTest::count_ = 0;

TEST_F(BonxaiPlannerTest, AroundTheBuildingAtItsDistanceAndToTheGoalsHeight)
{
  nav_state_.set("map.bonxai", town());
  nav_state_.set("goals", goal_at(15.0, 0.0, 12.0, map_frame()));
  planner_.update(nav_state_);
  ASSERT_TRUE(nav_state_.has("path"));
  const auto path = nav_state_.get<nav_msgs::msg::Path>("path");
  ASSERT_GT(path.poses.size(), 10u);
  const auto & first = path.poses.front().pose.position;
  const auto & last = path.poses.back().pose;
  EXPECT_NEAR(first.x, -15.0, 1e-6);
  EXPECT_NEAR(last.position.x, 15.0, 1e-6);
  EXPECT_NEAR(last.position.z, 12.0, 1e-6);
  EXPECT_NEAR(last.orientation.z, std::sin(0.5), 1e-9) << "the goal's heading";
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y, b.z - a.z), 0.5 + 1e-6) << "evenly spaced";
    EXPECT_GE(planner_.grid().distance_at({b.x, b.y, b.z}), 2.0 - 1.0) << i;
  }
}

TEST_F(BonxaiPlannerTest, NoPathWithoutAMapForAnotherFrameOrToAGoalInsideTheBuilding)
{
  nav_state_.set("goals", goal_at(15.0, 0.0, 12.0, map_frame()));
  planner_.update(nav_state_);
  EXPECT_FALSE(nav_state_.has("path")) << "no map: nothing yet";

  nav_state_.set("map.bonxai", town());
  nav_state_.set("goals", goal_at(15.0, 0.0, 12.0, "odom_other"));
  planner_.update(nav_state_);
  ASSERT_TRUE(nav_state_.has("path"));
  EXPECT_TRUE(nav_state_.get<nav_msgs::msg::Path>("path").poses.empty());

  nav_state_.set("goals", goal_at(0.0, 0.0, 10.0, map_frame()));
  planner_.update(nav_state_);
  EXPECT_TRUE(nav_state_.get<nav_msgs::msg::Path>("path").poses.empty()) << "inside";
}

TEST_F(BonxaiPlannerTest, InvalidParametersFailToInitialize)
{
  for (const auto & p : std::vector<rclcpp::Parameter>{
    {"bonxai.resolution", 0.0}, {"bonxai.path_step", 0.0}, {"bonxai.min_clearance", -1.0}})
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "bad_planner_" + std::to_string(count_++), rclcpp::NodeOptions().parameter_overrides({p}));
    easynav::BonxaiPlanner planner;
    EXPECT_THROW(planner.initialize(node, "bonxai"), std::runtime_error) << p.get_name();
  }
}
