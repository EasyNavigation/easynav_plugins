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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_obstacle_too_close_evaluator/ObstacleTooCloseEvaluator.hpp"

class ObstacleTooCloseEvaluatorTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }

  static nav_msgs::msg::Odometry make_odom(double vx, double wz)
  {
    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = wz;
    return odom;
  }

  static easynav::PointPerception make_obstacle_at(double x, double y)
  {
    easynav::PointPerception perception;
    perception.frame_id = "base_link";
    perception.stamp = rclcpp::Time(0);
    perception.valid = true;
    perception.data.points.resize(1);
    perception.data.points[0].x = x;
    perception.data.points[0].y = y;
    perception.data.points[0].z = 0.0;
    return perception;
  }

  std::shared_ptr<easynav::ObstacleTooCloseEvaluator> make_ready_evaluator(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name)
  {
    auto eval = std::make_shared<easynav::ObstacleTooCloseEvaluator>();
    eval->initialize(node, name);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    return eval;
  }
};

TEST_F(ObstacleTooCloseEvaluatorTestCase, OkWithoutRobotPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_pose_node");
  auto eval = make_ready_evaluator(node, "close1");

  easynav::NavState nav_state;
  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.close1");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ObstacleTooCloseEvaluatorTestCase, OkWhileStillMovingEvenIfObstacleIsClose)
{
  // Compound condition: must not fire while the robot is still moving (e.g. the level-0 reflex
  // is still braking). See docs/recoveries_easynav.md, §5.2.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_moving_node");
  auto eval = make_ready_evaluator(node, "close2");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", make_odom(0.5, 0.0));  // still moving
  nav_state.set("obstacle_scan", make_obstacle_at(0.1, 0.0));  // very close

  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.close2");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ObstacleTooCloseEvaluatorTestCase, OkWhenStoppedButNoObstacleNearby)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_stopped_far_node");
  auto eval = make_ready_evaluator(node, "close3");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", make_odom(0.0, 0.0));
  nav_state.set("obstacle_scan", make_obstacle_at(5.0, 0.0));  // far away

  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.close3");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ObstacleTooCloseEvaluatorTestCase, ErrorWhenStoppedTooCloseToAnObstacle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_stopped_close_node");
  auto eval = make_ready_evaluator(node, "close4");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", make_odom(0.0, 0.0));
  nav_state.set("obstacle_scan", make_obstacle_at(0.2, 0.0));  // well within default safe_distance

  eval->internal_update(nav_state);

  const auto & status =
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.close4");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.hardware_id, "obstacle_proximity");
  ASSERT_EQ(status.values.size(), 2u);
  EXPECT_EQ(status.values[0].key, "distance");
  EXPECT_NEAR(std::stod(status.values[0].value), 0.2, 1e-3);
}
