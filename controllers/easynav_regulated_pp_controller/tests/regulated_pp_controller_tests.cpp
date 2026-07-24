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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "easynav_regulated_pp_controller/regulation_functions.hpp"
#include "easynav_regulated_pp_controller/dynamic_window_pure_pursuit_functions.hpp"
#include "easynav_regulated_pp_controller/RegulatedPurePursuitController.hpp"

using easynav::heuristics::approachVelocityConstraint;
using easynav::heuristics::calculateCurvature;
using easynav::heuristics::curvatureConstraint;
using easynav::heuristics::obstacleConstraint;

TEST(RegulationFunctions, CalculateCurvatureStraightAhead)
{
  // A lookahead point directly ahead has zero lateral offset -> zero curvature.
  EXPECT_NEAR(calculateCurvature(1.0, 0.0), 0.0, 1e-6);
}

TEST(RegulationFunctions, CalculateCurvatureLateralOffset)
{
  // k = 2y / (x^2+y^2)
  const double k = calculateCurvature(1.0, 0.5);
  EXPECT_NEAR(k, 2.0 * 0.5 / (1.0 * 1.0 + 0.5 * 0.5), 1e-9);
}

TEST(RegulationFunctions, CalculateCurvatureDegenerate)
{
  EXPECT_NEAR(calculateCurvature(0.0, 0.0), 0.0, 1e-9);
}

TEST(RegulationFunctions, CurvatureConstraintNoRegulationOnGentleTurn)
{
  // Radius = 1/curvature = 10 m, well above the 0.9 m min radius: no regulation applied.
  const double v = curvatureConstraint(1.0, 0.1, 0.9);
  EXPECT_NEAR(v, 1.0, 1e-9);
}

TEST(RegulationFunctions, CurvatureConstraintSlowsOnSharpTurn)
{
  // Radius = 1/2.0 = 0.5 m, below the 1.0 m min radius: velocity should be reduced.
  const double v = curvatureConstraint(1.0, 2.0, 1.0);
  EXPECT_LT(v, 1.0);
  EXPECT_GT(v, 0.0);
}

TEST(RegulationFunctions, ObstacleConstraintNoObstacleNearby)
{
  const double v = obstacleConstraint(1.0, 5.0, 0.3, 1.0);
  EXPECT_NEAR(v, 1.0, 1e-9);
}

TEST(RegulationFunctions, ObstacleConstraintSlowsNearObstacle)
{
  const double v = obstacleConstraint(1.0, 0.1, 0.3, 1.0);
  EXPECT_LT(v, 1.0);
  EXPECT_GT(v, 0.0);
}

TEST(RegulationFunctions, ObstacleConstraintIgnoresInfiniteDistance)
{
  const double v = obstacleConstraint(1.0, std::numeric_limits<double>::infinity(), 0.3, 1.0);
  EXPECT_NEAR(v, 1.0, 1e-9);
}

TEST(RegulationFunctions, ApproachVelocityConstraintFarFromGoal)
{
  const double v = approachVelocityConstraint(1.0, 10.0, 10.0, 0.05, 1.0);
  EXPECT_NEAR(v, 1.0, 1e-9);
}

TEST(RegulationFunctions, ApproachVelocityConstraintNearGoal)
{
  const double v = approachVelocityConstraint(1.0, 0.5, 0.5, 0.05, 1.0);
  EXPECT_LT(v, 1.0);
  EXPECT_GE(v, 0.05);
}

TEST(DynamicWindowPurePursuit, ComputeDynamicWindowClampsToAccelLimits)
{
  geometry_msgs::msg::Twist current_speed;
  current_speed.linear.x = 0.0;
  current_speed.angular.z = 0.0;

  const auto window = easynav::dynamic_window_pure_pursuit::computeDynamicWindow(
    current_speed, /*max_linear_vel=*/1.0, /*min_linear_vel=*/-1.0,
    /*max_angular_vel=*/1.0, /*min_angular_vel=*/-1.0,
    /*max_linear_accel=*/2.0, /*max_linear_decel=*/2.0,
    /*max_angular_accel=*/2.0, /*max_angular_decel=*/2.0, /*dt=*/0.1);

  EXPECT_NEAR(window.max_linear_vel, 0.2, 1e-9);
  EXPECT_NEAR(window.min_linear_vel, -0.2, 1e-9);
}

TEST(DynamicWindowPurePursuit, ComputeOptimalVelocityZeroCurvatureForward)
{
  easynav::dynamic_window_pure_pursuit::DynamicWindowBounds window;
  window.max_linear_vel = 1.0;
  window.min_linear_vel = -1.0;
  window.max_angular_vel = 1.0;
  window.min_angular_vel = -1.0;

  const auto [lin, ang] =
    easynav::dynamic_window_pure_pursuit::computeOptimalVelocityWithinDynamicWindow(
    window, /*curvature=*/0.0, /*sign=*/1.0);

  EXPECT_NEAR(lin, 1.0, 1e-9);
  EXPECT_NEAR(ang, 0.0, 1e-9);
}

class FriendRegulatedPurePursuitController : public easynav::RegulatedPurePursuitController
{
public:
  using easynav::RegulatedPurePursuitController::getLookAheadPoint;
  using easynav::RegulatedPurePursuitController::circleSegmentIntersection;
  using easynav::RegulatedPurePursuitController::toRobotFrame;
  using easynav::RegulatedPurePursuitController::remainingPathDistance;
  using easynav::RegulatedPurePursuitController::findClosestPoseIndex;
};

TEST(RegulatedPurePursuitControllerHelpers, LookAheadPointInterpolatesOnSegment)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p0, p1, p2;
  p0.pose.position.x = 0.0;
  p1.pose.position.x = 1.0;
  p2.pose.position.x = 2.0;
  path.poses = {p0, p1, p2};

  geometry_msgs::msg::Point robot;
  robot.x = 0.0;
  robot.y = 0.0;

  const auto carrot = FriendRegulatedPurePursuitController::getLookAheadPoint(
    path, robot, /*lookahead_dist=*/1.5);

  EXPECT_NEAR(carrot.x, 1.5, 1e-6);
  EXPECT_NEAR(carrot.y, 0.0, 1e-6);
}

TEST(RegulatedPurePursuitControllerHelpers, LookAheadPointIgnoresPathBehindRobot)
{
  // Regression test: the controller receives the full, un-pruned global path, so the carrot
  // search must start from the pose closest to the robot, not from the path's start pose. A path
  // start pose that is far behind the robot must never be picked up as the carrot.
  nav_msgs::msg::Path path;
  for (double x = 0.0; x <= 10.0; x += 0.1) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    path.poses.push_back(p);
  }

  // Robot has already travelled most of the path.
  geometry_msgs::msg::Point robot;
  robot.x = 9.0;
  robot.y = 0.0;

  const auto carrot = FriendRegulatedPurePursuitController::getLookAheadPoint(
    path, robot, /*lookahead_dist=*/0.5);

  // The carrot must be ahead of the robot (close to x=9.5), never back at the path's start (x=0).
  EXPECT_NEAR(carrot.x, 9.5, 1e-6);
  EXPECT_NEAR(carrot.y, 0.0, 1e-6);
}

TEST(RegulatedPurePursuitControllerHelpers, FindClosestPoseIndexPicksNearestNotFirst)
{
  nav_msgs::msg::Path path;
  for (double x = 0.0; x <= 10.0; x += 1.0) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    path.poses.push_back(p);
  }

  geometry_msgs::msg::Point robot;
  robot.x = 7.1;
  robot.y = 0.0;

  const auto idx = FriendRegulatedPurePursuitController::findClosestPoseIndex(path, robot);
  EXPECT_EQ(idx, 7u);
}

TEST(RegulatedPurePursuitControllerHelpers, LookAheadPointClampsToPathEnd)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p0, p1;
  p0.pose.position.x = 0.0;
  p1.pose.position.x = 1.0;
  path.poses = {p0, p1};

  geometry_msgs::msg::Point robot;
  robot.x = 0.0;
  robot.y = 0.0;

  const auto carrot = FriendRegulatedPurePursuitController::getLookAheadPoint(
    path, robot, /*lookahead_dist=*/5.0);

  EXPECT_NEAR(carrot.x, 1.0, 1e-6);
}

TEST(RegulatedPurePursuitControllerHelpers, ToRobotFrameRotatesAndTranslates)
{
  geometry_msgs::msg::Point global_point;
  global_point.x = 1.0;
  global_point.y = 1.0;

  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position.x = 1.0;
  robot_pose.position.y = 0.0;

  // Robot facing +90 degrees (looking along +y): the point straight ahead in the global
  // frame (0, 1) relative to the robot should map to local +x.
  const auto local = FriendRegulatedPurePursuitController::toRobotFrame(
    global_point, robot_pose, M_PI_2);

  EXPECT_NEAR(local.x, 1.0, 1e-6);
  EXPECT_NEAR(local.y, 0.0, 1e-6);
}

TEST(RegulatedPurePursuitControllerHelpers, RemainingPathDistanceFromClosestPoint)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p0, p1, p2;
  p0.pose.position.x = 0.0;
  p1.pose.position.x = 1.0;
  p2.pose.position.x = 2.0;
  path.poses = {p0, p1, p2};

  geometry_msgs::msg::Point robot;
  robot.x = 1.0;
  robot.y = 0.0;

  const double dist = FriendRegulatedPurePursuitController::remainingPathDistance(path, robot);
  EXPECT_NEAR(dist, 1.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
