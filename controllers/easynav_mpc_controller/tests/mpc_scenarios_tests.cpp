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
/// \brief The MPC in closed loop with a simulated robot of each motion model: it reaches the goal
/// along the path, within the robot's limits, around and before obstacles, in a control cycle.

#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "mpc_sim.hpp"

using Eigen::Vector3d;

namespace
{

std::vector<rclcpp::Parameter> limits(
  double v, double w, double acc, double decel, std::vector<rclcpp::Parameter> extra = {})
{
  std::vector<rclcpp::Parameter> p {
    {"robot_limits.max_linear_vel", v},
    {"robot_limits.min_linear_vel", 0.0},
    {"robot_limits.max_angular_vel", w},
    {"robot_limits.max_linear_acc", acc},
    {"robot_limits.max_linear_decel", decel},
    {"robot_limits.max_angular_acc", 3.0},
    {"robot_limits.max_angular_decel", 6.0},
  };
  p.insert(p.end(), extra.begin(), extra.end());
  return p;
}

// 33 ms: a 30 Hz control cycle
constexpr double kCycleMs = 1000.0 / 30.0;

void print(const std::string & name, const mpc_sim::Result & r)
{
  std::printf(
    "%-24s %s t=%5.1fs dev_mean=%.3f dev_max=%.3f v_max=%.2f solve_max=%.1fms fails=%d "
    "clearance=%.2f curv_excess=%.4f final=(%.2f, %.2f, %.2f)\n",
    name.c_str(), r.reached ? "REACHED" : "NOT", r.time, r.dev_mean, r.dev_max, r.v_max,
    r.max_solve_ms, r.qp_failures, r.min_clearance, r.max_curvature_excess, r.final_position.x(),
    r.final_position.y(), r.final_position.z());
}

}  // namespace

class MpcScenarios : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(MpcScenarios, DiffStraight)
{
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {8, 0, 0}};
  sc.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("diff straight", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.02);
  EXPECT_GT(r.v_max, 0.95) << "at full speed";
  EXPECT_LT(r.time, 12.0);
  EXPECT_LT(r.max_solve_ms, kCycleMs);
  EXPECT_EQ(r.qp_failures, 0);
}

TEST_F(MpcScenarios, DiffLCorner)
{
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {5, 0, 0}, {5, 5, 0}};
  sc.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("diff L", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.35) << "the corner is rounded, not overshot";
  EXPECT_LT(r.max_solve_ms, kCycleMs);
}

TEST_F(MpcScenarios, DiffUTurn)
{
  // Out and back 1.5 m apart: the old MPC aimed at the way back and never arrived
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {5, 0, 0}, {5, 1.5, 0}, {0, 1.5, 0}};
  sc.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("diff U-turn", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.5);
  EXPECT_LT(r.max_solve_ms, kCycleMs);
}

TEST_F(MpcScenarios, DiffGoalBehindTurnsInPlaceFirst)
{
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {-4, 0, 0}};
  sc.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("diff behind", r);
  EXPECT_TRUE(r.reached);
  EXPECT_GT(r.max_rotation_in_place, 0.5) << "it turned around in place";
  EXPECT_LT(r.dev_max, 0.15) << "and did not go off the path to turn";
}

TEST_F(MpcScenarios, DiffWithActuatorLag)
{
  // The robot's velocity lags the command 0.3 s, and the MPC knows it
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {5, 0, 0}, {5, 5, 0}};
  sc.lag = 0.3;
  sc.params = limits(1.0, 1.5, 0.8, 1.5, {{"mpc.time_constant", 0.3}});
  const auto r = mpc_sim::run(sc);
  print("diff lag", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.45);
}

TEST_F(MpcScenarios, OmniFollowsAPathToItsSide)
{
  mpc_sim::Scenario sc;
  sc.model = "omni";
  sc.path = {{0, 0, 0}, {0, 3, 0}, {3, 3, 0}};
  sc.params = limits(
    1.0, 1.5, 0.8, 1.5,
    {{"robot_limits.max_lateral_vel", 0.8}, {"robot_limits.max_lateral_acc", 0.8},
      {"robot_limits.max_lateral_decel", 1.5}});
  const auto r = mpc_sim::run(sc);
  print("omni", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.3);
  EXPECT_LT(r.max_rotation_in_place, 1e-9) << "no turning in place: it moves sideways";
}

TEST_F(MpcScenarios, AckermannCurveAtSpeedWithinItsSteering)
{
  // A car (rear axle at the frame, 2.8 m wheelbase, 4.5 m turning radius) at 6 m/s through
  // a 90 deg turn of radius 10 m
  mpc_sim::Scenario sc;
  sc.model = "ackermann";
  sc.path.push_back({0, 0, 0});
  for (int i = 0; i <= 30; ++i) {
    const double a = i * (M_PI / 2.0) / 30.0;
    sc.path.push_back({20.0 + 10.0 * std::sin(a), 10.0 - 10.0 * std::cos(a), 0});
  }
  sc.path.push_back({30, 40, 0});
  sc.geometry = {3.83, 1.0, 1.5, {{3.69, 1.0}, {-0.93, 1.0}, {-0.93, -1.0}, {3.69, -1.0}}};
  sc.goal_tolerance = 1.0;
  sc.timeout = 60.0;
  sc.params = limits(
    6.0, 1.0, 2.0, 4.0,
    {{"robot_limits.min_turning_radius", 4.5}, {"mpc.horizon_steps", 30}});
  const auto r = mpc_sim::run(sc);
  print("ackermann curve", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.6);
  EXPECT_GT(r.v_max, 5.0) << "fast on the straights";
  EXPECT_LT(r.max_curvature_excess, 1e-6) << "never a curvature the steering cannot give";
  EXPECT_LT(r.max_rotation_in_place, 1e-9) << "a car does not turn in place";
  EXPECT_LT(r.max_solve_ms, kCycleMs);
}

TEST_F(MpcScenarios, AckermannSharpCornerStaysCloseWithinItsSteering)
{
  // A planner's 90 deg corner, sharper than the car can turn: it rounds it
  mpc_sim::Scenario sc;
  sc.model = "ackermann";
  sc.path = {{0, 0, 0}, {25, 0, 0}, {25, 25, 0}};
  sc.geometry = {3.83, 1.0, 1.5, {{3.69, 1.0}, {-0.93, 1.0}, {-0.93, -1.0}, {3.69, -1.0}}};
  sc.goal_tolerance = 1.0;
  sc.timeout = 60.0;
  sc.params = limits(
    6.0, 1.0, 2.0, 4.0,
    {{"robot_limits.min_turning_radius", 4.5}, {"mpc.horizon_steps", 30}});
  const auto r = mpc_sim::run(sc);
  print("ackermann corner", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 2.5);
  EXPECT_LT(r.max_curvature_excess, 1e-6);
}

TEST_F(MpcScenarios, AerialClimbsAndTurns)
{
  mpc_sim::Scenario sc;
  sc.model = "aerial";
  sc.start = {0, 0, 2};
  sc.path = {{0, 0, 2}, {10, 0, 6}, {10, 10, 6}};
  sc.geometry = {0.5, 0.5, 0.3};
  sc.params = limits(
    3.0, 1.5, 1.5, 2.5,
    {{"robot_limits.max_lateral_vel", 3.0}, {"robot_limits.max_vertical_vel", 1.0},
      {"robot_limits.max_vertical_acc", 1.0}, {"robot_limits.max_vertical_decel", 1.5}});
  const auto r = mpc_sim::run(sc);
  print("aerial", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.dev_max, 0.5);
  EXPECT_NEAR(r.final_position.z(), 6.0, 0.25);
  EXPECT_LT(r.max_solve_ms, kCycleMs);
}

TEST_F(MpcScenarios, StopsBeforeAnObstacleOnThePath)
{
  // Nobody replans: the robot must stop short of it
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {10, 0, 0}};
  sc.obstacles = {{5.0, 0.0, 0.2}, {5.0, 0.1, 0.2}, {5.0, -0.1, 0.2}};
  sc.timeout = 20.0;
  sc.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("obstacle on path", r);
  EXPECT_FALSE(r.reached);
  EXPECT_GT(r.min_clearance, 0.0) << "never touched";
  EXPECT_GT(r.final_position.x(), 3.5) << "but got close";
}

TEST_F(MpcScenarios, KeepsItsDistanceFromAnObstacleBesideThePath)
{
  // A point 0.45 m from the path: the robot (0.3 m) keeps its 0.1 m margin and goes on
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {10, 0, 0}};
  sc.obstacles = {{5.0, 0.45, 0.2}};
  sc.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("obstacle beside", r);
  EXPECT_TRUE(r.reached);
  EXPECT_GT(r.min_clearance, 0.05);
}

TEST_F(MpcScenarios, ItsOwnBodySeenByItsSensorsIsNotAnObstacle)
{
  // A ground robot and an aerial one whose sensors see their own edges
  mpc_sim::Scenario ground;
  ground.path = {{0, 0, 0}, {4, 0, 0}};
  ground.body_points = {{0.28, 0.0, 0.2}, {-0.25, 0.1, 0.2}};
  ground.params = limits(1.0, 1.5, 0.8, 1.5);
  const auto g = mpc_sim::run(ground);
  print("own body (ground)", g);
  EXPECT_TRUE(g.reached);

  mpc_sim::Scenario air;
  air.model = "aerial";
  air.start = {0, 0, 5};
  air.path = {{0, 0, 5}, {6, 0, 7}};
  air.geometry = {0.5, 0.5, 0.3};
  air.body_points = {{0.45, 0.0, 0.05}, {0.0, -0.44, -0.05}};
  air.params = limits(
    2.0, 1.5, 1.5, 2.5,
    {{"robot_limits.max_lateral_vel", 2.0}, {"robot_limits.max_vertical_vel", 1.0},
      {"robot_limits.max_vertical_acc", 1.0}, {"robot_limits.max_vertical_decel", 1.5}});
  const auto a = mpc_sim::run(air);
  print("own body (aerial)", a);
  EXPECT_TRUE(a.reached);
}

TEST_F(MpcScenarios, ALowerSpeedLimitAppliesAtOnce)
{
  // Robot limits of 0.3 m/s: the MPC never asks for more
  mpc_sim::Scenario sc;
  sc.path = {{0, 0, 0}, {3, 0, 0}};
  sc.params = limits(0.3, 1.5, 0.8, 1.5);
  const auto r = mpc_sim::run(sc);
  print("slow", r);
  EXPECT_TRUE(r.reached);
  EXPECT_LE(r.v_max, 0.3 + 1e-6);
}
