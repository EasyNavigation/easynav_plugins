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

/// \brief SE(3) exponential, logarithm and adjoint.

#include <cmath>

#include "gtest/gtest.h"
#include "easynav_orbit_localizer/Lie.hpp"

using namespace easynav::orbit;  // NOLINT

TEST(Lie, ExpLogRoundTrip)
{
  for (const Vec6 & xi : {
    (Vec6() << 1.0, -2.0, 0.5, 0.1, -0.2, 0.3).finished(),
    (Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
    (Vec6() << 3.0, 1.0, 0.0, 0.0, 0.0, 2.5).finished(),
    (Vec6() << 0.01, 0.02, 0.03, 1e-5, 0.0, -1e-5).finished()})
  {
    const Vec6 back = logSE3(expSE3(xi));
    EXPECT_LT((back - xi).norm(), 1e-9) << xi.transpose();
  }
}

TEST(Lie, LogOfAHalfTurn)
{
  const Pose T = fromXYZYaw(1.0, 2.0, 0.0, M_PI - 1e-7);
  const Vec6 xi = logSE3(T);
  EXPECT_NEAR(std::abs(xi(5)), M_PI, 1e-5);
  EXPECT_LT((expSE3(xi).matrix() - T.matrix()).norm(), 1e-5);
}

TEST(Lie, AdjointMovesPerturbationsAcrossTheGroup)
{
  const Pose T = expSE3((Vec6() << 1.0, 0.5, -0.3, 0.2, 0.1, 1.0).finished());
  const Vec6 d = (Vec6() << 0.01, -0.02, 0.03, 0.004, 0.005, -0.006).finished();
  // T Exp(d) T^-1 == Exp(Ad(T) d)
  const Pose left = T * expSE3(d) * T.inverse();
  const Pose right = expSE3(adjoint(T) * d);
  EXPECT_LT((left.matrix() - right.matrix()).norm(), 1e-9);
}

TEST(Lie, YawAndRPY)
{
  const Pose T = fromXYZYaw(0.0, 0.0, 0.0, 2.0);
  EXPECT_NEAR(yawOf(T), 2.0, 1e-12);
  const Eigen::Matrix3d R = fromRPY(0.1, -0.2, 0.3);
  const Eigen::Vector3d rpy = rpyOf(R);
  EXPECT_NEAR(rpy.x(), 0.1, 1e-12);
  EXPECT_NEAR(rpy.y(), -0.2, 1e-12);
  EXPECT_NEAR(rpy.z(), 0.3, 1e-12);
  EXPECT_NEAR(wrapAngle(3.5 * M_PI), -0.5 * M_PI, 1e-12);
}

TEST(Lie, RightPerturbationIsInTheBodyFrame)
{
  // A pose heading +y, moved 1 m along its own x, moves +1 in y of the world
  const Pose T = fromXYZYaw(0.0, 0.0, 0.0, M_PI / 2.0);
  const Pose M = T * expSE3((Vec6() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  EXPECT_NEAR(M.translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(M.translation().y(), 1.0, 1e-12);
}
