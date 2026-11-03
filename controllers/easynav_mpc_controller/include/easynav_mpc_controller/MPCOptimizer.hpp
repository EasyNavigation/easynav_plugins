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
/// \brief The MPC's optimization: a quadratic program over the horizon, linearized around the
/// previous solution, solved once per control cycle.

#ifndef EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_
#define EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_

#include <vector>

#include "Eigen/Core"

#include "easynav_mpc_controller/MotionModel.hpp"
#include "easynav_mpc_controller/Obstacles.hpp"
#include "easynav_mpc_controller/QPSolver.hpp"
#include "easynav_mpc_controller/Reference.hpp"

namespace easynav
{
namespace mpc
{

/// @brief Per velocity component (vx, [vy], [vz], wz): bounds and acceleration limits.
struct AxisLimits
{
  double min {0.0};
  double max {0.0};
  double acc {1.0};    ///< Speeding up (/s^2).
  double decel {1.0};  ///< Slowing down (/s^2).
};

struct MPCParams
{
  MotionModel model {MotionModel::Diff};
  int horizon_steps {20};
  double dt {0.1};
  /// First-order lag of the robot's velocity behind the command (s). 0: it follows at once.
  double time_constant {0.0};

  double weight_xy {10.0};       ///< Position error (per m^2).
  double weight_z {10.0};        ///< Height error (aerial).
  double weight_yaw {2.0};       ///< Heading error (per rad^2).
  double terminal_factor {5.0};  ///< Weight of the last pose, relative to the others.
  /// Pose weights grow linearly along the horizon from this factor (first step) to 1: below 1,
  /// the robot is not pressed to be on the path right now, only further ahead (less weaving).
  double near_weight {1.0};
  double weight_linear_vel {0.5};    ///< Linear velocity off the reference.
  double weight_angular_vel {0.2};   ///< Angular velocity off the reference.
  double weight_linear_rate {1.0};   ///< Change of the linear command between steps.
  double weight_angular_rate {1.0};  ///< Change of the angular command between steps.

  std::vector<AxisLimits> axes;  ///< One per velocity component (model's layout).
  double min_turning_radius {0.0};  ///< |wz| <= vx / radius (and vx >= 0) when > 0.

  double obstacle_range {3.0};    ///< Obstacles farther than this from a pose are ignored (m).
  int obstacles_per_step {2};     ///< Half-planes per pose, from different directions.
  double safety_margin {0.1};     ///< Clearance from the robot's shape to obstacles (m).

  QPSettings qp;
};

struct MPCInput
{
  Eigen::VectorXd pose;      ///< Model's pose layout.
  Eigen::VectorXd velocity;  ///< Robot's velocity now (body frame, model's layout).
  Eigen::VectorXd last_cmd;  ///< Command sent last cycle (rate cost of the first step).
  const ReferenceTrajectory * reference {nullptr};
  const Obstacles * obstacles {nullptr};
  const RobotShape * shape {nullptr};
};

struct MPCResult
{
  QPStatus status {QPStatus::Failed};
  Eigen::VectorXd cmd;                    ///< First command (valid if solved).
  std::vector<Eigen::VectorXd> commands;  ///< All of them.
  std::vector<Eigen::VectorXd> predicted;  ///< Poses 0..N they lead to.
  int variables {0};
  int constraints {0};
  double solve_ms {0.0};
};

/// @brief Builds and solves the horizon's QP. The deviations from a nominal trajectory (the
/// previous solution, shifted one step, rolled out from the robot's state) are the variables:
/// poses, velocities and commands at every step, tied by the linearized model.
class MPCOptimizer
{
public:
  void configure(const MPCParams & params);
  const MPCParams & params() const {return params_;}
  void set_axes(const std::vector<AxisLimits> & axes, double min_turning_radius);

  MPCResult compute(const MPCInput & input);

  /// Forget the previous solution (a new goal, a stop).
  void reset() {previous_.clear();}

private:
  MPCParams params_;
  ModelLayout layout_ {MotionModel::Diff};
  std::vector<Eigen::VectorXd> previous_;  ///< Commands of the last solution.
};

}  // namespace mpc
}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_
