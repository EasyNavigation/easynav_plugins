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
/// \brief Kinematic motion models of the MPC: the pose they reach with a body velocity, and its
/// linearization.

#ifndef EASYNAV_MPC_CONTROLLER__MOTIONMODEL_HPP_
#define EASYNAV_MPC_CONTROLLER__MOTIONMODEL_HPP_

#include <string>

#include "Eigen/Core"

namespace easynav
{
namespace mpc
{

/// @brief How the robot moves.
///
/// The pose is (x, y, yaw), or (x, y, z, yaw) for Aerial; the velocity is in the body frame:
/// Diff and Ackermann (vx, wz), Omni (vx, vy, wz), Aerial (vx, vy, vz, wz). Ackermann moves as Diff
/// (a kinematic bicycle at the rear axle); its turning radius is a constraint, not the model.
enum class MotionModel {Diff, Omni, Ackermann, Aerial};

/// @brief "diff", "omni", "ackermann" or "aerial" to \p model; false if it is none of them.
bool parse_motion_model(const std::string & name, MotionModel & model);

/// @brief Sizes and indices of the pose and the velocity of a motion model.
struct ModelLayout
{
  explicit ModelLayout(MotionModel model);

  MotionModel model;
  int np;      ///< Pose size: 3, or 4 for Aerial.
  int nv;      ///< Velocity size.
  int yaw;     ///< Index of yaw in the pose.
  int vy;      ///< Index of vy in the velocity, or -1.
  int vz;      ///< Index of vz in the velocity, or -1.
  int wz;      ///< Index of wz in the velocity.

  bool holonomic() const {return vy >= 0;}
  bool aerial() const {return vz >= 0;}
  /// Whether it turns in place (not an Ackermann robot).
  bool turns_in_place() const {return model != MotionModel::Ackermann;}
};

/// @brief The pose after \p dt seconds at body velocity \p v from \p p (midpoint heading).
Eigen::VectorXd step(
  const ModelLayout & layout, const Eigen::VectorXd & p, const Eigen::VectorXd & v, double dt);

/// @brief Jacobians of step() with respect to the pose (\p A, np x np) and the velocity (\p B,
/// np x nv), at (\p p, \p v).
void linearize(
  const ModelLayout & layout, const Eigen::VectorXd & p, const Eigen::VectorXd & v, double dt,
  Eigen::MatrixXd & A, Eigen::MatrixXd & B);

}  // namespace mpc
}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MOTIONMODEL_HPP_
