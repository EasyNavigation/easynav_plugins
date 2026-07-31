// Copyright 2025 Intelligent Robotics Lab
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

// #pragma once
#ifndef EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_
#define EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace easynav
{

/// \brief A MPC parameters class.
class MPCParameters
{
public:
  MPCParameters(
    Eigen::Vector2d goal,
    Eigen::Vector3d x0,
    Eigen::Vector3d theta0,
    const pcl::PointCloud<pcl::PointXYZ> & points,
    int N,
    double dt);

  /// \brief Destructor.
  ~MPCParameters();

  Eigen::Vector2d goal;                           ///< Goal pose (x,y) to optimizer.
  Eigen::Vector3d x0;                             ///< Init pos (x,y,z).
  Eigen::Vector3d theta0;                         ///< Init orientation (roll,pitch,yaw).
  const pcl::PointCloud<pcl::PointXYZ> & points;  ///< Filtered Point Cloud to detect collisions.

  /// \brief Get the number of horizont steps
  /// \return integer value of amount of horizont step used in optimization
  int get_steps();

  /// \brief Get the differential time used by integrator in kinematic model
  /// \return double value of time in seconds
  double get_timestep();

  /// \brief Get angular cost used in angle optimization
  /// \return double value of angular cost
  double get_angular_tracking_cost();

  /// \brief Get effort cost matrix used in optimization
  /// \return Eigen::Matrix2d as effort cost matrix
  Eigen::Matrix2d get_effort_cost();

  /// \brief Get tracking cost matrix used in optimization
  /// \return Eigen::Matrix2d as tracking cost matrix
  Eigen::Matrix2d get_tracking_cost();

  /// \brief Get smooth cost matrix used in optimization
  /// \return Eigen::Matrix2d as smooth cost matrix
  Eigen::Matrix2d get_smooth_cost();

private:
  int N_ {5};                                     ///< Horizont Step to optimize.
  double dt_ {0.1};                               ///< Differential time to integrate.
  Eigen::Matrix2d Q_ {{4.0, 0.0}, {0.0, 4.0}};    ///< Tracking Cost Matrix
  Eigen::Matrix2d R_ {{0.1, 0.0}, {0.0, 0.1}};    ///< Effort Cost Matrix
  Eigen::Matrix2d Rd_ {{0.1, 0.0}, {0.0, 0.1}};   ///< Smooth Cost Matrix
  double qtheta_ {3.0};                           ///< Angular cost value.

};

/// \brief A MPC Optimizer class.
class MPCOptimizer
{
public:
  MPCOptimizer();

  /// \brief Destructor.
  ~MPCOptimizer();

  /// \brief Kinematic model for a particle used by optmizer to stimate final position
  /// \param x Eigen::Vector3d Initial position for robot (x,y,z)
  /// \param q Eigen::Vector3d Initial angel for robot (roll, pitch , yaw)
  /// \param v double lineal velocity
  /// \param w double angular velocity
  /// \param dt double differential time used for integration
  /// \return Eigen::Vector3d Final state for robot (x,y,yaw)
  Eigen::Vector3d kinematic_model(
    const Eigen::Vector3d & x,
    const Eigen::Vector3d & q, double v, double w, double dt);

  /// \brief Wrap for real cost function
  /// \param u std::vector<double> Velocity vector to be optimized
  /// \param grad std::vector<double> gradient values for optimizer. It is NOT used for this implementation.
  /// \param data MPCParameter pointer with parameters used by optimizer
  /// \return double cost value used by nlOpt in internal callback
  double cost_function(const std::vector<double> & u, [[maybe_unused]]
    std::vector<double> & grad, void *data);

  /// \brief Real cost function with static propierties
  /// \param u std::vector<double> Velocity vector to be optimized
  /// \param grad std::vector<double> gradient values for optimizer. It is NOT used for this implementation.
  /// \param data MPCParameter pointer with parameters used by optimizer
  /// \return double cost value used by nlOpt in internal callback
  static double nlopt_cost_callback(
    const std::vector<double> & x,
    std::vector<double> & grad, void *data);

};

/// \brief Struct used as element in callback.
struct NLoptCallbackData
{
  MPCOptimizer *optimizer;                        ///< Pointer to optimizer.
  MPCParameters *params;                          ///< Pointer to parameter for optimizer.
};

}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_
