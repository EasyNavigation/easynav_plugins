// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.

// #pragma once
#ifndef EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_
#define EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace easynav
{

class MPCParameters
{
public:

  MPCParameters(Eigen::Vector2d goal,
    Eigen::Vector3d x0,
    Eigen::Vector3d theta0,
    const pcl::PointCloud<pcl::PointXYZ> & points,
    int N,
    double dt);

  ~MPCParameters();

  Eigen::Vector2d goal;
  Eigen::Vector3d x0;
  Eigen::Vector3d theta0;
  const pcl::PointCloud<pcl::PointXYZ> & points;

  int get_steps();
  double get_timestep();
  double get_angular_tracking_cost();
  Eigen::Matrix2d get_effort_cost();
  Eigen::Matrix2d get_tracking_cost();
  Eigen::Matrix2d get_smooth_cost();

private:
  int N_ {5};
  double dt_ {0.1};
  Eigen::Matrix2d Q_ {{4.0, 0.0}, {0.0, 4.0}};    ///< Tracking Cost
  Eigen::Matrix2d R_ {{0.1, 0.0}, {0.0, 0.1}};    ///< Effort Cost
  Eigen::Matrix2d Rd_ {{0.1, 0.0}, {0.0, 0.1}};   ///< Smooth Cost
  double qtheta_ {3.0};

};

class MPCOptimizer
{
public:

  MPCOptimizer();

  /// \brief Destructor.
  ~MPCOptimizer();

  Eigen::Vector3d kinematic_model(const Eigen::Vector3d & x, 
    const Eigen::Vector3d & q, double v, double w, double dt);  

  double cost_function(const std::vector<double> & u, 
    std::vector<double> & grad, void *data);

  static double nlopt_cost_callback(const std::vector<double> & x, 
    std::vector<double> & grad, void *data);

};

struct NLoptCallbackData {
    MPCOptimizer *optimizer;
    MPCParameters *params;
  };

}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MPCOPTIMIZER_HPP_
