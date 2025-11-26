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
    Eigen::Matrix2d Q,
    Eigen::Matrix2d R,
    Eigen::Matrix2d Rd,
    double qtheta,
    int N,
    double dt,
    const pcl::PointCloud<pcl::PointXYZ> & points);

  ~MPCParameters();

  Eigen::Vector2d goal;
  Eigen::Vector3d x0;
  Eigen::Vector3d theta0;
  Eigen::Matrix2d Q;
  Eigen::Matrix2d R;
  Eigen::Matrix2d Rd;
  double qtheta;
  int N;
  double dt;
  const pcl::PointCloud<pcl::PointXYZ> & points;
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
