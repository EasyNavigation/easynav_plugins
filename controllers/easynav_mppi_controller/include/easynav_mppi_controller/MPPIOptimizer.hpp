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
#ifndef EASYNAV_MPPI_CONTROLLER__MPPIOPTIMIZER_HPP_
#define EASYNAV_MPPI_CONTROLLER__MPPIOPTIMIZER_HPP_

#include <utility>
#include <vector>
#include <random>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace easynav
{

/// \brief Result structure for MPPI optimization containing control commands and trajectories.
struct MPPIResult
{
  double v; ///< Linear velocity command.
  double w; ///< Angular velocity command.
  std::vector<std::vector<std::pair<double, double>>> all_trajectories; ///< All sampled trajectories.
  std::vector<std::pair<double, double>> best_trajectory; ///< Best trajectory found during optimization.
};

struct TrajectorySample
{
  double v;     ///< Linear velocity for this sample.
  double w;     ///< Angular velocity for this sample.
  double cost;  ///< Cost associated with this trajectory sample.
};

class MPPIOptimizer
{
public:
  /// \brief Constructor for MPPIOptimizer.
  /// \param num_samples Number of samples to generate for MPPI.
  /// \param horizon_steps Number of steps in the prediction horizon.
  /// \param dt Time step for the simulation.
  /// \param lambda Temperature parameter for MPPI.
  /// \param max_lin_vel Maximum linear velocity in m/s.
  /// \param max_ang_vel Maximum angular velocity in rad/s.
  /// \param max_lin_acc Maximum linear acceleration in m/s^2.
  /// \param max_ang_acc Maximum angular acceleration in rad/s^2.
  /// \param fov Field of view in radians for trajectory sampling.
  /// \param safety_radius Safety radius for obstacle avoidance in meters.
  MPPIOptimizer(
    double num_samples, double horizon_steps, double dt, double lambda,
    double max_lin_vel = 1.0, double max_ang_vel = 1.0, double max_lin_acc = 1.0,
    double max_ang_acc = 1.0, double fov = M_PI / 2.0, double safety_radius = 0.6);

  /// \brief Computes the control commands using MPPI optimization.
  /// \param current_pose Current pose of the robot.
  /// \param path Planned path to follow.
  /// \param points Point cloud of the environment.
  /// \return MPPIResult containing the best control commands and trajectories.
  MPPIResult compute_control(
    const geometry_msgs::msg::Pose & current_pose,
    const nav_msgs::msg::Path & path,
    const pcl::PointCloud<pcl::PointXYZ> & points);

private:
  double num_samples_;    ///< Number of samples to generate for MPPI.
  double horizon_steps_;  ///< Number of steps in the prediction horizon.
  double dt_;             ///< Time step for the simulation.
  double lambda_;         ///< Temperature parameter for MPPI.
  double max_lin_vel_;    ///< Maximum linear velocity in m/s.
  double max_ang_vel_;    ///< Maximum angular velocity in rad/s.
  double max_lin_acc_;    ///< Maximum linear acceleration in m/s^2.
  double max_ang_acc_;    ///< Maximum angular acceleration in rad/s^2.
  double fov_;            ///< Field of view in radians for trajectory sampling.
  double safety_radius_;  ///< Safety radius for obstacle avoidance in meters.
  double last_v_ = 0.0;   ///< Last linear velocity command for smoothing.
  double last_w_ = 0.0;   ///< Last angular velocity command for smoothing.

  std::default_random_engine rng_; ///< Random number generator for sampling.
  std::normal_distribution<double> normal_ = std::normal_distribution<double>(0.0, 0.5);    ///< Normal distribution for noise in sampling.
  std::normal_distribution<double> v_noise_ = std::normal_distribution<double>(0.0, 0.05);  ///< Normal distribution for noise in linear velocity.
  std::normal_distribution<double> w_noise_ = std::normal_distribution<double>(0.0, 0.02);  ///< Normal distribution for noise in angular velocity.

  /// \brief Computes the cost of a trajectory based on its distance to the path and heading error.
  /// \param trajectory The trajectory to evaluate.
  /// \param path The planned path to follow.
  /// \param v Linear velocity of the trajectory.
  /// \param w Angular velocity of the trajectory.
  /// \param initial_yaw Initial yaw orientation of the robot.
  /// \param points The point cloud of the environment.
  /// \return The computed cost of the trajectory.
  double compute_cost(
    const std::vector<std::pair<double, double>> & trajectory,
    const nav_msgs::msg::Path & path,
    double v, double w, double initial_yaw,
    const pcl::PointCloud<pcl::PointXYZ> & points);

  /// \brief Simulates a trajectory based on initial position, orientation, and velocities.
  /// \param x Initial x position.
  /// \param y Initial y position.
  /// \param yaw Initial yaw orientation.
  /// \param v Linear velocity.
  /// \param w Angular velocity.
  /// \param path The planned path to follow.
  /// \param steps Number of steps to simulate.
  /// \return The simulated trajectory.
  std::vector<std::pair<double, double>> simulate_trajectory(
    double x, double y, double yaw,
    double v, double w, const nav_msgs::msg::Path & path, int steps);

  /// \brief Computes the heading error between the robot's current orientation and the target point.
  /// \param robot_yaw Current yaw orientation of the robot.
  /// \param target_x X coordinate of the target point.
  /// \param target_y Y coordinate of the target point.
  /// \param robot_x X coordinate of the robot's current position.
  /// \param robot_y Y coordinate of the robot's current position.
  /// \return The heading error in radians.
  double heading_error(
    double robot_yaw,
    double target_x, double target_y,
    double robot_x, double robot_y);

  /// \brief Computes the shortest angular distance between two angles.
  /// \param from Starting angle in radians.
  /// \param to Ending angle in radians.
  /// \return The shortest angular distance in radians.
  double shortest_angular_distance(double from, double to);

};

}  // namespace easynav

#endif  // EASYNAV_MPPI_CONTROLLER__MPPIOPTIMIZER_HPP_
