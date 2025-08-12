#include "easynav_mppi_controller/MPPIOptimizer.hpp"
#include "tf2/utils.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

namespace easynav
{

MPPIOptimizer::MPPIOptimizer(
  double num_samples, double horizon_steps, double dt, double lambda,
  double max_lin_vel, double max_ang_vel, double fov)
: num_samples_(num_samples), horizon_steps_(horizon_steps), dt_(dt), lambda_(lambda),
  max_lin_vel_(max_lin_vel), max_ang_vel_(max_ang_vel), fov_(fov)
{
}

std::vector<std::pair<double, double>>
MPPIOptimizer::simulate_trajectory(double x, double y, double yaw, double v, double w)
{
  std::vector<std::pair<double, double>> trajectory;
  trajectory.reserve(static_cast<size_t>(horizon_steps_));
  // Simulate trajectory: generate points based on velocity and angular velocity
  for (int i = 0; i < static_cast<int>(horizon_steps_); ++i) {
    x += v * std::cos(yaw) * dt_;
    y += v * std::sin(yaw) * dt_;
    yaw += w * dt_;
    trajectory.emplace_back(x, y);
  }

  return trajectory;
}

double MPPIOptimizer::heading_error(
  double robot_yaw,
  double target_x, double target_y,
  double robot_x, double robot_y)
{
  double target_yaw = std::atan2(target_y - robot_y, target_x - robot_x);
  double err = std::atan2(std::sin(target_yaw - robot_yaw), std::cos(target_yaw - robot_yaw));
  return std::abs(err);
}

double MPPIOptimizer::shortest_angular_distance(double from, double to)
{
  double result = std::fmod(to - from + M_PI, 2.0 * M_PI);
  if (result < 0) {result += 2.0 * M_PI;}
  return result - M_PI;
}

double MPPIOptimizer::compute_cost(
  const std::vector<std::pair<double, double>> & trajectory,
  const nav_msgs::msg::Path & path,
  double v, double w,
  double initial_yaw)
{
  // Total cost for the trajectory
  double cost = 0.0;

  // --- Penalties ---
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto &[x, y] = trajectory[i];

    // 1. Distance penalty: get the minimum distance to the path
    double min_dist = std::numeric_limits<double>::max();
    // 2. Heading penalty: how well the trajectory aligns with the path
    double heading_penalty = 0.0;
    // 3. FOV penalty: how well the trajectory stays within the robot's field of view
    double fov_penalty = 0.0;

    for (const auto & pose_stamped : path.poses) {
      double dx = pose_stamped.pose.position.x - x;
      double dy = pose_stamped.pose.position.y - y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;

        heading_penalty = heading_error(
          initial_yaw, pose_stamped.pose.position.x, pose_stamped.pose.position.y, x, y);
      }
    }

    double angle_to_target = heading_error(initial_yaw, trajectory.back().first,
        trajectory.back().second, x, y);
    fov_penalty = 0.5 * std::pow(std::max(0.0, angle_to_target - fov_), 2);

    cost += min_dist;                // Distance
    cost += 0.1 * heading_penalty;   // Heading
    cost += 0.5 * fov_penalty;       // FOV
  }

  // --- Regularization ---
  // Penalize high speeds to avoid sudden accelerations
  // and keep the robot within a reasonable speed range
  // This helps prevent abrupt movements and improves the robot's stability
  cost += 0.002 * (v * v + w * w);

  return cost;
}

MPPIResult MPPIOptimizer::compute_control(
  const geometry_msgs::msg::Pose & current_pose,
  const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return MPPIResult{0.0, 0.0, {}, {}};
  }

  // Initial conditions
  double x = current_pose.position.x;
  double y = current_pose.position.y;
  double yaw = tf2::getYaw(current_pose.orientation);

  // Compute the angle to the goal pose based on the horizon steps
  // If horizon_steps_ is larger than the path size, use the last pose
  // This ensures that we always have a valid goal pose to align with
  const auto & goal_pose = path.poses[std::min(static_cast<size_t>(horizon_steps_),
      path.poses.size() - 1)].pose;
  double gx = goal_pose.position.x;
  double gy = goal_pose.position.y;
  double angle_to_goal = std::atan2(gy - y, gx - x);
  double angle_error = shortest_angular_distance(yaw, angle_to_goal);

  // Check if the robot is aligned with the goal
  if (std::abs(angle_error) > fov_ / 2.0) {
    // Rotate to align with the goal
    double w = std::clamp(angle_error, -max_ang_vel_, max_ang_vel_);
    return MPPIResult{0.0, w, {}, {}};
  }

  // If aligned, proceed with MPPI sampling
  std::vector<TrajectorySample> samples;
  samples.reserve(static_cast<size_t>(num_samples_));

  std::vector<std::vector<std::pair<double, double>>> all_trajs;
  std::vector<std::pair<double, double>> best_traj;
  double best_cost = std::numeric_limits<double>::max();

  // Base velocity and angular velocity
  double base_v = 0.6;
  double base_w = 0.1;

  // Generate samples
  for (int i = 0; i < num_samples_; ++i) {
    // Add Gaussian noise to the base velocities
    double v = std::max(0.0, base_v + normal_(rng_));
    // Add Gaussian noise to the base angular velocities
    double w = base_w + normal_(rng_);

    // Ensure velocities are within limits
    // Clamp linear velocity to max_lin_vel_ and angular velocity to max_ang_vel_
    v = std::clamp(v, -max_lin_vel_, max_lin_vel_);
    w = std::clamp(w, -max_ang_vel_, max_ang_vel_);

    // Simulate the trajectory with the given velocities
    auto traj = simulate_trajectory(x, y, yaw, v, w);
    all_trajs.push_back(traj);

    // Compute the cost of the trajectory
    double cost = compute_cost(traj, path, v, w, yaw);

    // Update the best trajectory
    if (cost < best_cost) {
      best_cost = cost;
      best_traj = traj;
    }

    // Store the sample with its cost
    samples.push_back({v, w, cost});
  }

  // Sort samples by cost: Softmin
  double min_cost = std::min_element(samples.begin(), samples.end(),
      [](const auto & a, const auto & b) {return a.cost < b.cost;})->cost;

  // Compute the mean and variance of the costs
  double mean_cost = 0.0;
  for (const auto & s : samples) {
    mean_cost += s.cost;
  }
  mean_cost /= samples.size();

  double var_cost = 0.0;
  for (const auto & s : samples) {
    var_cost += std::pow(s.cost - mean_cost, 2);
  }
  var_cost /= samples.size();

  // Compute lambda dynamically based on the variance of the costs
  // This allows the algorithm to adapt to the variability of the costs
  // and adjust the exploration-exploitation trade-off
  lambda_ = std::clamp(var_cost, 1.0, 5.0);

  // Normalize the costs
  double denom = 0.0;
  for (auto & sample : samples) {
    sample.cost = std::exp(-1.0 / lambda_ * (sample.cost - min_cost));
    denom += sample.cost;
  }

  // Compute the average linear and angular velocities based on the normalized costs
  // This step combines the samples into a single control command
  // weighted by their costs, allowing the algorithm to focus on the most promising trajectories
  double vlin = 0.0;
  double vrot = 0.0;
  for (const auto & sample : samples) {
    vlin += sample.v * sample.cost / denom;
    vrot += sample.w * sample.cost / denom;
  }

  // Ensure velocities are within limits
  vlin = std::clamp(vlin, -max_lin_vel_, max_lin_vel_);
  vrot = std::clamp(vrot, -max_ang_vel_, max_ang_vel_);

  // Predict the trajectory based on the velocities
  best_traj = simulate_trajectory(x, y, yaw, vlin, vrot);
  all_trajs.push_back(best_traj);

  // Return the control command
  return MPPIResult{vlin, vrot, all_trajs, best_traj};
}

}  // namespace easynav
