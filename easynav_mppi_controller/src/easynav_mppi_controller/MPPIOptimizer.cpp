#include "easynav_mppi_controller/MPPIOptimizer.hpp"
#include "tf2/utils.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

namespace easynav
{

MPPIOptimizer::MPPIOptimizer(
  double num_samples, double horizon_steps, double dt, double lambda,
  double max_lin_vel, double max_ang_vel, double fov, double safety_radius)
: num_samples_(num_samples), horizon_steps_(horizon_steps), dt_(dt), lambda_(lambda),
  max_lin_vel_(max_lin_vel), max_ang_vel_(max_ang_vel), fov_(fov), safety_radius_(safety_radius)
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
  double v, double w, double initial_yaw,
  const pcl::PointCloud<pcl::PointXYZ> & points
)
{
  // Total cost accumulator
  double cost = 0.0;

  // --- Path-Tracking Penalties ---
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto &[x, y] = trajectory[i];

    double min_dist = std::numeric_limits<double>::max();
    double heading_penalty = 0.0;
    double fov_penalty = 0.0;

    // Distance to path: encourage staying close to planned path
    for (const auto & pose_stamped : path.poses) {
      double dx = pose_stamped.pose.position.x - x;
      double dy = pose_stamped.pose.position.y - y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;
        // Heading alignment with nearest path point
        heading_penalty = heading_error(
          initial_yaw, pose_stamped.pose.position.x, pose_stamped.pose.position.y, x, y);
      }
    }

    // FOV penalty: discourage trajectories outside robot's view
    double angle_to_target = heading_error(initial_yaw, trajectory.back().first,
        trajectory.back().second, x, y);
    fov_penalty = 0.5 * std::pow(std::max(0.0, angle_to_target - fov_), 2);

    // Accumulate penalties
    cost += min_dist;                // distance to path
    cost += 0.1 * heading_penalty;   // heading misalignment
    cost += 0.5 * fov_penalty;       // leaving field of view
  }

  // --- Goal Progress Penalties ---
  const auto & goal_pose = path.poses.back().pose;
  const double gx = goal_pose.position.x;
  const double gy = goal_pose.position.y;

  const auto & start_xy = trajectory.front();
  const auto & end_xy = trajectory.back();

  double d_start_goal = std::hypot(gx - start_xy.first, gy - start_xy.second);
  double d_end_goal = std::hypot(gx - end_xy.first, gy - end_xy.second);

  // how much closer we got to goal
  double progress = d_start_goal - d_end_goal;

  cost += -2.0 * progress;   // reward moving closer to goal
  cost += 1.5 * d_end_goal;  // penalize being far from goal at end


  // --- Obstacle Avoidance Penalties ---
  double min_obs_overall = std::numeric_limits<double>::max();

  for (const auto & point : points) {
    double min_obs_dist = std::numeric_limits<double>::max();
    for (const auto &[x, y] : trajectory) {
      double dx = point.x - x;
      double dy = point.y - y;
      double dist = std::hypot(dx, dy);
      if (dist < min_obs_dist) {min_obs_dist = dist;}
    }
    min_obs_overall = std::min(min_obs_overall, min_obs_dist);

    // Safety margin (robot radius + margin)
    if (min_obs_dist < safety_radius_) {
      // Heavy penalty for collision risk
      cost += 1000.0 * (safety_radius_ - min_obs_dist);
    } else {
      // Small penalty: encourage keeping clearance
      cost += 1.0 / (min_obs_dist * min_obs_dist);
    }
  }

  // --- Velocity Encouragement ---
  // If obstacles are far, discourage staying too slow
  if (min_obs_overall > 0.6) {
    cost += 0.2 / std::max(0.05, v);
  }

  // --- Regularization ---
  // Smooth motions: penalize high linear/angular velocities
  cost += 0.005 * (v * v) + 0.01 * (w * w);

  return cost;
}

MPPIResult MPPIOptimizer::compute_control(
  const geometry_msgs::msg::Pose & current_pose,
  const nav_msgs::msg::Path & path,
  const pcl::PointCloud<pcl::PointXYZ> & points)
{
  // If the path is empty, stop the robot
  if (path.poses.empty()) {
    return MPPIResult{0.0, 0.0, {}, {}};
  }

  // Current robot state
  double x = current_pose.position.x;
  double y = current_pose.position.y;
  double yaw = tf2::getYaw(current_pose.orientation);

  // Select goal pose (within horizon, or last path point)
  const auto & goal_pose = path.poses[std::min(static_cast<size_t>(horizon_steps_),
      path.poses.size() - 1)].pose;
  double gx = goal_pose.position.x;
  double gy = goal_pose.position.y;

  // Compute heading error to the goal
  double angle_to_goal = std::atan2(gy - y, gx - x);
  double angle_error = shortest_angular_distance(yaw, angle_to_goal);

  // If not facing the goal, rotate in place to align
  if (std::abs(angle_error) > fov_ / 2.0) {
    double w = std::clamp(angle_error, -max_ang_vel_, max_ang_vel_);
    return MPPIResult{0.0, w, {}, {}};
  }

  // Initialize sampling
  std::vector<TrajectorySample> samples;
  samples.reserve(static_cast<size_t>(num_samples_));

  std::vector<std::vector<std::pair<double, double>>> all_trajs;
  std::vector<std::pair<double, double>> best_traj;

  // Base velocities: forward motion reduced if misaligned
  double base_v = 0.6 * std::cos(angle_error);
  base_v = std::max(0.2, base_v);
  double base_w = 0.1;

  // Generate candidate trajectories
  for (int i = 0; i < num_samples_; ++i) {
    // Sample linear/angular velocities with Gaussian noise
    double v = std::max(0.0, base_v + normal_(rng_));
    double w = base_w + normal_(rng_);

    // Clamp velocities to allowed limits
    v = std::clamp(v, -max_lin_vel_, max_lin_vel_);
    w = std::clamp(w, -max_ang_vel_, max_ang_vel_);

    // Simulate trajectory and compute its cost
    auto traj = simulate_trajectory(x, y, yaw, v, w);
    double cost = compute_cost(traj, path, v, w, yaw, points);
    all_trajs.push_back(traj);

    // Store the sample with its cost
    samples.push_back({v, w, cost});
  }

  // Softmin: Find minimum cost among samples
  double min_cost = std::min_element(samples.begin(), samples.end(),
      [](const auto & a, const auto & b) {return a.cost < b.cost;})->cost;

  // Adapt lambda (temperature) if velocities collapse to near zero
  double min_v_sample = std::numeric_limits<double>::max();
  for (const auto & s : samples) {
    min_v_sample = std::min(min_v_sample, s.v);
  }

  if (min_v_sample < 0.1) {
    lambda_ = std::min(8.0, lambda_ * 1.5); // increase lambda if tends to zero (stop)
  }

  // Softmin weighting of samples
  double denom = 0.0;
  for (auto & sample : samples) {
    sample.cost = std::exp(-1.0 / lambda_ * (sample.cost - min_cost));
    denom += sample.cost;
  }

  // Weighted average of velocities
  double vlin = 0.0, vrot = 0.0;
  for (const auto & sample : samples) {
    vlin += sample.v * sample.cost / denom;
    vrot += sample.w * sample.cost / denom;
  }

  // Clamp to velocity limits
  vlin = std::clamp(vlin, -max_lin_vel_, max_lin_vel_);
  vrot = std::clamp(vrot, -max_ang_vel_, max_ang_vel_);

  // Best trajectory from averaged control
  best_traj = simulate_trajectory(x, y, yaw, vlin, vrot);

  // Obstacle clearance check
  double clearance = std::numeric_limits<double>::max();
  for (const auto & p : points) {
    for (const auto & xy : best_traj) {
      double dx = p.x - xy.first;
      double dy = p.y - xy.second;
      clearance = std::min(clearance, std::hypot(dx, dy));
    }
  }

  // If clearance is safe and goal is in FOV, ensure minimum forward speed
  if (clearance > 0.6 && std::abs(angle_error) < fov_) {
    vlin = std::max(vlin, 0.2);
    best_traj = simulate_trajectory(x, y, yaw, vlin, vrot);
  }

  all_trajs.push_back(best_traj);

  // Return final control command and trajectories
  return MPPIResult{vlin, vrot, all_trajs, best_traj};
}

}  // namespace easynav
