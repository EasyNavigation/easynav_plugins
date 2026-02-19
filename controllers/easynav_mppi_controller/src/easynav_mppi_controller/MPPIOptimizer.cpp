#include "easynav_mppi_controller/MPPIOptimizer.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

namespace easynav
{

MPPIOptimizer::MPPIOptimizer(
  double num_samples, double horizon_steps, double dt, double lambda,
  double max_lin_vel, double max_ang_vel, double max_lin_acc, double max_ang_acc,
  double fov, double safety_radius)
: num_samples_(num_samples), horizon_steps_(horizon_steps), dt_(dt), lambda_(lambda),
  max_lin_vel_(max_lin_vel), max_ang_vel_(max_ang_vel), max_lin_acc_(max_lin_acc),
  max_ang_acc_(max_ang_acc),
  fov_(fov), safety_radius_(safety_radius)
{
}

std::vector<std::pair<double, double>>
MPPIOptimizer::simulate_trajectory(
  double x, double y, double yaw, double v, double w,
  const nav_msgs::msg::Path & path, int steps)
{
  size_t horizon_steps = static_cast<size_t>(steps);
  std::vector<std::pair<double, double>> trajectory;
  trajectory.reserve(horizon_steps);

  bool has_path = !path.poses.empty();

  for (size_t i = 0; i < horizon_steps; ++i) {
    // MPPI noise sampling
    double v_sample = v + normal_(rng_);
    double w_sample = w + normal_(rng_);

    // Differential drive kinematics
    x += v_sample * std::cos(yaw) * dt_;
    y += v_sample * std::sin(yaw) * dt_;
    yaw += w_sample * dt_;

    if (has_path) {
      // Calculate the index in the path based on the current step
      size_t path_idx = std::min(static_cast<size_t>((i * path.poses.size()) / horizon_steps),
          path.poses.size() - 1);
      double target_x = path.poses[path_idx].pose.position.x;
      double target_y = path.poses[path_idx].pose.position.y;

      // Smooth motion towards the target
      double alpha = 0.2;
      x += alpha * (target_x - x);
      y += alpha * (target_y - y);
    }

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
  const pcl::PointCloud<pcl::PointXYZ> & points)
{
  // Total cost accumulator
  double cost = 0.0;
  size_t path_size = path.poses.size();

  // --- Path-Tracking Penalties ---
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto &[x, y] = trajectory[i];

    // Cost is distributed along the trajectory
    double path_alpha = static_cast<double>(i) / trajectory.size();
    size_t idx = std::min(static_cast<size_t>(path_alpha * path_size), path_size - 1);

    double dx = path.poses[idx].pose.position.x - x;
    double dy = path.poses[idx].pose.position.y - y;
    double dist = std::hypot(dx, dy);

    // Heading error is calculated based on the initial yaw
    double heading_penalty = heading_error(initial_yaw, path.poses[idx].pose.position.x,
                                               path.poses[idx].pose.position.y, x, y);

    // FOV penalty: discourage trajectories outside robot's view
    double angle_to_goal = heading_error(initial_yaw, trajectory.back().first,
                                             trajectory.back().second, x, y);
    double fov_penalty = std::pow(std::max(0.0, angle_to_goal - fov_), 2);

    // Accumulate penalties
    cost += dist;                // distance to path
    cost += 0.05 * heading_penalty;   // heading misalignment
    cost += 0.2 * fov_penalty;       // leaving field of view
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
      cost += 5000.0 * std::pow(safety_radius_ - min_obs_dist, 2) * (1.0 + v);
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

  // Encourage smooth motions
  double delta_v = v - last_v_;
  double delta_w = w - last_w_;
  cost += 0.1 * (delta_v * delta_v) + 0.1 * (delta_w * delta_w);

  // --- Regularization ---
  // Smooth motions: penalize high linear/angular velocities
  cost += 0.002 * (v * v) + 0.005 * (w * w);

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
  int last_pose = std::min(static_cast<size_t>(horizon_steps_), path.poses.size() - 1);
  int sim_steps = std::max(1, last_pose);
  const auto & path_pose = path.poses[last_pose].pose;
  double px = path_pose.position.x;
  double py = path_pose.position.y;
  double dist_to_goal = std::hypot(px - x, py - y);

  // Compute heading error to the goal
  double angle_to_goal = std::atan2(py - y, px - x);
  double angle_error = shortest_angular_distance(yaw, angle_to_goal);

  // Initialize sampling
  std::vector<TrajectorySample> samples;
  samples.reserve(static_cast<size_t>(num_samples_));

  std::vector<std::vector<std::pair<double, double>>> all_trajs;
  std::vector<std::pair<double, double>> best_traj;

  // Base velocity proportional to heading
  double angle_mag = std::abs(angle_error);

  // Scale linear velocity based on angular error
  double v_scale = 1.0;
  if (angle_mag > M_PI / 4.0) {          // more than 45°
    v_scale = 0.2;                       // move slowly
  } else if (angle_mag > M_PI / 8.0) {   // more than 22.5° and less than 45°
    v_scale = 0.5;                       // move moderately
  }

  // Base velocities
  // Scale linear velocity based on distance to goal: closer to goal, faster we go
  double base_v = max_lin_vel_ * std::min(dist_to_goal, 1.0) * v_scale;
  base_v = std::clamp(base_v, 0.0, max_lin_vel_);

  // Angular velocity is proportional to the heading error: turn faster if more misaligned
  double w_scale = std::min(1.0, 2.0 * angle_mag / M_PI);
  double base_w = std::clamp(w_scale * angle_error, -max_ang_vel_, max_ang_vel_);

  // Generate candidate trajectories
  for (int i = 0; i < num_samples_; ++i) {
    // Sample noise for velocities with Gaussian distribution and clamp
    double v = std::clamp(base_v + v_noise_(rng_), -max_lin_vel_, max_lin_vel_);
    double w = std::clamp(base_w + w_noise_(rng_), -max_ang_vel_, max_ang_vel_);

    // Simulate trajectory and compute its cost
    auto traj = simulate_trajectory(x, y, yaw, v, w, path, sim_steps);
    double cost = compute_cost(traj, path, v, w, yaw, points);
    all_trajs.push_back(traj);

    // Store the sample with its cost
    samples.push_back({v, w, cost});
  }

  // Softmin: Find minimum cost among samples
  auto best_sample_it = std::min_element(samples.begin(), samples.end(),
      [](const auto & a, const auto & b) {return a.cost < b.cost;});

  // Best trajectory and cost
  best_traj = all_trajs[std::distance(samples.begin(), best_sample_it)];
  double min_cost = best_sample_it->cost;

  // Adapt lambda (temperature) if velocities collapse to near zero
  double min_v_sample = std::numeric_limits<double>::max();
  for (const auto & s : samples) {
    min_v_sample = std::min(min_v_sample, s.v);
  }

  if (min_v_sample < 0.05) {
    lambda_ = std::min(5.0, lambda_ * 1.2); // increase lambda if tends to zero (stop)
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

  // Calculate the maximum change in velocities based on acceleration limits
  double max_v_delta = max_lin_acc_ * dt_;
  double max_w_delta = max_ang_acc_ * dt_;

  // Limit velocity changes (slew rate)
  vlin = last_v_ + std::clamp(vlin - last_v_, -max_v_delta, max_v_delta);
  vrot = last_w_ + std::clamp(vrot - last_w_, -max_w_delta, max_w_delta);

  // Smooth velocities
  double alpha_smooth = 0.2;  // lower is more smooth
  last_v_ = alpha_smooth * vlin + (1.0 - alpha_smooth) * last_v_;
  last_w_ = alpha_smooth * vrot + (1.0 - alpha_smooth) * last_w_;

  // Return final control command and trajectories
  return MPPIResult{last_v_, last_w_, all_trajs, best_traj};
}

}  // namespace easynav
