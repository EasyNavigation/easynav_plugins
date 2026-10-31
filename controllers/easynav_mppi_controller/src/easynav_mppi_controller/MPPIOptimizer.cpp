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

#include "easynav_mppi_controller/MPPIOptimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace easynav
{

namespace
{

/// Fraction of rollouts sampled around zero instead of the last solution (exploration).
constexpr double kExploreRatio = 0.1;
/// Local path beyond the reach of the rollouts (m).
constexpr double kLocalPathMargin = 1.0;
/// Path weights while an obstacle blocks the path (the rollouts must leave it).
constexpr double kBlockedPathScale = 0.1;

double normalize(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// One velocity axis towards \p target, within [lo, hi] and the acceleration limits.
double step_axis(
  double prev, double target, double lo, double hi, double acc, double decel, double dt)
{
  target = std::clamp(target, lo, hi);
  const bool slowing = target * prev < 0.0 || std::abs(target) < std::abs(prev);
  const double max_delta = (slowing ? decel : acc) * dt;
  return std::clamp(prev + std::clamp(target - prev, -max_delta, max_delta), lo, hi);
}

/// Where (vx, vy) (robot frame) is in the ellipse of the limits of \p p: beyond them if > 1.
/// An axis with no room (limit 0) is not part of it: see limit_translation().
double ellipse_norm(const MPPIParams & p, double vx, double vy)
{
  const double lim_x = vx >= 0.0 ? p.max_vx : -p.min_vx;
  const double nx = lim_x > 0.0 ? vx / lim_x : 0.0;
  const double ny = p.max_vy > 0.0 ? vy / p.max_vy : 0.0;
  return std::hypot(nx, ny);
}

bool finite_pose(const geometry_msgs::msg::Pose & p)
{
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
         std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
}

}  // namespace

bool parse_motion_model(const std::string & name, MotionModel & model)
{
  if (name == "diff") {
    model = MotionModel::Diff;
  } else if (name == "omni") {
    model = MotionModel::Omni;
  } else {
    return false;
  }
  return true;
}

bool parse_heading_mode(const std::string & name, HeadingMode & mode)
{
  if (name == "path") {
    mode = HeadingMode::Path;
  } else if (name == "free") {
    mode = HeadingMode::Free;
  } else if (name == "goal") {
    mode = HeadingMode::Goal;
  } else if (name == "hold") {
    mode = HeadingMode::Hold;
  } else if (name == "fixed") {
    mode = HeadingMode::Fixed;
  } else {
    return false;
  }
  return true;
}

void
MPPIOptimizer::configure(const MPPIParams & params)
{
  params_ = params;
  params_.num_samples = std::max(2, params_.num_samples);
  params_.horizon_steps = std::max(1, params_.horizon_steps);
  params_.dt = std::max(1e-3, params_.dt);
  params_.lambda = std::max(1e-6, params_.lambda);
  params_.grid_resolution = std::max(0.01, params_.grid_resolution);
  params_.noise_correlation = std::clamp(params_.noise_correlation, 0.0, 0.99);
  params_.robot_radius = std::max(0.0, params_.robot_radius);
  params_.safety_radius = std::max(params_.robot_radius, params_.safety_radius);
  params_.inscribed_radius = std::clamp(params_.inscribed_radius, 0.0, params_.robot_radius);
  if (params_.footprint.size() < 3) {
    params_.footprint.clear();
    params_.inscribed_radius = params_.robot_radius;
  }
  set_limits(params_);

  samples_ = params_.num_samples;
  steps_ = params_.horizon_steps;
  control_.assign(steps_, Velocity{});
  shifted_.assign(steps_, Velocity{});
  sampled_.assign(static_cast<size_t>(samples_) * steps_, Velocity{});
  poses_.assign(static_cast<size_t>(samples_) * (steps_ + 1), Pose2D{});
  costs_.assign(samples_, 0.0);
  optimal_.assign(steps_ + 1, Pose2D{});
  local_path_.reserve(4096);
  primitives_.reserve(64);

  const double half = reach() + params_.safety_radius + kLocalPathMargin;
  const int side = std::min(600, static_cast<int>(std::ceil(2.0 * half / params_.grid_resolution)));
  obstacle_grid_.reserve(side);
  path_grid_.reserve(side);

  rng_.seed(params_.seed);
  have_path_ = false;
  aligning_ = false;
  last_rollouts_ = 0;
}

void
MPPIOptimizer::set_limits(const MPPIParams & limits)
{
  params_.max_vx = std::max(0.0, limits.max_vx);
  params_.min_vx = std::min(0.0, limits.min_vx);
  params_.max_vy = params_.motion_model == MotionModel::Omni ? std::max(0.0, limits.max_vy) : 0.0;
  params_.max_wz = std::max(0.0, limits.max_wz);
  params_.acc_vx = limits.acc_vx;
  params_.decel_vx = limits.decel_vx;
  params_.acc_vy = limits.acc_vy;
  params_.decel_vy = limits.decel_vy;
  params_.acc_wz = limits.acc_wz;
  params_.decel_wz = limits.decel_wz;
}

void
MPPIOptimizer::reset()
{
  std::fill(control_.begin(), control_.end(), Velocity{});
}

double
MPPIOptimizer::reach() const
{
  const double speed = std::max({params_.max_vx, -params_.min_vx, params_.max_vy});
  return speed * params_.dt * params_.horizon_steps;
}

void
MPPIOptimizer::shift_control(double elapsed)
{
  if (!(elapsed > 0.0)) {
    return;
  }
  const double shift = std::min(elapsed / params_.dt, static_cast<double>(steps_));
  for (int t = 0; t < steps_; ++t) {
    const double tau = t + shift;
    const int i = static_cast<int>(std::floor(tau));
    if (i >= steps_ - 1) {
      shifted_[t] = control_[steps_ - 1];
      continue;
    }
    const double f = tau - i;
    const auto & a = control_[i];
    const auto & b = control_[i + 1];
    shifted_[t] = {a.vx + f * (b.vx - a.vx), a.vy + f * (b.vy - a.vy), a.wz + f * (b.wz - a.wz)};
  }
  control_.swap(shifted_);
}

void
MPPIOptimizer::update_local_path(const nav_msgs::msg::Path & path, const Pose2D & pose)
{
  const auto & poses = path.poses;
  const auto & front = poses.front().pose.position;
  const auto & back = poses.back().pose.position;
  const bool same_path = have_path_ && poses.size() == path_size_ &&
    front.x == path_front_x_ && front.y == path_front_y_ &&
    back.x == path_back_x_ && back.y == path_back_y_;

  auto dist2 = [&pose](const geometry_msgs::msg::Point & p) {
      return (p.x - pose.x) * (p.x - pose.x) + (p.y - pose.y) * (p.y - pose.y);
    };

  // The nearest pose: on a new path anywhere, else ahead of the last one (paths may cross).
  size_t first = 0;
  size_t last = poses.size();
  if (same_path) {
    first = std::min(closest_index_, poses.size() - 1);
    double length = 0.0;
    last = first + 1;
    const double window = reach() + kLocalPathMargin;
    while (last < poses.size() && length <= window) {
      length += std::hypot(
        poses[last].pose.position.x - poses[last - 1].pose.position.x,
        poses[last].pose.position.y - poses[last - 1].pose.position.y);
      ++last;
    }
  }
  size_t best = first;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = first; i < last; ++i) {
    const double d2 = dist2(poses[i].pose.position);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  closest_index_ = best;
  have_path_ = true;
  path_size_ = poses.size();
  path_front_x_ = front.x;
  path_front_y_ = front.y;
  path_back_x_ = back.x;
  path_back_y_ = back.y;

  // Local path: from the nearest pose, every grid cell, as far as the rollouts can go.
  local_path_.clear();
  const double step = 0.5 * params_.grid_resolution;
  const double length = reach() + params_.safety_radius + kLocalPathMargin;
  const double goal_yaw = tf2::getYaw(poses.back().pose.orientation);
  double s = 0.0;
  double yaw = goal_yaw;
  local_path_reaches_goal_ = true;
  for (size_t i = closest_index_; i + 1 < poses.size(); ++i) {
    const auto & a = poses[i].pose.position;
    const auto & b = poses[i + 1].pose.position;
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < 1e-6) {
      continue;
    }
    yaw = std::atan2(b.y - a.y, b.x - a.x);
    const int n = static_cast<int>(std::ceil(seg / step));
    for (int k = 0; k < n; ++k) {
      const double f = static_cast<double>(k) / n;
      local_path_.push_back({a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), yaw, s + f * seg, 0});
    }
    s += seg;
    if (s > length) {
      local_path_reaches_goal_ = false;
      local_path_.push_back({b.x, b.y, yaw, s, 0});
      break;
    }
  }
  if (local_path_reaches_goal_) {
    // A path of one pose (or of repeated ones) is the goal: its heading is the goal's.
    local_path_.push_back({back.x, back.y, local_path_.empty() ? goal_yaw : yaw, s, 0});
  }
  size_t ahead = 0;
  for (size_t i = 0; i < local_path_.size(); ++i) {
    ahead = std::max(ahead, i);
    while (ahead + 1 < local_path_.size() &&
      local_path_[ahead].s < local_path_[i].s + params_.heading_lookahead)
    {
      ++ahead;
    }
    local_path_[i].ahead = static_cast<int>(ahead);
  }
}

void
MPPIOptimizer::build_grids(const Pose2D & pose, const pcl::PointCloud<pcl::PointXYZ> * obstacles)
{
  const double half = reach() + params_.safety_radius + kLocalPathMargin;
  obstacle_grid_.reset(pose.x, pose.y, half, params_.grid_resolution);
  if (obstacles) {
    int id = 0;
    for (const auto & p : *obstacles) {
      obstacle_grid_.add_seed(id++, p.x, p.y);
    }
  }
  obstacle_grid_.propagate();
  if (!params_.footprint.empty()) {
    obstacle_buckets_.reset(pose.x, pose.y, half, 0.25);
    if (obstacles) {
      for (const auto & p : *obstacles) {
        obstacle_buckets_.add(p.x, p.y);
      }
    }
    obstacle_buckets_.build();
  }

  path_grid_.reset(pose.x, pose.y, half, params_.grid_resolution);
  path_blocked_ = false;
  const double horizon = reach() + params_.safety_radius;
  for (size_t i = 0; i < local_path_.size(); ++i) {
    const auto & sample = local_path_[i];
    path_grid_.add_seed(static_cast<int>(i), sample.x, sample.y);
    // Within reach, too close to an obstacle to follow (whatever the heading).
    if (!path_blocked_ && sample.s <= horizon &&
      obstacle_grid_.distance(sample.x, sample.y, horizon) < params_.inscribed_radius)
    {
      path_blocked_ = true;
    }
  }
  path_grid_.propagate();
}

double
MPPIOptimizer::rollout_cost(
  int k, const MPPIInput & input, const Pose2D & goal, bool explore, const Primitive * primitive)
{
  const auto & p = params_;
  const bool omni = p.motion_model == MotionModel::Omni;
  const double inv_steps = 1.0 / steps_;
  // Rollout 0 is the last solution as is, 1 brakes to a stop: always candidates.
  const bool noisy = k >= 2 && !primitive;
  const bool zero_mean = explore || k == 1;

  Velocity prev = input.velocity;
  if (!omni) {
    prev.vy = 0.0;
  }
  Pose2D pose = input.pose;
  const size_t base = static_cast<size_t>(k) * (steps_ + 1);
  poses_[base] = pose;

  // Already overlapping an obstacle (e.g. noise): only getting closer is a collision.
  const double collision_clearance = std::min(0.0, body_clearance(pose) - 1e-3);
  const double margin = std::max(1e-6, p.safety_radius - p.robot_radius);
  // An obstacle on the path: it is only a guide around it.
  const double path_weight = path_blocked_ ? kBlockedPathScale * p.path_weight : p.path_weight;
  const double heading_weight =
    path_blocked_ ? kBlockedPathScale * p.heading_weight : p.heading_weight;

  // Noise correlated in time (AR(1), same variance): smooth rollouts that really turn, not a
  // zigzag that averages out.
  const double a = p.noise_correlation;
  const double b = std::sqrt(1.0 - a * a);
  Velocity noise {
    p.vx_std * normal_(rng_), omni ? p.vy_std * normal_(rng_) : 0.0, p.wz_std * normal_(rng_)};

  double cost = 0.0;
  int t = 0;
  for (; t < steps_; ++t) {
    Velocity target = zero_mean ? Velocity{} : control_[t];
    if (primitive) {
      target = primitive->to_goal ? to_goal(pose, goal) :
        t < primitive->switch_step ? primitive->first : primitive->then;
    }
    if (noisy) {
      if (t > 0) {
        noise.vx = a * noise.vx + b * p.vx_std * normal_(rng_);
        noise.wz = a * noise.wz + b * p.wz_std * normal_(rng_);
        if (omni) {
          noise.vy = a * noise.vy + b * p.vy_std * normal_(rng_);
        }
      }
      target.vx += noise.vx;
      target.vy += noise.vy;
      target.wz += noise.wz;
    }
    // Omni sequences are in the map frame (turning does not change the course): limited in
    // the robot frame, where the robot's limits are.
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    if (omni) {
      target = {c * target.vx + s * target.vy, -s * target.vx + c * target.vy, target.wz};
      limit_translation(target);
    }
    Velocity v;
    v.vx = step_axis(prev.vx, target.vx, p.min_vx, p.max_vx, p.acc_vx, p.decel_vx, p.dt);
    v.vy = omni ? step_axis(prev.vy, target.vy, -p.max_vy, p.max_vy, p.acc_vy, p.decel_vy, p.dt) :
      0.0;
    v.wz = step_axis(prev.wz, target.wz, -p.max_wz, p.max_wz, p.acc_wz, p.decel_wz, p.dt);
    if (omni) {
      limit_translation(v);  // Rarely needed (a step between two allowed velocities): tiny.
    }

    double world_vx;
    double world_vy;
    if (omni) {
      world_vx = c * v.vx - s * v.vy;
      world_vy = s * v.vx + c * v.vy;
    } else {
      const double mid_yaw = pose.yaw + 0.5 * v.wz * p.dt;
      world_vx = v.vx * std::cos(mid_yaw);
      world_vy = v.vx * std::sin(mid_yaw);
    }
    sampled_[static_cast<size_t>(k) * steps_ + t] = omni ? Velocity{world_vx, world_vy, v.wz} : v;
    pose.x += world_vx * p.dt;
    pose.y += world_vy * p.dt;
    pose.yaw += v.wz * p.dt;
    poses_[base + t + 1] = pose;

    // Obstacles.
    const double clearance = body_clearance(pose);
    if (clearance < collision_clearance) {
      // Sooner is worse; the rest of the rollout is not driven.
      cost += p.collision_cost * (steps_ - t);
      break;
    }
    if (clearance < margin) {
      const double q = (margin - clearance) / margin;
      cost += p.obstacle_weight * q * q * inv_steps;
    }

    // Path: close to it, and as far along it (and as close to the goal) as soon as possible.
    const int nearest = path_grid_.nearest(pose.x, pose.y);
    const PathSample * sample = nearest >= 0 ? &local_path_[path_grid_.seed_id(nearest)] : nullptr;
    if (sample) {
      cost += path_weight * std::hypot(pose.x - sample->x, pose.y - sample->y) * inv_steps;
      cost += p.progress_weight * (local_path_.back().s - sample->s) * inv_steps;
    }
    if (local_path_reaches_goal_) {
      cost += p.goal_weight * std::hypot(goal.x - pose.x, goal.y - pose.y) * inv_steps;
    }

    // Heading: the mode's, giving way to the goal's near the goal.
    // A differential robot follows the path to the goal and turns there in place.
    double w = 1.0;
    if (omni && local_path_reaches_goal_ && p.goal_heading_distance > 0.0) {
      w = std::clamp(
        std::hypot(goal.x - pose.x, goal.y - pose.y) / p.goal_heading_distance, 0.0, 1.0);
    }
    switch (input.heading_mode) {
      case HeadingMode::Path:
        if (sample) {
          // Towards the path ahead (at a corner, already the next leg). On it (its end), none:
          // the goal's heading takes over.
          const auto & ahead = local_path_[sample->ahead];
          const double dx = ahead.x - pose.x;
          const double dy = ahead.y - pose.y;
          if (dx * dx + dy * dy > 0.01) {
            cost += heading_weight * w * std::abs(normalize(std::atan2(dy, dx) - pose.yaw)) *
              inv_steps;
          }
        }
        break;
      case HeadingMode::Goal:
        cost += p.heading_weight * w * std::abs(normalize(goal.yaw - pose.yaw)) * inv_steps;
        break;
      case HeadingMode::Hold:
      case HeadingMode::Fixed:
        cost += p.heading_weight * w * std::abs(normalize(input.heading_yaw - pose.yaw)) *
          inv_steps;
        break;
      case HeadingMode::Free:
        break;
    }
    cost += p.goal_heading_weight * (1.0 - w) * std::abs(normalize(goal.yaw - pose.yaw)) *
      inv_steps;

    if (!omni && v.vx < 0.0) {
      cost += p.forward_weight * -v.vx * inv_steps;
    }

    const double ax = (v.vx - prev.vx) / p.dt;
    const double ay = (v.vy - prev.vy) / p.dt;
    const double aw = (v.wz - prev.wz) / p.dt;
    cost += p.smoothness_weight * (ax * ax + ay * ay + aw * aw) * inv_steps;
    prev = v;
  }
  // A collided rollout stays where it collided (for the markers), stopped from then on.
  for (int r = t + 1; r < steps_; ++r) {
    poses_[base + r + 1] = pose;
    sampled_[static_cast<size_t>(k) * steps_ + r] = Velocity{};
  }
  return cost;
}

void
MPPIOptimizer::make_primitives(double yaw)
{
  const auto & p = params_;
  primitives_.clear();
  if (p.motion_model == MotionModel::Diff) {
    // Arcs, turns in place and reversing.
    for (double vx : {p.max_vx, 0.5 * p.max_vx, 0.0, p.min_vx}) {
      for (int i = -3; i <= 3; ++i) {
        if ((vx == 0.0 && i == 0) || (vx < 0.0 && p.min_vx == 0.0)) {
          continue;
        }
        const Velocity v {vx, 0.0, i * p.max_wz / 3.0};
        primitives_.push_back({v, v, steps_});
      }
    }
    // Turn in place, then go: around an obstacle right ahead.
    for (double wz : {-p.max_wz, p.max_wz}) {
      for (int switch_step : {steps_ / 4, steps_ / 2}) {
        for (double vx : {p.max_vx, 0.5 * p.max_vx}) {
          primitives_.push_back({{0.0, 0.0, wz}, {vx, 0.0, 0.0}, switch_step});
        }
      }
    }
    add_to_goal_primitive();
    return;
  }
  // Holonomic: straight lines in 8 directions (map frame), turning or not.
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (int d = 0; d < 8; ++d) {
    const double a = d * M_PI / 4.0;
    const double bx = std::cos(a);
    const double by = std::sin(a);
    // As fast as the robot's limits allow in that direction (robot frame).
    // Not in a direction the robot cannot move.
    if ((bx > 1e-9 && p.max_vx <= 0.0) || (bx < -1e-9 && p.min_vx >= 0.0) ||
      (std::abs(by) > 1e-9 && p.max_vy <= 0.0))
    {
      continue;
    }
    const double n = ellipse_norm(p, bx, by);
    if (!(n > 0.0)) {
      continue;
    }
    const double speed = 1.0 / n;
    for (double f : {1.0, 0.5}) {
      const double vx = f * speed * bx;
      const double vy = f * speed * by;
      for (double wz : {-0.5 * p.max_wz, 0.0, 0.5 * p.max_wz}) {
        const Velocity v {c * vx - s * vy, s * vx + c * vy, wz};
        primitives_.push_back({v, v, steps_});
      }
    }
  }
  for (double wz : {-p.max_wz, p.max_wz}) {
    const Velocity v {0.0, 0.0, wz};
    primitives_.push_back({v, v, steps_});
  }
  add_to_goal_primitive();
}

void
MPPIOptimizer::add_to_goal_primitive()
{
  // Stopping exactly at a goal a few cm away: no constant velocity or noisy rollout does it.
  // Only on the last stretch of the path: straight to the goal, it would cut its corners.
  if (local_path_reaches_goal_ && !local_path_.empty() &&
    local_path_.back().s <= std::max(0.5, params_.goal_heading_distance))
  {
    Primitive p {Velocity{}, Velocity{}, steps_, true};
    primitives_.push_back(p);
  }
}

Velocity
MPPIOptimizer::to_goal(const Pose2D & pose, const Pose2D & goal) const
{
  const auto & p = params_;
  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  const double d = std::hypot(dx, dy);
  if (d < 1e-3) {
    return {};
  }
  if (p.motion_model == MotionModel::Omni) {
    // Straight to it, turning to the goal's heading on the way.
    const double speed = std::min(std::max(p.max_vx, p.max_vy), std::sqrt(2.0 * p.decel_vx * d));
    return {speed * dx / d, speed * dy / d, 2.0 * normalize(goal.yaw - pose.yaw)};
  }
  // Differential: towards it, forward as it faces it.
  const double error = normalize(std::atan2(dy, dx) - pose.yaw);
  const double speed = std::min(p.max_vx, std::sqrt(2.0 * p.decel_vx * d));
  return {speed * std::max(0.0, std::cos(error)), 0.0, 2.0 * error};
}

double
MPPIOptimizer::body_clearance(const Pose2D & pose) const
{
  const auto & p = params_;
  const double d = obstacle_grid_.distance(pose.x, pose.y, std::numeric_limits<double>::infinity());
  if (p.footprint.empty()) {
    return d - p.robot_radius;
  }
  // Far enough, or surely overlapping, whatever the heading: no need to be exact.
  const double margin = p.safety_radius - p.robot_radius;
  if (d >= p.robot_radius + margin) {
    return d - p.robot_radius;
  }
  if (d < p.inscribed_radius) {
    return d - p.inscribed_radius;
  }
  // The polygon, turned with the robot, against the obstacles near it.
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double best = std::numeric_limits<double>::infinity();
  obstacle_buckets_.for_each_near(
    pose.x, pose.y, p.robot_radius + margin, [&](double x, double y) {
      const double dx = x - pose.x;
      const double dy = y - pose.y;
      const double lx = c * dx + s * dy;
      const double ly = -s * dx + c * dy;
      const double border = footprint_border_distance(p.footprint, lx, ly);
      best = std::min(best, footprint_contains(p.footprint, lx, ly) ? -border : border);
    });
  return std::min(best, d - p.inscribed_radius);  // The inscribed circle is inside the body.
}

void
MPPIOptimizer::limit_translation(Velocity & v) const
{
  if (params_.max_vy <= 0.0) {
    v.vy = 0.0;
  }
  if ((v.vx > 0.0 && params_.max_vx <= 0.0) || (v.vx < 0.0 && params_.min_vx >= 0.0)) {
    v.vx = 0.0;
  }
  const double n = ellipse_norm(params_, v.vx, v.vy);
  if (n > 1.0) {
    v.vx /= n;
    v.vy /= n;
  }
}

Velocity
MPPIOptimizer::align(double yaw_error) const
{
  // Fast enough to arrive, slow enough to brake within the deceleration limit.
  const double speed = std::clamp(
    std::sqrt(2.0 * params_.decel_wz * std::abs(yaw_error)),
    std::min(0.1, params_.max_wz), params_.max_wz);
  Velocity cmd;
  cmd.wz = std::copysign(speed, yaw_error);
  return cmd;
}

MPPIResult
MPPIOptimizer::compute(const MPPIInput & input)
{
  last_rollouts_ = 0;
  MPPIResult result;

  const bool valid_pose = std::isfinite(input.pose.x) && std::isfinite(input.pose.y) &&
    std::isfinite(input.pose.yaw);
  const bool valid_path = input.path && !input.path->poses.empty() &&
    std::all_of(
    input.path->poses.begin(), input.path->poses.end(),
    [](const geometry_msgs::msg::PoseStamped & p) {return finite_pose(p.pose);});
  if (!valid_pose || !valid_path) {
    reset();
    have_path_ = false;
    aligning_ = false;
    result.status = MPPIStatus::NoPath;
    return result;
  }

  const auto & goal_pose = input.path->poses.back().pose;
  const Pose2D goal {
    goal_pose.position.x, goal_pose.position.y, tf2::getYaw(goal_pose.orientation)};
  if (have_path_ && (goal.x != path_back_x_ || goal.y != path_back_y_)) {
    aligning_ = false;  // A new goal.
  }

  // At the goal position: turn to its heading. Enter at half the tolerance (or stopped within
  // it) and leave out of it, so localization noise at the border does not switch between both.
  const double dist_goal = std::hypot(goal.x - input.pose.x, goal.y - input.pose.y);
  const double yaw_error = normalize(goal.yaw - input.pose.yaw);
  // Also within the tolerance and (almost) stopped: no need to creep to the very goal.
  const bool stopped = std::hypot(input.velocity.vx, input.velocity.vy) < 0.05;
  if (dist_goal <= 0.5 * input.xy_goal_tolerance ||
    (dist_goal <= input.xy_goal_tolerance && stopped))
  {
    aligning_ = true;
  } else if (dist_goal > input.xy_goal_tolerance) {
    aligning_ = false;
  }
  if (dist_goal <= input.xy_goal_tolerance && std::abs(yaw_error) <= input.yaw_goal_tolerance) {
    reset();
    result.status = MPPIStatus::Arrived;
    return result;
  }
  if (aligning_) {
    reset();
    result.cmd = align(yaw_error);
    result.status = MPPIStatus::Aligning;
    return result;
  }

  MPPIInput in = input;
  if (params_.motion_model == MotionModel::Diff) {
    in.heading_mode = HeadingMode::Path;
  }

  update_local_path(*input.path, input.pose);
  build_grids(input.pose, input.obstacles);
  shift_control(input.elapsed);

  make_primitives(input.pose.yaw);
  const int primitives = std::min(static_cast<int>(primitives_.size()), samples_ - 2);
  const int explore_from = samples_ - static_cast<int>(kExploreRatio * samples_);
  double min_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < samples_; ++k) {
    const Primitive * primitive = k >= 2 && k < 2 + primitives ? &primitives_[k - 2] : nullptr;
    costs_[k] = rollout_cost(k, in, goal, k >= explore_from, primitive);
    min_cost = std::min(min_cost, costs_[k]);
  }
  last_rollouts_ = samples_;

  if (!(min_cost < params_.collision_cost)) {
    reset();
    std::fill(optimal_.begin(), optimal_.end(), input.pose);
    result.status = MPPIStatus::Blocked;
    return result;
  }

  // The best rollout, in case the mean is worse (below).
  const int best = static_cast<int>(
    std::min_element(costs_.begin(), costs_.end()) - costs_.begin());
  const Velocity * best_sequence = &sampled_[static_cast<size_t>(best) * steps_];
  std::copy(best_sequence, best_sequence + steps_, shifted_.begin());

  // Softmin weights, and the new solution: their weighted mean.
  double total = 0.0;
  for (auto & c : costs_) {
    c = std::exp(-(c - min_cost) / params_.lambda);
    total += c;
  }
  std::fill(control_.begin(), control_.end(), Velocity{});
  for (int k = 0; k < samples_; ++k) {
    const double w = costs_[k] / total;
    if (w < 1e-12) {
      continue;
    }
    const Velocity * v = &sampled_[static_cast<size_t>(k) * steps_];
    for (int t = 0; t < steps_; ++t) {
      control_[t].vx += w * v[t].vx;
      control_[t].vy += w * v[t].vy;
      control_[t].wz += w * v[t].wz;
    }
  }

  // Two equally good options (e.g. turning left or right) average to a bad one (not turning):
  // then the best rollout is the solution.
  if (rollout_cost(0, in, goal, false, nullptr) > min_cost + params_.lambda) {
    control_.swap(shifted_);
  }

  // The optimal trajectory, for visualization.
  const bool omni = params_.motion_model == MotionModel::Omni;
  Pose2D pose = input.pose;
  optimal_[0] = pose;
  for (int t = 0; t < steps_; ++t) {
    const auto & v = control_[t];
    if (omni) {
      pose.x += v.vx * params_.dt;
      pose.y += v.vy * params_.dt;
    } else {
      const double mid_yaw = pose.yaw + 0.5 * v.wz * params_.dt;
      pose.x += v.vx * std::cos(mid_yaw) * params_.dt;
      pose.y += v.vx * std::sin(mid_yaw) * params_.dt;
    }
    pose.yaw += v.wz * params_.dt;
    optimal_[t + 1] = pose;
  }

  // The first command, in the robot frame.
  result.cmd = control_[0];
  if (omni) {
    const double c = std::cos(input.pose.yaw);
    const double s = std::sin(input.pose.yaw);
    result.cmd = {c * control_[0].vx + s * control_[0].vy,
      -s * control_[0].vx + c * control_[0].vy, control_[0].wz};
    limit_translation(result.cmd);  // An axis without room exactly 0 (not a rounding error).
  } else {
    result.cmd.vy = 0.0;
  }
  result.status = MPPIStatus::Following;
  return result;
}

}  // namespace easynav
