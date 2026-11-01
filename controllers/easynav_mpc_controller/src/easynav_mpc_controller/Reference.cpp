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

#include "easynav_mpc_controller/Reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace easynav
{
namespace mpc
{

namespace
{

constexpr double kNewPathWindow = 10.0;   // where a new path is searched for the robot (m)
constexpr double kForwardWindow = 10.0;   // how far ahead progress is searched (m)
constexpr double kBackwardWindow = 1.0;   // how far behind (an overshoot) (m)
constexpr double kProfileStep = 0.1;      // speed profile resolution (m)
constexpr double kCollisionStep = 0.25;   // obstacle check resolution along the path (m)

bool same_path(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b)
{
  if (a.poses.size() != b.poses.size() || a.poses.empty()) {
    return false;
  }
  for (std::size_t i : {std::size_t{0}, a.poses.size() / 2, a.poses.size() - 1}) {
    const auto & p = a.poses[i].pose.position;
    const auto & q = b.poses[i].pose.position;
    if (p.x != q.x || p.y != q.y || p.z != q.z) {
      return false;
    }
  }
  return true;
}

}  // namespace

double angle_diff(double a, double b)
{
  return std::remainder(a - b, 2.0 * M_PI);
}

Reference::Reference(MotionModel model)
: layout_(model)
{
}

void Reference::set_path(const nav_msgs::msg::Path & path)
{
  if (!pts_.empty() && same_path(path, last_path_)) {
    return;
  }
  last_path_ = path;
  pts_.clear();
  s_.clear();
  heading_.clear();
  slope_.clear();
  cum_heading_.clear();
  progress_ = -1.0;  // a new path: search its beginning
  speed_ = 0.0;

  for (const auto & ps : path.poses) {
    const auto & p = ps.pose.position;
    const Eigen::Vector3d q(p.x, p.y, layout_.aerial() ? p.z : 0.0);
    if (!q.allFinite()) {
      continue;
    }
    if (pts_.empty() || (q - pts_.back()).norm() > 1e-4) {
      pts_.push_back(q);
    }
  }
  if (pts_.size() < 2) {
    pts_.clear();
    return;
  }
  if (path_smoothing > 0.0) {
    smooth();
  }

  const std::size_t n = pts_.size();
  s_.assign(n, 0.0);
  heading_.assign(n, 0.0);
  slope_.assign(n, 0.0);
  double previous_heading = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Eigen::Vector3d d = pts_[i + 1] - pts_[i];
    const double ds = d.norm();
    s_[i + 1] = s_[i] + ds;
    slope_[i] = d.z() / ds;
    double h = d.head<2>().norm() > 1e-6 ? std::atan2(d.y(), d.x()) : previous_heading;
    if (std::isnan(h)) {
      h = 0.0;  // a vertical start: no heading yet
    } else if (!std::isnan(previous_heading)) {
      h = previous_heading + angle_diff(h, previous_heading);
    }
    heading_[i] = h;
    previous_heading = h;
  }
  heading_[n - 1] = heading_[n - 2];
  slope_[n - 1] = slope_[n - 2];

  cum_heading_.assign(n, 0.0);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    cum_heading_[i + 1] = cum_heading_[i] + heading_[i] * (s_[i + 1] - s_[i]);
  }
}

void Reference::smooth()
{
  // Evenly spaced points, then each one the mean of those within path_smoothing (a window that
  // narrows towards the ends, so that the start and the goal stay where they are)
  constexpr double kSpacing = 0.1;
  std::vector<Eigen::Vector3d> even {pts_.front()};
  double carry = 0.0;
  for (std::size_t i = 0; i + 1 < pts_.size(); ++i) {
    const Eigen::Vector3d d = pts_[i + 1] - pts_[i];
    const double len = d.norm();
    double t = kSpacing - carry;
    for (; t <= len; t += kSpacing) {
      even.push_back(pts_[i] + d * (t / len));
    }
    carry = len - (t - kSpacing);
  }
  if ((even.back() - pts_.back()).norm() > 1e-6) {
    even.push_back(pts_.back());
  }
  const int n = static_cast<int>(even.size());
  const int half = static_cast<int>(path_smoothing / kSpacing);
  if (n < 3 || half < 1) {
    pts_ = even;
    return;
  }
  // Only where the path turns faster than the robot can: the heading's change over a window
  // of half the smoothing (~0.7 R) above 1 rad (a radius below ~0.7 R) smooths fully, below
  // 0.5 rad not at all; that need is then averaged over the window, so it grows smoothly.
  std::vector<double> heading(n, 0.0);
  for (int i = 0; i + 1 < n; ++i) {
    const Eigen::Vector3d d = even[i + 1] - even[i];
    const double h = std::atan2(d.y(), d.x());
    heading[i] = i == 0 ? h : heading[i - 1] + angle_diff(h, heading[i - 1]);
  }
  heading[n - 1] = heading[n - 2];
  std::vector<double> need(n), need_prefix(n + 1, 0.0);
  const int q = std::max(1, half / 4);
  for (int i = 0; i < n; ++i) {
    const double turn = std::abs(heading[std::min(n - 1, i + q)] - heading[std::max(0, i - q)]);
    need[i] = std::clamp(2.0 * (turn - 0.5), 0.0, 1.0);
    need_prefix[i + 1] = need_prefix[i] + need[i];
  }
  std::vector<Eigen::Vector3d> prefix(n + 1, Eigen::Vector3d::Zero());
  for (int i = 0; i < n; ++i) {
    prefix[i + 1] = prefix[i] + even[i];
  }
  pts_.assign(n, Eigen::Vector3d::Zero());
  for (int i = 0; i < n; ++i) {
    const int a = std::max(0, i - half), b = std::min(n - 1, i + half);
    const double f = (need_prefix[b + 1] - need_prefix[a]) / (b - a + 1);
    const int w = std::min({static_cast<int>(std::lround(f * half)), i, n - 1 - i});
    pts_[i] = (prefix[i + w + 1] - prefix[i - w]) / (2 * w + 1);
  }
}

std::size_t Reference::segment(double s) const
{
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const std::size_t i = it == s_.begin() ? 0 : static_cast<std::size_t>(it - s_.begin()) - 1;
  return std::min(i, s_.size() - 2);
}

double Reference::interp(const std::vector<double> & values, double s) const
{
  s = std::clamp(s, 0.0, s_.back());
  const std::size_t i = segment(s);
  const double t = (s - s_[i]) / std::max(s_[i + 1] - s_[i], 1e-9);
  return values[i] + t * (values[i + 1] - values[i]);
}

Eigen::Vector3d Reference::point_at(double s) const
{
  s = std::clamp(s, 0.0, s_.back());
  const std::size_t i = segment(s);
  const double t = (s - s_[i]) / std::max(s_[i + 1] - s_[i], 1e-9);
  return pts_[i] + t * (pts_[i + 1] - pts_[i]);
}

double Reference::raw_heading(double s) const
{
  return heading_[segment(std::clamp(s, 0.0, s_.back()))];
}

double Reference::heading_at(double s) const
{
  // The mean of the segments' headings over a window centred on s (exact: their integral is
  // linear on each), narrower near the ends so that it stays centred
  s = std::clamp(s, 0.0, s_.back());
  const double half = std::min({curvature_window, s, s_.back() - s});
  if (half < 1e-3) {
    return raw_heading(s);
  }
  return (interp(cum_heading_, s + half) - interp(cum_heading_, s - half)) / (2.0 * half);
}

double Reference::curvature_at(double s) const
{
  // The change of the headings across the window (one-sided at the ends)
  s = std::clamp(s, 0.0, s_.back());
  const double half = std::max(std::min({curvature_window, s, s_.back() - s}), 0.05);
  const double a = std::max(0.0, s - half), b = std::min(s_.back(), s + half);
  if (b - a < 1e-6) {
    return 0.0;
  }
  return (raw_heading(b) - raw_heading(a)) / (b - a);
}

double Reference::project(const Eigen::Vector3d & position)
{
  const bool fresh = progress_ < 0.0;
  const double from = fresh ? 0.0 : progress_ - kBackwardWindow;
  const double to = fresh ? kNewPathWindow : progress_ + kForwardWindow;
  double best_s = fresh ? 0.0 : progress_;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = segment(std::max(0.0, from)); i + 1 < pts_.size() && s_[i] <= to; ++i) {
    Eigen::Vector3d a = pts_[i], b = pts_[i + 1], p = position;
    if (!layout_.aerial()) {
      a.z() = b.z() = p.z() = 0.0;
    }
    const Eigen::Vector3d e = b - a;
    const double t = std::clamp((p - a).dot(e) / std::max(e.squaredNorm(), 1e-12), 0.0, 1.0);
    const double d = (p - (a + t * e)).norm();
    if (d < best_d) {
      best_d = d;
      best_s = s_[i] + t * (s_[i + 1] - s_[i]);
    }
  }
  progress_ = best_s;
  return best_s;
}

double Reference::free_distance(
  double s0, double horizon, const Obstacles & obstacles, const RobotShape & shape,
  double margin) const
{
  for (double ds = 0.0; ds <= horizon; ds += kCollisionStep) {
    const double s = std::min(s0 + ds, s_.back());
    if (obstacles.collides(shape, point_at(s), heading_at(s), margin)) {
      return std::max(0.0, ds - kCollisionStep);
    }
    if (s >= s_.back()) {
      break;
    }
  }
  return std::numeric_limits<double>::infinity();
}

ReferenceTrajectory Reference::build(
  const Eigen::Vector3d & position, double yaw, double speed, int steps, double dt,
  const SpeedLimits & lim, const Obstacles * obstacles, const RobotShape & shape, double margin)
{
  ReferenceTrajectory ref;
  if (empty() || steps <= 0) {
    return ref;
  }
  ref.s0 = project(position);
  ref.remaining = s_.back() - ref.s0;

  // Path ahead that matters: the horizon at full speed, and the stop after it
  const double decel = std::max(lim.decel, 1e-3);
  const double reach = lim.max_vel * steps * dt + lim.max_vel * lim.max_vel / (2.0 * decel) + 1.0;
  ref.free_distance = obstacles && !obstacles->empty() ?
    free_distance(ref.s0, reach, *obstacles, shape, margin) :
    std::numeric_limits<double>::infinity();
  const double stop_at = ref.s0 + ref.free_distance;

  // Speed allowed at each sample of the path ahead, then braking back from what comes later
  const int samples = std::max(2, static_cast<int>(std::ceil(reach / kProfileStep)) + 1);
  std::vector<double> vlim(samples);
  for (int j = 0; j < samples; ++j) {
    const double s = ref.s0 + j * kProfileStep;
    double v = lim.max_vel;
    if (layout_.aerial()) {
      const double slope = std::abs(interp(slope_, s));
      const double horizontal = std::sqrt(std::max(0.0, 1.0 - slope * slope));
      if (horizontal > 1e-6) {v = std::min(v, lim.max_vel / horizontal);}
      if (slope > 1e-6) {v = std::min(v, lim.max_vertical_vel / slope);}
    }
    const double kappa = std::abs(curvature_at(s));
    if (kappa > 1e-6) {
      v = std::min({v, std::sqrt(lim.max_lateral_acc / kappa), lim.max_angular_vel / kappa});
    }
    const double to_goal = s_.back() - s;
    const double approach = to_goal > 0.0 ?
      std::max(std::sqrt(2.0 * decel * to_goal), lim.approach_speed) : lim.approach_speed;
    v = std::min(v, approach);
    v = std::min(v, std::sqrt(2.0 * decel * std::max(0.0, stop_at - s)));
    vlim[j] = std::max(0.0, v);
  }
  for (int j = samples - 2; j >= 0; --j) {
    vlim[j] = std::min(vlim[j], std::sqrt(vlim[j + 1] * vlim[j + 1] + 2.0 * decel * kProfileStep));
  }
  auto vlim_at = [&](double s) {
      const double j = std::clamp((s - ref.s0) / kProfileStep, 0.0, samples - 1.0);
      const int j0 = static_cast<int>(j);
      const int j1 = std::min(j0 + 1, samples - 1);
      return vlim[j0] + (j - j0) * (vlim[j1] - vlim[j0]);
    };

  // The reference heading continues from the robot's (the path's is unwrapped)
  const double yaw_offset = 2.0 * M_PI * std::round((yaw - heading_at(ref.s0)) / (2.0 * M_PI));
  auto pose_at = [&](double s) {
      Eigen::VectorXd p(layout_.np);
      const Eigen::Vector3d q = point_at(s);
      p(0) = q.x();
      p(1) = q.y();
      if (layout_.aerial()) {p(2) = q.z();}
      p(layout_.yaw) = heading_at(s) + yaw_offset;
      return p;
    };

  std::vector<double> s(steps + 1), v(steps + 1);
  s[0] = ref.s0;
  // From the faster of the robot and the reference's own speed, within what the path allows here
  v[0] = std::min(std::max(speed, speed_), std::max(vlim_at(ref.s0), speed));
  v[0] = std::max(v[0], 0.0);
  for (int k = 0; k < steps; ++k) {
    // The speed allowed where this step ends, not where it starts
    const double target = std::min(vlim_at(s[k] + v[k] * dt), v[k] + lim.acc * dt);
    v[k + 1] = std::max(target, v[k] - decel * dt);
    s[k + 1] = std::min({s[k] + 0.5 * (v[k] + v[k + 1]) * dt, s_.back(), stop_at});
    if (s[k + 1] >= stop_at) {
      v[k + 1] = 0.0;
    }
  }

  speed_ = v[1];
  ref.poses.reserve(steps + 1);
  for (int k = 0; k <= steps; ++k) {
    ref.poses.push_back(pose_at(s[k]));
  }
  ref.velocities.reserve(steps);
  for (int k = 0; k < steps; ++k) {
    Eigen::VectorXd u = Eigen::VectorXd::Zero(layout_.nv);
    const double ds = (s[k + 1] - s[k]) / dt;
    const double slope = layout_.aerial() ? interp(slope_, 0.5 * (s[k] + s[k + 1])) : 0.0;
    u(0) = ds * std::sqrt(std::max(0.0, 1.0 - slope * slope));
    if (layout_.aerial()) {u(layout_.vz) = ds * slope;}
    u(layout_.wz) = (ref.poses[k + 1](layout_.yaw) - ref.poses[k](layout_.yaw)) / dt;
    ref.velocities.push_back(u);
  }
  return ref;
}

}  // namespace mpc
}  // namespace easynav
