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
/// \brief Kinematic simulation of a robot driven by the MPPI optimizer, for the tests.

#ifndef CONTROLLERS__EASYNAV_MPPI_CONTROLLER__TESTS__MPPI_SIM_HPP_
#define CONTROLLERS__EASYNAV_MPPI_CONTROLLER__TESTS__MPPI_SIM_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2/LinearMath/Quaternion.hpp"

#include "easynav_mppi_controller/MPPIOptimizer.hpp"

namespace mppi_test
{

using easynav::HeadingMode;
using easynav::MotionModel;
using easynav::MPPIParams;
using easynav::MPPIStatus;
using easynav::Pose2D;
using easynav::Velocity;

inline double normalize(double a) {return std::atan2(std::sin(a), std::cos(a));}

inline geometry_msgs::msg::PoseStamped pose_stamped(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  return p;
}

/// A path through \p corners every 5 cm, facing along it; the last pose faces \p goal_yaw.
inline nav_msgs::msg::Path polyline(
  const std::vector<std::pair<double, double>> & corners, double goal_yaw)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (size_t i = 0; i + 1 < corners.size(); ++i) {
    const auto [ax, ay] = corners[i];
    const auto [bx, by] = corners[i + 1];
    const double len = std::hypot(bx - ax, by - ay);
    const double yaw = std::atan2(by - ay, bx - ax);
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.05)));
    for (int k = 0; k < n; ++k) {
      const double f = static_cast<double>(k) / n;
      path.poses.push_back(pose_stamped(ax + f * (bx - ax), ay + f * (by - ay), yaw));
    }
  }
  const auto [gx, gy] = corners.back();
  path.poses.push_back(pose_stamped(gx, gy, goal_yaw));
  return path;
}

/// Distance from (x, y) to the polyline.
inline double distance_to(
  const std::vector<std::pair<double, double>> & corners, double x, double y)
{
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < corners.size(); ++i) {
    const auto [ax, ay] = corners[i];
    const auto [bx, by] = corners[i + 1];
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    const double f = len2 > 0.0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0) :
      0.0;
    best = std::min(best, std::hypot(x - (ax + f * dx), y - (ay + f * dy)));
  }
  return best;
}

/// Points on a circle of radius \p r around (cx, cy).
inline void add_disc(pcl::PointCloud<pcl::PointXYZ> & cloud, double cx, double cy, double r)
{
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * M_PI * i / 24;
    cloud.push_back(pcl::PointXYZ(cx + r * std::cos(a), cy + r * std::sin(a), 0.2));
  }
  cloud.push_back(pcl::PointXYZ(cx, cy, 0.2));
}

/// Points from (ax, ay) to (bx, by), every 5 cm.
inline void add_wall(
  pcl::PointCloud<pcl::PointXYZ> & cloud, double ax, double ay, double bx, double by)
{
  const int n = std::max(1, static_cast<int>(std::hypot(bx - ax, by - ay) / 0.05));
  for (int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    cloud.push_back(pcl::PointXYZ(ax + f * (bx - ax), ay + f * (by - ay), 0.2));
  }
}

/// A rectangle \p length x \p width centered on the robot frame, and its radii, into \p p.
inline void set_rectangle(MPPIParams & p, double length, double width)
{
  const double hx = length / 2.0;
  const double hy = width / 2.0;
  p.footprint = {{hx, hy}, {-hx, hy}, {-hx, -hy}, {hx, -hy}};
  p.robot_radius = std::hypot(hx, hy);
  p.inscribed_radius = std::min(hx, hy);
  p.safety_radius = p.robot_radius + 0.1;
}

/// Limits of a differential robot like the defaults of RobotLimits.
inline MPPIParams diff_params()
{
  auto p = MPPIParams::defaults_for(MotionModel::Diff);
  p.max_vx = 0.5;
  p.min_vx = -0.2;
  p.max_wz = 1.0;
  p.acc_vx = 0.5;
  p.decel_vx = 1.0;
  p.acc_wz = 1.0;
  p.decel_wz = 2.0;
  p.robot_radius = 0.3;
  p.safety_radius = 0.6;
  return p;
}

/// An omnidirectional robot, same speed in every direction.
inline MPPIParams omni_params()
{
  const auto diff = diff_params();
  auto p = MPPIParams::defaults_for(MotionModel::Omni);
  p.max_vx = diff.max_vx;
  p.max_wz = diff.max_wz;
  p.acc_vx = diff.acc_vx;
  p.decel_vx = diff.decel_vx;
  p.acc_wz = diff.acc_wz;
  p.decel_wz = diff.decel_wz;
  p.robot_radius = diff.robot_radius;
  p.safety_radius = diff.safety_radius;
  p.min_vx = -0.5;
  p.max_vy = 0.5;
  p.acc_vy = 0.5;
  p.decel_vy = 1.0;
  return p;
}

/// A robot following the optimizer's commands through a velocity smoother (as ControllerNode).
struct Sim
{
  easynav::MPPIOptimizer optimizer;
  MPPIParams params;
  Pose2D pose;
  Velocity velocity;   ///< Of the robot (smoothed).
  Velocity last_cmd;   ///< Last command (the controller's input).
  nav_msgs::msg::Path path;
  pcl::PointCloud<pcl::PointXYZ> obstacles;
  HeadingMode heading_mode {HeadingMode::Path};
  double heading_yaw {0.0};
  double xy_tolerance {0.1};
  double yaw_tolerance {0.1};
  double period {1.0 / 30.0};  ///< Control period (s).
  double time {0.0};

  // What happened.
  std::vector<Pose2D> trace;
  std::vector<Velocity> cmds;
  std::vector<MPPIStatus> statuses;
  double min_clearance {std::numeric_limits<double>::infinity()};  ///< Of the body (< 0: hit).
  bool limits_ok {true};
  bool acceleration_ok {true};

  explicit Sim(const MPPIParams & p, Pose2D start = {})
  : params(p), pose(start)
  {
    optimizer.configure(params);
  }

  static double step_axis(double v, double target, double acc, double decel, double dt)
  {
    const bool slowing = target * v < 0.0 || std::abs(target) < std::abs(v);
    const double max_delta = (slowing ? decel : acc) * dt;
    double next = v + std::clamp(target - v, -max_delta, max_delta);
    if ((v > 0.0 && next < 0.0) || (v < 0.0 && next > 0.0)) {
      next = 0.0;
    }
    return next;
  }

  MPPIStatus step()
  {
    easynav::MPPIInput in;
    in.pose = pose;
    in.velocity = last_cmd;
    in.path = &path;
    in.obstacles = &obstacles;
    in.heading_mode = heading_mode;
    in.heading_yaw = heading_yaw;
    in.elapsed = time > 0.0 ? period : 0.0;
    in.xy_goal_tolerance = xy_tolerance;
    in.yaw_goal_tolerance = yaw_tolerance;
    const auto r = optimizer.compute(in);
    const auto & p = optimizer.params();
    const auto & c = r.cmd;
    constexpr double eps = 1e-9;
    if (c.vx > p.max_vx + eps || c.vx < p.min_vx - eps || std::abs(c.vy) > p.max_vy + eps ||
      std::abs(c.wz) > p.max_wz + eps)
    {
      limits_ok = false;
    }
    // Holonomic: no direction faster than the configured ones.
    if (p.motion_model == MotionModel::Omni) {
      const double lx = c.vx >= 0.0 ? p.max_vx : -p.min_vx;
      if (lx > 0.0 && p.max_vy > 0.0 && std::hypot(c.vx / lx, c.vy / p.max_vy) > 1.0 + 1e-6) {
        limits_ok = false;
      }
    }
    if (r.status == MPPIStatus::Following) {
      // Omni: a step may be projected back into the ellipse of the speed limits, a bit more.
      const double projection = p.motion_model == MotionModel::Omni ? 2e-3 : 0.0;
      const double dv = std::max(p.acc_vx, p.decel_vx) * p.dt + eps + projection;
      const double dy = std::max(p.acc_vy, p.decel_vy) * p.dt + eps + projection;
      const double dw = std::max(p.acc_wz, p.decel_wz) * p.dt + eps;
      if (std::abs(c.vx - last_cmd.vx) > dv || std::abs(c.vy - last_cmd.vy) > dy ||
        std::abs(c.wz - last_cmd.wz) > dw)
      {
        acceleration_ok = false;
      }
    }
    last_cmd = c;
    cmds.push_back(c);
    statuses.push_back(r.status);

    // The robot: smoothed velocity, integrated.
    velocity.vx = step_axis(velocity.vx, c.vx, p.acc_vx, p.decel_vx, period);
    velocity.vy = step_axis(velocity.vy, c.vy, p.acc_vy, p.decel_vy, period);
    velocity.wz = step_axis(velocity.wz, c.wz, p.acc_wz, p.decel_wz, period);
    const double yaw = pose.yaw + 0.5 * velocity.wz * period;
    pose.x += (velocity.vx * std::cos(yaw) - velocity.vy * std::sin(yaw)) * period;
    pose.y += (velocity.vx * std::sin(yaw) + velocity.vy * std::cos(yaw)) * period;
    pose.yaw = normalize(pose.yaw + velocity.wz * period);
    time += period;
    trace.push_back(pose);

    // Clearance of the robot's body (its footprint, or its circle).
    const auto & fp = optimizer.params().footprint;
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    for (const auto & o : obstacles) {
      double clearance;
      if (fp.empty()) {
        clearance = std::hypot(o.x - pose.x, o.y - pose.y) - optimizer.params().robot_radius;
      } else {
        const double lx = cos_yaw * (o.x - pose.x) + sin_yaw * (o.y - pose.y);
        const double ly = -sin_yaw * (o.x - pose.x) + cos_yaw * (o.y - pose.y);
        const double border = easynav::footprint_border_distance(fp, lx, ly);
        clearance = easynav::footprint_contains(fp, lx, ly) ? -border : border;
      }
      min_clearance = std::min(min_clearance, clearance);
    }
    return r.status;
  }

  /// Runs until arrived (true) or \p max_time.
  bool run(double max_time)
  {
    while (time < max_time) {
      if (step() == MPPIStatus::Arrived) {
        return true;
      }
    }
    return false;
  }

  double max_abs_vy() const
  {
    double m = 0.0;
    for (const auto & c : cmds) {m = std::max(m, std::abs(c.vy));}
    return m;
  }
};

}  // namespace mppi_test

#endif  // CONTROLLERS__EASYNAV_MPPI_CONTROLLER__TESTS__MPPI_SIM_HPP_
