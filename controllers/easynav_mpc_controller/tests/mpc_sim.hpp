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
/// \brief Closed loop of the MPC with a simulated robot: the controller in a ControllerNode
/// (robot limits), the robot moving as its kinematics say (an Ackermann car steers within its
/// limit, the velocity may lag the command), and what happened.

#ifndef CONTROLLERS__EASYNAV_MPC_CONTROLLER__TESTS__MPC_SIM_HPP_
#define CONTROLLERS__EASYNAV_MPC_CONTROLLER__TESTS__MPC_SIM_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_mpc_controller/MPCController.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

namespace mpc_sim
{

struct Scenario
{
  std::string model {"diff"};
  std::vector<Eigen::Vector3d> path;       ///< Polyline (z only for aerial).
  Eigen::Vector3d start {0.0, 0.0, 0.0};
  double yaw {0.0};
  std::vector<rclcpp::Parameter> params;   ///< robot_limits.* and mpc.*
  easynav::RobotGeometry geometry {0.3, 0.3, 0.5};
  std::vector<Eigen::Vector3d> obstacles;  ///< Map frame.
  std::vector<Eigen::Vector3d> body_points;  ///< Robot frame: its own body, seen by its sensors.
  double timeout {40.0};                   ///< Simulated seconds.
  double goal_tolerance {0.2};
  double lag {0.0};                        ///< Robot's velocity time constant (s).
  double wheelbase {2.8};                  ///< Ackermann.
  bool verbose {false};                    ///< Print the loop (debugging).
};

struct Result
{
  bool reached {false};
  double time {0.0};
  double dev_mean {0.0};
  double dev_max {0.0};
  double v_max {0.0};
  double max_curvature_excess {-std::numeric_limits<double>::infinity()};  ///< |w| - v / R
  double max_solve_ms {0.0};
  int qp_failures {0};
  double min_clearance {std::numeric_limits<double>::infinity()};  ///< To any obstacle point.
  Eigen::Vector3d final_position {0.0, 0.0, 0.0};
  double final_yaw {0.0};
  double max_rotation_in_place {0.0};  ///< Largest |w| while |v| ~ 0.
  double max_linear_jump {0.0};        ///< Largest change of the linear command per cycle.
};

inline nav_msgs::msg::Path polyline(const std::vector<Eigen::Vector3d> & pts, double res = 0.05)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const int n = std::max(1, static_cast<int>((pts[i] - pts[i - 1]).norm() / res));
    for (int k = (i == 1 ? 0 : 1); k <= n; ++k) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = path.header.frame_id;
      const Eigen::Vector3d q = pts[i - 1] + (pts[i] - pts[i - 1]) * k / n;
      p.pose.position.x = q.x();
      p.pose.position.y = q.y();
      p.pose.position.z = q.z();
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
  }
  return path;
}

inline double distance_to(const Eigen::Vector3d & p, const std::vector<Eigen::Vector3d> & pts)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const Eigen::Vector3d e = pts[i] - pts[i - 1];
    const double t = std::clamp(
      (p - pts[i - 1]).dot(e) / std::max(e.squaredNorm(), 1e-12), 0.0, 1.0);
    best = std::min(best, (p - (pts[i - 1] + t * e)).norm());
  }
  return best;
}

inline Result run(const Scenario & sc)
{
  static int instance = 0;
  const bool aerial = sc.model == "aerial";
  const bool ackermann = sc.model == "ackermann";
  easynav::RobotGeometryRegistry::getInstance()->set_geometry(sc.geometry);

  std::vector<rclcpp::Parameter> params = sc.params;
  params.emplace_back("mpc.motion_model", sc.model);
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r", "__node:=mpc_sim_" + std::to_string(instance++)})
    .parameter_overrides(params));
  easynav::MPCController mpc;
  mpc.initialize(node, "mpc");
  const auto limits = node->get_robot_limits();

  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
  easynav::NavState nav_state;
  std::vector<Eigen::Vector3d> path_pts = sc.path;
  if (!aerial) {
    for (auto & p : path_pts) {
      p.z() = 0.0;
    }
  }
  nav_state.set("path", polyline(path_pts));
  nav_state.set("goal_tolerance.position", sc.goal_tolerance);
  nav_state.set("goal_tolerance.yaw", 2.0 * M_PI);

  const Eigen::Vector3d goal = path_pts.back();
  Eigen::Vector3d pos = sc.start;
  if (!aerial) {pos.z() = 0.0;}
  double yaw = sc.yaw;
  Eigen::Vector4d vel = Eigen::Vector4d::Zero();  // vx, vy, vz, wz (body)
  geometry_msgs::msg::Twist last_cmd;

  const double dt = 1.0 / 30.0;
  Result r;
  double dev_sum = 0.0;
  int n = 0;
  for (double t = 0.0; t < sc.timeout; t += dt) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = tf_info.map_frame;
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    nav_state.set("robot_pose", odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = tf_info.map_frame;
    tf.child_frame_id = tf_info.robot_frame;
    tf.transform.translation.x = pos.x();
    tf.transform.translation.y = pos.y();
    tf.transform.translation.z = pos.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    easynav::RTTFBuffer::getInstance()->setTransform(tf, "mpc_sim", true);
    if (!sc.obstacles.empty() || !sc.body_points.empty()) {
      easynav::PointPerception perception;
      perception.frame_id = tf_info.map_frame;
      perception.stamp = node->now();
      perception.valid = true;
      for (const auto & o : sc.obstacles) {
        perception.data.push_back(pcl::PointXYZ(o.x(), o.y(), o.z()));
      }
      for (const auto & b : sc.body_points) {
        const Eigen::Vector3d w = pos + Eigen::Vector3d(
          std::cos(yaw) * b.x() - std::sin(yaw) * b.y(),
          std::sin(yaw) * b.x() + std::cos(yaw) * b.y(), b.z());
        perception.data.push_back(pcl::PointXYZ(w.x(), w.y(), w.z()));
      }
      nav_state.set("points", perception);
    }

    mpc.update_rt(nav_state);
    const auto cmd = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist;
    r.max_solve_ms = std::max(r.max_solve_ms, mpc.last_result().solve_ms);
    const auto & qp = mpc.last_result();
    if (qp.variables > 0 &&  // it ran (not turning in place, or at the goal)
      (qp.status == easynav::mpc::QPStatus::Infeasible ||
      qp.status == easynav::mpc::QPStatus::Failed))
    {
      ++r.qp_failures;
    }
    r.max_linear_jump = std::max(r.max_linear_jump, std::abs(cmd.linear.x - last_cmd.linear.x));
    if (std::abs(cmd.linear.x) + std::abs(cmd.linear.y) + std::abs(cmd.linear.z) < 0.02) {
      r.max_rotation_in_place = std::max(r.max_rotation_in_place, std::abs(cmd.angular.z));
    }
    if (limits.min_turning_radius > 0.0 && std::abs(cmd.linear.x) > 1e-3) {
      r.max_curvature_excess = std::max(
        r.max_curvature_excess,
        std::abs(cmd.angular.z) - std::abs(cmd.linear.x) / limits.min_turning_radius);
    }
    last_cmd = cmd;
    if ((sc.verbose || std::getenv("MPC_SIM_VERBOSE")) && static_cast<int>(t / dt) % 5 == 0) {
      std::printf(
        "t=%5.2f pos=(%6.2f,%6.2f,%5.2f) yaw=%6.2f cmd=(%5.2f,%5.2f,%5.2f,%5.2f) st=%d\n", t,
        pos.x(), pos.y(), pos.z(), yaw, cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z,
        static_cast<int>(mpc.last_result().status));
    }

    // The robot: its velocity follows the command (with the lag), within the robot limits
    Eigen::Vector4d target(
      std::clamp(cmd.linear.x, limits.min_linear_vel, limits.max_linear_vel),
      std::clamp(cmd.linear.y, -limits.max_lateral_vel, limits.max_lateral_vel),
      std::clamp(cmd.linear.z, -limits.max_vertical_vel, limits.max_vertical_vel),
      std::clamp(cmd.angular.z, -limits.max_angular_vel, limits.max_angular_vel));
    if (sc.model == "diff" || ackermann) {target(1) = 0.0;}
    if (!aerial) {target(2) = 0.0;}
    const double a = sc.lag > 0.0 ? std::exp(-dt / sc.lag) : 0.0;
    vel = a * vel + (1.0 - a) * target;
    if (ackermann) {
      // Steering at the rear axle: w = v tan(delta) / L, |delta| within the turning radius
      const double max_delta = limits.min_turning_radius > 0.0 ?
        std::atan(sc.wheelbase / limits.min_turning_radius) : M_PI / 2.0 - 1e-3;
      const double delta = std::abs(vel(0)) > 1e-6 ?
        std::clamp(std::atan(sc.wheelbase * vel(3) / vel(0)), -max_delta, max_delta) : 0.0;
      vel(3) = vel(0) * std::tan(delta) / sc.wheelbase;
    }
    const int sub = 10;
    for (int i = 0; i < sub; ++i) {
      const double h = dt / sub;
      const double c = std::cos(yaw), s = std::sin(yaw);
      pos.x() += (vel(0) * c - vel(1) * s) * h;
      pos.y() += (vel(0) * s + vel(1) * c) * h;
      pos.z() += vel(2) * h;
      yaw += vel(3) * h;
    }

    const double dev = distance_to(pos, path_pts);
    dev_sum += dev;
    ++n;
    r.dev_max = std::max(r.dev_max, dev);
    r.v_max = std::max(r.v_max, std::hypot(vel(0), vel(1)));
    for (const auto & o : sc.obstacles) {
      const Eigen::Vector3d d = o - pos;
      // Clearance from the robot's circle (ground: in the plane)
      const double dist = aerial ? d.norm() : d.head<2>().norm();
      r.min_clearance = std::min(r.min_clearance, dist - sc.geometry.radius);
    }
    r.time = t;
    if ((goal - pos).norm() < sc.goal_tolerance &&
      std::abs(vel(0)) + std::abs(vel(1)) + std::abs(vel(2)) < 0.05)
    {
      r.reached = true;
      break;
    }
  }
  r.dev_mean = n ? dev_sum / n : 0.0;
  r.final_position = pos;
  r.final_yaw = yaw;
  easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
  return r;
}

}  // namespace mpc_sim

#endif  // CONTROLLERS__EASYNAV_MPC_CONTROLLER__TESTS__MPC_SIM_HPP_
