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

#ifndef EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_
#define EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_core/RobotLimits.hpp"
#include "easynav_mpc_controller/MPCOptimizer.hpp"
#include "easynav_mpc_controller/Obstacles.hpp"
#include "easynav_mpc_controller/Reference.hpp"

namespace easynav
{

/// @brief Model predictive controller: tracks the path with a linear time-varying MPC.
///
/// Each cycle samples a reference along the path ahead (at the speed its curvature, the goal and
/// the obstacles allow), linearizes the motion model (diff, omni, ackermann or aerial) around the
/// previous solution and solves a QP over the horizon with OSQP, with the robot limits as
/// constraints and half-planes keeping the robot's shape off the perceived obstacles.
class MPCController : public ControllerMethodBase
{
public:
  MPCController() = default;
  ~MPCController() override = default;

  void on_initialize() override;
  void update_rt(NavState & nav_state) override;

  /// The QP status and solve time of the last cycle (tests, diagnostics).
  const mpc::MPCResult & last_result() const {return last_result_;}

protected:
  /// Robot limits to the optimizer's per-axis limits and the reference's speed limits.
  void apply_limits(const RobotLimits & limits);
  /// The perceived points the MPC avoids, in the map frame: not those on the robot itself (its
  /// own body seen by its sensors), at \p pose.
  std::vector<Eigen::Vector3d> obstacle_points(
    const NavState & nav_state, const Eigen::VectorXd & pose);
  void publish_command(NavState & nav_state, const Eigen::VectorXd & v);
  void stop(NavState & nav_state);
  /// The velocity braking from the current one within the deceleration limits.
  Eigen::VectorXd braking(double dt) const;
  void publish_paths(
    const std::vector<Eigen::VectorXd> & predicted, const std::vector<Eigen::VectorXd> & reference,
    const std::string & frame);

  mpc::MotionModel model_ {mpc::MotionModel::Diff};
  mpc::ModelLayout layout_ {mpc::MotionModel::Diff};
  mpc::MPCOptimizer optimizer_;
  mpc::Reference reference_;
  mpc::Obstacles obstacles_;
  mpc::RobotShape shape_;
  mpc::SpeedLimits speed_limits_;
  RobotLimits limits_;
  double robot_height_ {0.5};

  double latency_ {0.0};
  double rotate_to_heading_angle_ {0.785};
  double xy_goal_tolerance_ {0.2};
  double yaw_goal_tolerance_ {0.2};
  double goal_align_gain_ {1.0};
  double min_height_ {0.1};
  bool use_odometry_velocity_ {false};

  Eigen::VectorXd velocity_;  ///< Robot's velocity estimate (model's layout).
  Eigen::VectorXd last_cmd_;
  std::optional<rclcpp::Time> last_update_;
  bool aligning_ {false};   ///< At the goal, turning to its heading.
  bool rotating_ {false};   ///< Turning in place towards the path before following it.
  mpc::MPCResult last_result_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  geometry_msgs::msg::TwistStamped cmd_vel_;
};

}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_
