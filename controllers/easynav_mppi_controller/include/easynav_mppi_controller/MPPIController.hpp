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

#ifndef EASYNAV_MPPI_CONTROLLER__MPPICONTROLLER_HPP_
#define EASYNAV_MPPI_CONTROLLER__MPPICONTROLLER_HPP_

#include <optional>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_core/RobotLimits.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_mppi_controller/MPPIOptimizer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easynav
{

/// \brief NavState key of an optional heading (double, rad, path frame) for holonomic robots:
/// while present, the robot keeps it as it moves, whatever heading_mode says.
inline constexpr char kHeadingReferenceKey[] = "heading_reference";

/// \brief MPPI controller for differential and holonomic (omnidirectional) robots.
class MPPIController : public ControllerMethodBase
{
public:
  MPPIController();

  /// \brief Destructor.
  ~MPPIController() override;

  /// \brief Initializes parameters and MPPI controller.
  /// \throws std::runtime_error if a parameter is invalid.
  void on_initialize() override;

  /// \brief Updates the controller using the given NavState.
  /// \param nav_state Current navigation state, including odometry and planned path.
  void update_rt(NavState & nav_state) override;

protected:
  MPPIParams params_;                        ///< Optimizer configuration.
  HeadingMode heading_mode_ {HeadingMode::Path};
  double heading_yaw_ {0.0};                 ///< For heading_mode "fixed" (rad).
  double xy_goal_tolerance_ {0.25};          ///< Positional tolerance to the goal (m).
  double yaw_goal_tolerance_ {0.25};         ///< Angular tolerance to the goal (rad).
  double obstacle_range_ {2.0};  ///< Obstacles farther than this from the robot are ignored (m).
  double z_min_filter_ {0.0};    ///< Points below this height (robot frame) are the ground (m).
  double robot_height_ {0.5};    ///< From robot_geometry.
  int max_visualized_rollouts_ {20};  ///< Candidate rollouts published (0: none).

  MPPIOptimizer optimizer_;
  RobotLimits limits_;           ///< Limits the optimizer uses now.
  Velocity last_cmd_;            ///< Last command (the velocity the robot is reaching).
  std::optional<rclcpp::Time> last_update_;
  double hold_yaw_ {0.0};        ///< Heading when the current goal was received (mode "hold").
  bool have_goal_ {false};
  double goal_x_ {0.0};
  double goal_y_ {0.0};
  MPPIStatus last_status_ {MPPIStatus::NoPath};

  geometry_msgs::msg::TwistStamped twist_stamped_;  ///< Current velocity command.

  /// Publisher for MPPI candidates markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mppi_candidates_pub_;
  /// Publisher for MPPI optimal path markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mppi_optimal_pub_;

  /// \brief Obstacle points (map frame) within the robot's height and obstacle_range of it,
  /// all around it (a holonomic robot moves in any direction, any robot turns).
  pcl::PointCloud<pcl::PointXYZ> obstacle_points(const NavState & nav_state) const;

  /// \brief The optimizer's limits from the robot limits.
  void apply_limits(const RobotLimits & limits);

  /// \brief Stops the robot and forgets the solution and the goal.
  void stop(NavState & nav_state, bool forget_goal);

  /// \brief Publishes the rollouts and the optimal trajectory.
  void publish_markers();

  /// \brief Deletes the markers.
  void clear_markers();
};

}  // namespace easynav

#endif  // EASYNAV_MPPI_CONTROLLER__MPPICONTROLLER_HPP_
