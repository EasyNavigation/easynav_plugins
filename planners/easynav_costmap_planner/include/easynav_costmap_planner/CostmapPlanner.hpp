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

/// \file
/// \brief Declaration of the CostmapPlanner class implementing A* path planning using Costmap2D.

#ifndef EASYNAV_COSTMAP_PLANNER__COSTMAPPLANNER_HPP_
#define EASYNAV_COSTMAP_PLANNER__COSTMAPPLANNER_HPP_

#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/// \brief A planner implementing the A* algorithm on a Costmap2D grid.
///
/// This class generates a collision-free path using A* search over a 2D costmap.
/// It supports cost-based penalties and anisotropic movement costs.
class CostmapPlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Initializes internal parameters and configuration values.
   */
  explicit CostmapPlanner();

  /**
   * @brief Initializes the planner.
   *
   * Loads planner parameters, sets up ROS publishers,
   * and prepares the costmap-based planning environment.
   *
   * @throws std::runtime_error if initialization fails.
   */
  virtual void on_initialize() override;

  /**
   * @brief Executes a planning cycle using the current navigation state.
   *
   * Computes a path from the robot's current pose to the goal using A*.
   *
   * @param nav_state Current shared navigation state (input/output).
   */
  void update(NavState & nav_state) override;

protected:
  double cost_factor_;        ///< Scaling factor applied to cell cost values.
  double inflation_penalty_; ///< Extra cost penalty for paths near inflated obstacles.
  double heuristic_scale_ {1.0}; ///< Scaling factor applied to the heuristic term in A*.
  bool continuous_replan_ {true};    ///< Wheter replan path at freq time
  nav_msgs::msg::Path current_path_;  ///< Most recently computed path.
  geometry_msgs::msg::Pose current_goal_;  ///< Current goal.

  /// Publisher for the computed navigation path (for visualization or monitoring).
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /**
   * @brief Internal A* path planning routine.
   *
   * Computes a path on the given costmap from the start pose to the goal pose.
   *
   * Movement cost is influenced by:
   * - The cost of each cell (retrieved from the costmap).
   * - Additional inflation penalties near obstacles.
   * - Anisotropic weights for axial vs diagonal movement.
   *
   * @param map The costmap to plan over.
   * @param start The robot's starting pose in world coordinates.
   * @param goal The goal pose in world coordinates.
   * @return A vector of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const Costmap2D & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_PLANNER__COSTMAPPLANNER_HPP_
