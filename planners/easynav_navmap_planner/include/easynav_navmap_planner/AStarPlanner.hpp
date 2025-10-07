// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the AStarPlanner class implementing A* path planning using ::navmap::NavMap.

#ifndef EASYNAV_NAVMAP_PLANNER__NAVMAPPLANNER_HPP_
#define EASYNAV_NAVMAP_PLANNER__NAVMAPPLANNER_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{
namespace navmap
{

/// \brief A planner implementing the A* algorithm on a ::navmap::NavMap grid.
///
/// This class generates a collision-free path using A* search over a 2D costmap.
/// It supports cost-based penalties and anisotropic movement costs.
class AStarPlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Initializes internal parameters and configuration values.
   */
  explicit AStarPlanner();

  /**
   * @brief Initializes the planner.
   *
   * Loads planner parameters, sets up ROS publishers,
   * and prepares the costmap-based planning environment.
   *
   * @return std::expected<void, std::string> A success indicator or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

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
  double cost_axial_;        ///< Cost multiplier for axial (horizontal/vertical) moves.
  double cost_diagonal_;     ///< Cost multiplier for diagonal moves.
  std::string layer_name_;
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
    const ::navmap::NavMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

  /**
   * @brief Internal static map.
   */
  ::navmap::NavMap navmap_;
};

}  // namespace navmap

}  // namespace easynav

#endif  // EASYNAV_NAVMAP_PLANNER__NAVMAPPLANNER_HPP_
