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
/// \brief Declaration of the SimplePlanner class implementing A* path planning.

#ifndef EASYNAV_PLANNER__SIMPLEPLANNER_HPP_
#define EASYNAV_PLANNER__SIMPLEPLANNER_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/// \brief A planner implementing the A* algorithm on a SimpleMap grid.
class SimplePlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Initializes the internal variables and parameters of the planner.
   */
  explicit SimplePlanner();

  /**
   * @brief Initializes the planner.
   *
   * Configures publishers, retrieves parameters, and prepares the planner
   * for path generation using the available map data.
   *
   * @return std::expected<void, std::string> Success or an error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the planner by computing a new path.
   *
   * Uses the current navigation state (including the robot's position and goal)
   * to generate a path based on the A* algorithm.
   *
   * @param nav_state The current navigation state (contains odometry and goal information).
   */
  void update(NavState & nav_state) override;

protected:
  double robot_radius_;        ///< Radius of the robot used for collision checking.
  double clearance_distance_;  ///< Minimum clearance distance from obstacles in meters.

  nav_msgs::msg::Path current_path_;  ///< The last computed path.

  /// Publisher for the computed navigation path.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /**
   * @brief Runs the A* algorithm to compute a path.
   *
   * @param map The occupancy map used for path planning.
   * @param start The starting pose in world coordinates.
   * @param goal The target pose in world coordinates.
   * @param resolution The cell resolution of the map (in meters).
   * @return A sequence of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const SimpleMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double resolution);

  /**
   * @brief Checks whether a map cell is free, considering a clearance area.
   *
   * This function verifies if a cell and its surrounding cells (within the
   * specified clearance radius) are free of obstacles.
   *
   * @param map The occupancy map to query.
   * @param cx The x-coordinate of the cell.
   * @param cy The y-coordinate of the cell.
   * @param clearance_cells The clearance radius expressed in number of cells.
   * @return true if the cell and its clearance area are free, false otherwise.
   */
  bool isFreeWithClearance(
    const SimpleMap & map,
    int cx, int cy,
    double clearance_cells);
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__SIMPLEPLANNER_HPP_
