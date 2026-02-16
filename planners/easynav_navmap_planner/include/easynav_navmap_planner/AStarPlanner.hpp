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

#include <vector>
#include <cstdint>
#include <Eigen/Core>

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
/// This class generates a collision-free path using A* search over a surface-based NavMap.
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
   * and prepares the NavMap-based planning environment.
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
  double inflation_penalty_;  ///< Extra cost penalty for paths near inflated obstacles.
  double cost_axial_;         ///< Cost multiplier for axial (horizontal/vertical) moves.
  double cost_diagonal_;      ///< Cost multiplier for diagonal moves.
  std::string layer_name_;
  bool continuous_replan_ {true};     ///< Whether to replan the path at control frequency.
  nav_msgs::msg::Path current_path_;  ///< Most recently computed path.
  geometry_msgs::msg::Pose current_goal_;  ///< Current goal.

  /// Publisher for the computed navigation path (for visualization or monitoring).
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /// Cached centroids for each NavCel (same indexing as ::navmap::NavMap::navcels).
  std::vector<Eigen::Vector3f> centroids_;

  /// Cached per-NavCel occupancy / cost values (0..255).
  std::vector<std::uint8_t> occ_;

  /// Reusable buffers for A* search cost and parent links.
  std::vector<double> g_;
  std::vector<::navmap::NavCelId> parent_;

  /**
   * @brief Ensure internal caches (centroids and A* buffers) are sized for the given map.
   *
   * This avoids reallocations on every planning call. Values in g_ and parent_
   * are reset for the current run.
   *
   * @param map The NavMap for which caches must be valid.
   */
  void ensure_graph_cache(const ::navmap::NavMap & map);

  /**
   * @brief Smooth a Path in XY while keeping every waypoint inside its original NavCel.
   *
   * The algorithm performs several iterations of Laplacian smoothing on XY:
   *   p_i' = (1 - alpha) * p_i + alpha * 0.5 * (p_{i-1} + p_{i+1})
   * For each i, the candidate point is clamped to the original triangle (NavCel)
   * using closest-point-on-triangle, so it never leaves that NavCel. The final z'
   * is the triangle height at the resulting (x', y').
   *
   * Endpoints are kept fixed. Optionally, points forming a sharp angle are also
   * kept (see `corner_keep_deg`).
   *
   * @param in_path         Input nav_msgs::msg::Path (world coordinates).
   * @param navmap          The NavMap where the path lies on.
   * @param iterations      Number of smoothing iterations (>= 1). Default: 5.
   * @param alpha           Smoothing factor in (0, 0.5]. Default: 0.4.
   * @param corner_keep_deg Angle threshold (degrees): if the interior angle at a point
   *                        is below this value, the point is kept as an anchor. Set <= 0
   *                        to disable. Default: 0 (disabled).
   * @return nav_msgs::msg::Path Smoothed path, same frame_id and header stamp as input.
   */
  nav_msgs::msg::Path path_smoother(
    const nav_msgs::msg::Path & in_path,
    const ::navmap::NavMap & navmap,
    int iterations = 5,
    float alpha = 0.4f,
    float corner_keep_deg = 0.0f);

  /**
   * @brief Smooth a Path in XY while keeping every waypoint inside its original NavCel.
   *
   * The algorithm performs several iterations of Laplacian smoothing on XY:
   *   p_i' = (1 - alpha) * p_i + alpha * 0.5 * (p_{i-1} + p_{i+1})
   * For each i, the candidate point is clamped to the original triangle (NavCel)
   * using closest-point-on-triangle, so it never leaves that NavCel. The final z'
   * is the triangle height at the resulting (x', y').
   *
   * Endpoints are kept fixed. Optionally, points forming a sharp angle are also
   * kept (see `corner_keep_deg`).
   *
   * @param in_path         Input nav_msgs::msg::Path (world coordinates).
   * @param navmap          The NavMap where the path lies on.
   * @param iterations      Number of smoothing iterations (>= 1). Default: 5.
   * @param alpha           Smoothing factor in (0, 0.5]. Default: 0.4.
   * @param corner_keep_deg Angle threshold (degrees): if the interior angle at a point
   *                        is below this value, the point is kept as an anchor. Set <= 0
   *                        to disable. Default: 0 (disabled).
   * @return nav_msgs::msg::Path Smoothed path, same frame_id and header stamp as input.
   */
  nav_msgs::msg::Path path_smoother(
    const nav_msgs::msg::Path & in_path,
    const ::navmap::NavMap & navmap,
    int iterations = 5,
    float alpha = 0.4f,
    float corner_keep_deg = 0.0f);

  /**
   * @brief Internal A* path planning routine.
   *
   * Computes a path on the given NavMap from the start pose to the goal pose.
   *
   * Movement cost is influenced by:
   * - The cost of each NavCel (retrieved from a layer).
   * - Additional inflation penalties near obstacles.
   *
   * @param map   The NavMap to plan over.
   * @param start The robot's starting pose in world coordinates.
   * @param goal  The goal pose in world coordinates.
   * @return A vector of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const ::navmap::NavMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);
};

}  // namespace navmap

}  // namespace easynav

#endif  // EASYNAV_NAVMAP_PLANNER__NAVMAPPLANNER_HPP_
