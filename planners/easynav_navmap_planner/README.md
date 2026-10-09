# easynav_navmap_planner

[![ROS 2: humble](https://img.shields.io/badge/ROS%202-humble-blue)](#) [![ROS 2: jazzy](https://img.shields.io/badge/ROS%202-jazzy-blue)](#) [![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
**A\*** path planner operating over a `NavMap` triangular mesh.  
The planner computes the **minimum-cost path** between the robot and the goal, taking into account both the geometric distance between triangles and the cost values stored in a selected NavMap layer (typically `"inflated_obstacles"`).

Instead of simply avoiding non-free NavCels, the planner integrates their cost values (0–255) into the path evaluation. Cells marked as `LETHAL_OBSTACLE`, `NO_INFORMATION` or inscribed (`INSCRIBED_INFLATED_OBSTACLE`) are non-traversable (inscribed ones only near the start, so a robot inside that band can leave it), while inflated cells are allowed but penalized proportionally to their cost.  

The search moves between NavCels that share a vertex, not only an edge, so no direction is favored by how the mesh is split; a move through a shared vertex is only taken when every NavCel around that vertex is traversable, so the path never slips between two obstacles that touch at a corner. Then the path is shortened by line of sight: from each waypoint it goes straight to the farthest one whose segment crosses only traversable NavCels no costlier than the waypoints it replaces, and the result is resampled at the NavCel spacing on the surface. The path ends exactly at the goal.  

This enables smoother and safer trajectories that still respect proximity constraints imposed by obstacle inflation.


### Cost model
For two neighboring NavCels `u` and `v`, the edge cost is computed as:

\[
\text{cost}(u,v) = d(u,v) \times \left(\text{cost\_factor} + \text{inflation\_penalty} \times \frac{c(v)}{253}\right)
\]

where `d(u,v)` is the Euclidean distance between triangle centroids,  
and `c(v)` is the cost value of cell `v`.  
This formulation ensures that:
- cells near obstacles (high cost) are traversed only if geometrically necessary,
- lethal (`254`) and unknown (`255`) cells are not traversable.


## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_navmap_planner/AStarPlanner`
- **Type:** `easynav::navmap::AStarPlanner`
- **Base Class:** `easynav::PlannerMethodBase`
- **Library:** `easynav_navmap_planner`
- **Description:** A\* path planner over a NavMap triangular mesh using per-cell costs to compute the shortest safe path.

## Parameters
All parameters are declared under the plugin namespace, i.e.  
`/<node_fqn>/easynav_navmap_planner/AStarPlanner/...`

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.cost_factor` | `double` | `2.0` | Multiplicative weight for geometric distance; values > 1 increase the relative importance of distance. |
| `<plugin>.cost_weight` | `double` | `5.0` | Weight of the cell cost (inflation) against the path length; higher values keep paths farther from obstacles. |
| `<plugin>.continuous_replan` | `bool` | `true` | If true, recomputes the path whenever `NavState` updates; if false, plans once per goal. |

**Note:** The planner internally uses hardcoded values for `layer_name` (prefers `"inflated_obstacles"`, fallback to `"obstacles"`), `cost_axial`, and `cost_diagonal`. These are not runtime-configurable parameters.

## Interfaces (Topics and Services)

### Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/path` | `nav_msgs/msg/Path` | Publishes the computed A* path. | depth=10 |

This plugin does not create subscriptions or services directly; it retrieves all inputs from `NavState`.

## NavState Keys
| Key | Type | Access | Description |
|---|---|---|---|
| `goals` | `nav_msgs::msg::Goals` | **Read** | Target pose(s) for path planning. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose (start position). |
| `map.navmap` | `::navmap::NavMap` | **Read** | NavMap containing geometry and cost layer. The planner reads costs from the layer specified in `<plugin>.layer` (default: `"inflated_obstacles"`). If that layer does not exist, it automatically falls back to `"obstacles"`. |
| `path` | `nav_msgs::msg::Path` | **Write** | Output path, computed as the lowest-cost route. |

## TF Frames
The planner assumes frame consistency between NavMap, robot pose, and goals. No TF lookups are performed internally.

## License
Apache-2.0
