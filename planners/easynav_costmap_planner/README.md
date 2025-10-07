# easynav_costmap_planner

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
A planner plugin implementing a standard **A\*** path planner over a `Costmap2D` representation. It reads the dynamic costmap, goal list, and robot pose from NavState and publishes a computed path as a `nav_msgs/Path` message.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab  
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_costmap_planner/CostmapPlanner`
- **Type:** `easynav::CostmapPlanner`
- **Base Class:** `easynav::PlannerMethodBase`
- **Library:** `easynav_costmap_planner`
- **Description:** A default Costmap2D-based A* path planner implementation.

---

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_costmap_planner/CostmapPlanner/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.cost_factor` | `double` | `2.0` | Scaling factor applied to costmap cell values to compute traversal costs. |
| `<plugin>.inflation_penalty` | `double` | `5.0` | Extra penalty added to inflated/near-obstacle cells to push paths away from obstacles. |
| `<plugin>.cost_axial` | `double` | `1.0` | Base movement cost for axial (N/E/S/W) steps. |
| `<plugin>.cost_diagonal` | `double` | `1.41` | Base movement cost for diagonal steps (approx. √2). |
| `<plugin>.continuous_replan` | `bool` | `true` | If `true`, re-plans continuously as the map/goal changes; if `false`, plans once per request. |

---

## Interfaces (Topics and Services)

### Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/path` | `nav_msgs/msg/Path` | Publishes the computed path from start to goal. | `depth=10` |

This plugin does not create subscriptions or services directly; it reads inputs via NavState.

---

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `goals` | `nav_msgs::msg::Goals` | **Read** | Goal list used as planner targets. |
| `map.dynamic` | `Costmap2D` | **Read** | Dynamic costmap used for A* search. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose used as the start position. |
| `path` | `nav_msgs::msg::Path` | **Write** | Output path to follow. |

---

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Reads | `map -> base_footprint` | Requires consistent frames between the costmap and the published path. |

---

## License
GPL-3.0-only
