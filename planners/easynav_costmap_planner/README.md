# easynav_costmap_planner

[![ROS 2: humble](https://img.shields.io/badge/ROS%202-humble-blue)](#) [![ROS 2: jazzy](https://img.shields.io/badge/ROS%202-jazzy-blue)](#) [![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
A planner plugin implementing a standard **A\*** path planner over a `Costmap2D` representation. It reads the dynamic costmap, goal list, and robot pose from NavState and publishes a computed path as a `nav_msgs/Path` message.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab  
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| humble | ![kilted](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![kilted](https://img.shields.io/badge/jazzy-supported-brightgreen) |
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
| `<plugin>.cost_factor` | `double` | `2.0` | Scaling factor applied to costmap cell values to compute traversal costs. Higher values make high-cost cells much less attractive. |
| `<plugin>.inflation_penalty` | `double` | `5.0` | Extra penalty added to inflated/near-obstacle cells to push paths away from obstacles. |
| `<plugin>.heuristic_scale` | `double` | `1.0` | Scaling factor applied to the A* heuristic term. Controls the trade-off between following the cheapest route vs. going more directly. |
| `<plugin>.continuous_replan` | `bool` | `true` | If `true`, re-plans continuously as the map/goal changes; if `false`, plans once per request. |

### Effect of `cost_factor`

The planner uses a per-step traversal cost:

$$
g_{\text{new}} = g_{\text{current}} + \text{step\_cost} \cdot \left(1 + \text{cost\_factor} \cdot \frac{\text{cell\_cost}}{\text{LETHAL\_OBSTACLE}}\right)
$$

- **Low `cost_factor` (e.g. 1–2):**
	- Cell cost has a modest influence; the planner is closer to a pure geometric shortest-path search.
	- Paths may cross moderately expensive regions if that keeps distance short.
- **High `cost_factor` (e.g. 5–10+):**
	- High-cost cells become very unattractive, so the planner strongly prefers low-cost corridors (e.g. routes or low-inflation areas), even if they are longer.
	- Useful when combined with modules that encode preferences as higher costs (such as route managers).

### Effect of `heuristic_scale`

The A* priority is:

$$
f = g + h, \quad h = \text{heuristic\_scale} \cdot d_{\text{cells}}(\text{current}, \text{goal})
$$

where $d_{\text{cells}}$ is the Euclidean distance in cell indices (not metric distance, but proportional).

- **Decrease `heuristic_scale` (< 1.0):**
	- The heuristic term becomes weaker, and A* behaves more like Dijkstra.
	- The search explores more alternatives and is guided more strictly by the true accumulated cost $g$.
	- This usually makes the planner follow high-cost penalties (e.g. for leaving a route) more faithfully, at the expense of more computation.

- **Increase `heuristic_scale` (> 1.0):**
	- The heuristic term dominates more, so the planner prefers geometrically direct paths.
	- It may still avoid extremely high-cost regions if `cost_factor` is large, but will be more willing to “cut corners” through slightly more expensive zones.

**Typical tuning strategy:**

- First, adjust `cost_factor` so that the costmap properly encodes your preferences:
	- Increase it until paths clearly avoid high-cost areas that should be disfavored.
- Then, fine-tune `heuristic_scale`:
	- Start from `1.0`.
	- If the planner still takes shortcuts that ignore cost differences, consider reducing it slightly (e.g. `0.7–0.9`).
	- If planning becomes too slow or explores too widely, you can increase it modestly (e.g. `1.2–1.5`).

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
Apache-2.0
