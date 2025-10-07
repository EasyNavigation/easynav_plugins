# easynav_navmap_planner

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
A* path planner over a NavMap triangular surface/layer. Consumes NavMap and goals from NavState and publishes a `nav_msgs/Path`.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_navmap_planner/AStarPlanner`
- **Type:** `easynav::navmap::AStarPlanner`
- **Base Class:** `easynav::PlannerMethodBase`
- **Library:** `easynav_navmap_planner`
- **Description:** A* path planner over a NavMap triangular surface/layer. Consumes NavMap and goals from NavState and publishes a `nav_msgs/Path`.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_navmap_planner/AStarPlanner/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.layer` | `string` | `"inflated_obstacles"` | NavMap layer name to read costs from (e.g., `inflated_obstacles`). |
| `<plugin>.cost_factor` | `double` | `2.0` | Scaling factor applied to cell/triangle costs. |
| `<plugin>.inflation_penalty` | `double` | `5.0` | Extra penalty near inflated/inscribed regions to keep paths away from obstacles. |
| `<plugin>.continuous_replan` | `bool` | `true` | Replan continuously as NavState updates (true) or plan once per request (false). |


## Interfaces (Topics and Services)

### Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/path` | `nav_msgs/msg/Path` | Publishes the computed A* path. | depth=10 |


This plugin does not create subscriptions or services directly; it reads inputs from `NavState`.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `goals` | `nav_msgs::msg::Goals` | **Read** | Planner targets. |
| `map` | `::navmap::NavMap` | **Read** | NavMap (reads the specified `layer`). |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Start pose for path planning. |
| `path` | `nav_msgs::msg::Path` | **Write** | Output path to follow. |


## TF Frames
This plugin does not perform TF lookups directly; frame consistency is assumed between NavMap, robot pose, and published path.

## License
GPL-3.0-only
