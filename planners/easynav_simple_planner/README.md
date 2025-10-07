# easynav_simple_planner

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
A simple A* planner over a lightweight `SimpleMap` grid. It reads the dynamic simple map, goals, and robot pose from NavState and publishes a `nav_msgs/Path`.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_simple_planner/SimplePlanner`
- **Type:** `easynav::SimplePlanner`
- **Base Class:** `easynav::PlannerMethodBase`
- **Library:** `easynav_simple_planner`
- **Description:** A simple A* planner over a lightweight `SimpleMap` grid. It reads the dynamic simple map, goals, and robot pose from NavState and publishes a `nav_msgs/Path`.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_simple_planner/SimplePlanner/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.robot_radius` | `double` | `0.3` | Robot inscribed radius (m) used to validate traversability. |
| `<plugin>.clearance_distance` | `double` | `0.2` | Extra clearance distance (m) to keep away from obstacles. |


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
| `map.dynamic` | `SimpleMap` | **Read** | Dynamic `SimpleMap` grid used for search. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Start pose for path planning. |
| `path` | `nav_msgs::msg::Path` | **Write** | Output path to follow. |


## TF Frames
This plugin does not perform TF lookups directly; frame consistency is assumed between map, robot pose, and published path.

## License
GPL-3.0-only
