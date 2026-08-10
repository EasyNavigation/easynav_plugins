# easynav_vff_controller

## Description

Vector Field Histogram (VFF) style local obstacle avoidance controller. Generates `cmd_vel` commands from proximity/cost data and a target path reference using a histogram-based steering selection.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![kilted](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![kilted](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_vff_controller/VffController`  
  **Type:** `easynav::VffController`  
  **Base Class:** `easynav::ControllerMethodBase`  
  **Library:** `easynav_vff_controller`  
  **Description:** Histogram-based obstacle avoidance controller producing velocity commands.

## Parameters

No ROS parameters are currently declared in code. (Add declarations in the plugin to enable runtime tuning.)

> This plugin derives from [`easynav::ControllerMethodBase`](https://github.com/EasyNavigation/EasyNavigation/tree/rolling/easynav_core#easynavcontrollermethodbase).  \
> See that section for shared collision-checking parameters and debug markers common to all controllers.

## Interfaces

### NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current pose for steering decisions. |
| `path` | `nav_msgs::msg::Path` | **Read** | Planned path reference. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** | Output velocity command. |

### Publications

| Topic | Type | Purpose | QoS |
|---|---|---|---|
| `/vff/markers` | `visualization_msgs/msg/MarkerArray` | Histogram / debug visualization (name inferred from code marker publisher variable). | depth=10 |

### Subscriptions / Services

None directly; relies on NavState for data sharing.

## TF Frames

Relies on frames stamped in `robot_pose`; does not query TF directly.

## License

Apache-2.0
