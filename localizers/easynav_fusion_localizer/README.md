# easynav_fusion_localizer

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
Odometry fusion localizer based on Robot Localization that fuses any n odometry sources.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Miguel Ángel de Miguel <midemig@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_fusion_localizer/FusionLocalizer`
- **Type:** `easynav::FusionLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `fusion_localizer`
- **Description:** Odometry fusion localizer based on Robot Localization that fuses any n odometry sources.

## Parameters

All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_fusion_localizer/FusionLocalizer/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.initial_pose.x` | `double` | `0.0` | Initial X position (m). |
| `<plugin>.initial_pose.y` | `double` | `0.0` | Initial Y position (m). |
| `<plugin>.initial_pose.yaw` | `double` | `0.0` | Initial yaw (rad). |

For additional robot_localization-related configuration, see the upstream documentation:
https://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html

## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Seed/reset the filter pose from an external initial pose estimate. | depth=10 |
| Publisher | `odometry/filtered` | `nav_msgs/msg/Odometry` | Odometry fused from all sources. | SensorDataQoS |


### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | Fused odometry estimate. |


## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Transform that aligns the odometry frame with the global frame. |


## License
Apache-2.0
