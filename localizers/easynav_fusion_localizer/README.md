# easynav_gps_localizer

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

TODO

See [robot_localization documentation](https://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html)

## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `odometry/filtered` | `nav_msgs/msg/Odometry` | Odometry fused from all sources. | SensorDataQoS |


### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | GPS-based odometry estimate. |


## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Static or slowly varying transform aligning UTM to local odometry frame. |


## License
Apache-2.0
