# easynav_gps_localizer

## Description

GPS-based localizer that fuses NavSatFix and IMU to publish odometry in an odom-like frame.

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

- **Plugin Name:** `easynav_gps_localizer/GpsLocalizer`
- **Type:** `easynav::GpsLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `gps_localizer`
- **Description:** GPS-based localizer that fuses NavSatFix and IMU to publish odometry in an odom-like frame.

## Parameters

This plugin does not declare configurable ROS parameters.

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `robot/gps/fix` | `sensor_msgs/msg/NavSatFix` | Raw GPS fix. | SensorDataQoS (reliable) |
| Subscription | `imu/data` | `sensor_msgs/msg/Imu` | IMU orientation for yaw fusion. | SensorDataQoS (reliable) |
| Publisher | `robot/odom_gps` | `nav_msgs/msg/Odometry` | Odometry fused from GPS + IMU (UTM-projected). | SensorDataQoS |

### Services

This package does not create service servers or clients.

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | GPS-based odometry estimate. |

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | `map (or world) -> odom_gps` | Static or slowly varying transform aligning UTM to local odometry frame. |
| Optional | `base_link -> imu` | Used implicitly via IMU orientation; not looked up explicitly in this plugin. |

## License

Apache-2.0
