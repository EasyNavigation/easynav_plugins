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
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
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
| `<plugin>.latitude_origin` | `double` | `0.0` | Latitude (deg) of the map's origin, for GNSS fixes (global filter). |
| `<plugin>.longitude_origin` | `double` | `0.0` | Longitude (deg) of the map's origin. |
| `<plugin>.altitude_origin` | `double` | `0.0` | Altitude (m) of the map's origin. |
| `<plugin>.projection` | `string` | `utm` | How fixes become map coordinates: `utm`, the UTM grid shifted to the origin (its axes turn up to a few degrees from east and north away from the zone's central meridian), or `enu`, east-north-up at the origin, as Gazebo's `spherical_coordinates` and most local maps define it. |
| `<plugin>.navsatfix_topic` | `string` | `gps/filtered` | Where the global estimate is published as a fix. |

A fix is the position of the GNSS antenna, in the frame of its `header.frame_id`. With the robot's attitude and the antenna's mounting (TF from the filter's `base_link_frame` to that frame), the first fix places the robot, not the antenna, and the filter removes the mounting from the later ones. Without attitude or without that TF, the robot starts where the antenna is.

The attitude comes from the IMU of the sensor named `imu`: its orientation without its own mounting (TF from `base_link_frame` to the IMU's `header.frame_id`). An IMU that gives no orientation (`orientation_covariance[0] = -1`, or a null quaternion) or whose mounting is not in TF gives none; later fixes then use the filter's estimated heading. The IMU's yaw must be absolute (east-north-up, as a magnetometer or Gazebo gives it): one relative to where the IMU started turns the mounting the wrong way.

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
