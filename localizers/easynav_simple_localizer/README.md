# easynav_simple_localizer

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
AMCL-style localizer using a simple 2D grid-like map for scoring and odometry for prediction.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_simple_localizer/AMCLLocalizer`
- **Type:** `easynav::AMCLLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `easynav_simple_localizer`
- **Description:** AMCL-style localizer using a simple 2D grid-like map for scoring and odometry for prediction.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_simple_localizer/AMCLLocalizer/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.num_particles` | `int` | `100` | Number of AMCL particles. |
| `<plugin>.initial_pose.x` | `double` | `0.0` | Initial X position (m). |
| `<plugin>.initial_pose.y` | `double` | `0.0` | Initial Y position (m). |
| `<plugin>.initial_pose.yaw` | `double` | `0.0` | Initial yaw (rad). |
| `<plugin>.initial_pose.std_dev_xy` | `double` | `0.5` | Std dev used to sample initial X/Y. |
| `<plugin>.initial_pose.std_dev_yaw` | `double` | `0.5` | Std dev used to sample initial yaw. |
| `<plugin>.reseed_freq` | `double` | `1.0` | Reseeding frequency (Hz). |
| `<plugin>.noise_translation` | `double` | `0.01` | Translational noise factor. |
| `<plugin>.noise_rotation` | `double` | `0.01` | Rotational noise factor. |
| `<plugin>.noise_translation_to_rotation` | `double` | `0.01` | Translation-to-rotation noise coupling. |
| `<plugin>.min_noise_xy` | `double` | `0.05` | Minimum XY noise (m). |
| `<plugin>.min_noise_yaw` | `double` | `0.05` | Minimum yaw noise (rad). |


## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `/odom` | `nav_msgs/msg/Odometry` | Read odometry when compute_odom_from_tf=false (not present here). | SensorDataQoS (reliable) |
| Publisher | `<node_fqn>/<plugin>/particles` | `geometry_msgs/msg/PoseArray` | Publishes the current particle set. | depth=10 |
| Publisher | `<node_fqn>/<plugin>/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimated pose with covariance. | depth=10 |


### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `points` | `PointPerceptions` | **Read** | Perception point clouds used in correction. |
| `map.static` | `SimpleMap` | **Read** | Static map for likelihood evaluation. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | Estimated robot pose. |


## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Aligns the odometry frame with the map frame. |


## License
GPL-3.0-only
