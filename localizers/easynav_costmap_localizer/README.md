# easynav_costmap_localizer

## Description

AMCL-style localizer using a 2D Costmap2D for scoring and odometry/IMU for prediction.

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

- **Plugin Name:** `easynav_costmap_localizer/AMCLLocalizer`
- **Type:** `easynav::AMCLLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `easynav_costmap_localizer`
- **Description:** AMCL-style localizer using a 2D Costmap2D for scoring and odometry/IMU for prediction.

## Parameters

All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_costmap_localizer/AMCLLocalizer/...`.

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
| `<plugin>.compute_odom_from_tf` | `bool` | `false` | If true, read odometry from TF (odom->base_footprint) instead of /odom topic. |

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `odom` | `nav_msgs/msg/Odometry` | Read odometry when compute_odom_from_tf=false. | SensorDataQoS (reliable) |
| Subscription | `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Init particles pose to the received pose, using covariance | depth=10 |
| Publisher | `<node_fqn>/<plugin>/particles` | `geometry_msgs/msg/PoseArray` | Publishes the current particle set. | depth=10 |
| Publisher | `<node_fqn>/<plugin>/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimated pose with covariance. | depth=10 |

### Services

This package does not create service servers or clients.

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `points` | `PointPerceptions` | **Read** | Perception point clouds used in correction. |
| `map.static` | `Costmap2D` | **Read** | Static costmap for likelihood evaluation. |

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Aligns the odometry frame with the map frame. |
| Requires (optional) | `odom -> base_footprint` | Required when compute_odom_from_tf=true (read from TF). |

## License

Apache-2.0
