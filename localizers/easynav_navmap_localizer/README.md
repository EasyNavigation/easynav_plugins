# easynav_navmap_localizer

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
Easy Navigation: nAVmAP Localizer package.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_navmap_localizer/AMCLLocalizer`
- **Type:** `easynav::navmap::AMCLLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `easynav_navmap_localizer`
- **Description:** A default "navmap" implementation for the AMCL localizer.

## Parameters
All parameters are declared under the plugin name namespace, i.e., `/<node_fqn>/easynav_navmap_localizer/AMCLLocalizer/...`.

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
| `<plugin>.inflation_stddev` | `double` | `0.05` | Std dev used in occupancy inflation (m). |
| `<plugin>.inflation_prob_min` | `double` | `0.01` | Minimum occupancy probability after inflation. |
| `<plugin>.correct_max_points` | `int` | `1500` | Maximum points used per correction step (after downsampling). |
| `<plugin>.weights_tau` | `double` | `0.7` | Exponential decay factor for particle weight smoothing. |
| `<plugin>.top_keep_fraction` | `double` | `0.2` | Fraction of top particles to keep before reseeding. |
| `<plugin>.downsampled_cloud_size` | `double` | `0.05` | Voxel size for point cloud downsampling (m). |

## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `/odom` | `nav_msgs/msg/Odometry` | Used only when <plugin>.compute_odom_from_tf = false | SensorDataQoS (reliable) |
| Publisher | `<node_fqn>/<plugin>/particles` | `geometry_msgs/msg/PoseArray` | Publishes the current particle set | depth=10 |
| Publisher | `<node_fqn>/<plugin>/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Publishes the estimated pose with covariance | depth=10 |

### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `imu` | `IMUPerceptions` | **Read** | Used to get the latest IMU orientation when available. |
| `points` | `PointPerceptions` | **Read** | 3D point cloud bundles used in the correction step. |
| `map.navmap` | `::navmap::NavMap` | **Read** | Triangle mesh map providing surface geometry and elevation. |
| `map.bonxai` | `Bonxai::ProbabilisticMap` | **Read** | Probabilistic occupancy map used for scoring. |
| `map.bonxai.inflated` | `Bonxai::ProbabilisticMap` | **Write/Read** | Inflated map cached on first access for faster scoring. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | Estimated robot pose stored at the end of update/predict. |

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Broadcasts the global transform that aligns the odometry frame with the map frame. |
| Requires | `odom -> base_footprint` | Required only when <plugin>.compute_odom_from_tf = true (read from TF). |
| Requires | `base_footprint -> <sensor_frame>` | Required to transform perception point clouds into the robot base frame during correction. |

## License
GPL-3.0-only
