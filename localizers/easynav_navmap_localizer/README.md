# easynav_navmap_localizer

## Description

Easy Navigation: nAVmAP Localizer package.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
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
| `<plugin>.correct_max_points` | `int` | `1500` | Maximum points of each sensor's cloud scored per correction (after downsampling and the height band), evenly spread over it. |
| `<plugin>.weights_tau` | `double` | `0.7` | Exponential decay factor for particle weight smoothing. |
| `<plugin>.top_keep_fraction` | `double` | `0.2` | Fraction of top particles to keep before reseeding. |
| `<plugin>.downsampled_cloud_size` | `double` | `0.05` | Voxel size for point cloud downsampling (m). Of the order of the Bonxai map's voxels: much larger leaves too few points to tell poses apart. |
| `<plugin>.min_point_height` | `double` | `-inf` | Only points at least this high over the robot's footprint are scored (m). For 3D sensors: the floor fits any particle standing on it. |
| `<plugin>.max_point_height` | `double` | `inf` | Only points at most this high over the robot's footprint are scored (m), e.g. below a ceiling. |
| `<plugin>.ray_occlusion` | `bool` | `false` | A point whose ray from the sensor crosses an occupied voxel scores nothing. With voxels a few decimetres wide, rays along a floor or a ceiling look occluded, and the true pose scores worse than wrong ones. |
| `<plugin>.reseed_min_d` | `double` | `0.1` | Reseed only after the odometry has moved this much since the last reseed (m), or turned `reseed_min_a`: standing still, reseeding makes the particles wander. |
| `<plugin>.reseed_min_a` | `double` | `0.1` | See `reseed_min_d` (rad). |
| `<plugin>.odom_slip_fraction` | `double` | `0.05` | Each particle moves with its own scale of the odometry's translation. This fraction of the reseeded ones take any scale in [0, 1]: when the wheels slip, those that hardly move fit the sensors and survive. Rough terrain: higher (0.2). |
| `<plugin>.odom_scale_noise` | `double` | `0.02` | Noise of the scale a reseeded particle takes from its reference: the particles learn the odometry's scale. |
| `<plugin>.recovery_alpha_slow` | `double` | `0.001` | Rate of the slow average of the scan likelihood (of the best particles), per correction. |
| `<plugin>.recovery_alpha_fast` | `double` | `0.1` | Rate of the fast average. When it falls below the slow one, each reseed spreads the worst particles widely around the estimate, a fraction `1 - fast/slow` of those it replaces. `0` for both turns recovery off; it needs `0 <= slow < fast <= 1`. |
| `<plugin>.recovery_spread_xy` | `double` | `1.0` | Standard deviation of the spread particles' position around the estimate (m). |
| `<plugin>.recovery_spread_yaw` | `double` | `0.5` | Standard deviation of their heading around the best particle's (rad). |

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `/odom` | `nav_msgs/msg/Odometry` | Used only when `<plugin>.compute_odom_from_tf = false` | SensorDataQoS (reliable) |
| Subscription | `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initialize particles pose to the received pose, using covariance | depth=10 |
| Publisher | `<node_fqn>/<plugin>/particles` | `geometry_msgs/msg/PoseArray` | Publishes the current particle set | depth=10 |
| Publisher | `<node_fqn>/<plugin>/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Publishes the estimated pose with covariance | depth=10 |

### Services

This package does not create service servers or clients.

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `imu` | `IMUPerceptions` | **Read** | When available, the particles take its roll and pitch, and turn by the change of its yaw (not by the odometry's); its absolute yaw is never used (without a magnetometer it is arbitrary and drifts). Without it, the orientation follows the NavMap surface and the odometry's turns. |
| `points` | `PointPerceptions` | **Read** | Point clouds used in the correction step: only the valid ones (newer than the sensors' `forget_time`), each where the sensor was when it took it (the odometry since then, through `odom`). |
| `map.navmap` | `::navmap::NavMap` | **Read** | Triangle mesh map providing surface geometry and elevation. |
| `map.bonxai` | `Bonxai::ProbabilisticMap` | **Read** | Probabilistic occupancy map used for scoring. |
| `map.bonxai.inflated` | `Bonxai::ProbabilisticMap` | **Write/Read** | Inflated map cached on first access for faster scoring. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | Estimated robot pose stored at the end of update/predict, with its roll and pitch (the IMU's, or the surface's). |

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Broadcasts the global transform that aligns the odometry frame with the map frame. It carries the robot's roll and pitch (the IMU's, or the surface's) when the odometry is planar: on a slope, the points the sensors see are placed where they are. |
| Requires | `odom -> base_footprint` | Required only when `<plugin>.compute_odom_from_tf = true` (read from TF). |
| Requires | `base_footprint -> <sensor_frame>` | Required to transform perception point clouds into the robot base frame during correction, at the cloud's time through `odom` (falls back to the latest one). |

## License

Apache-2.0
