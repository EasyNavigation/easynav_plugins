# easynav_mhamcl_localizer

## Description

Multi-Hypothesis AMCL (MH-AMCL) localizer over a `Costmap2D` map. It is the EasyNav port of
[mh_amcl](https://github.com/fmrico/mh_amcl), described in *Portable Multi-Hypothesis Monte Carlo
Localization for Mobile Robots* (A. García, F. Martín, J. M. Guerrero, F. J. Rodríguez and V. Matellán,
ICRA 2023).

Instead of a single particle filter, it keeps a set of them (*hypotheses*) about the pose of the
robot:

- **Start:** the first hypothesis starts at `initial_pose` (or the pose sent to `initialpose`).
- **Creation:** every `1 / hypotheses_freq` seconds, a *cascade map matching* looks in the **whole**
  map for the poses from which the last perception could have been obtained. The map is stored in a
  pyramid of resolutions (each level halves the previous one). The coarsest level is scanned entirely
  with a fixed angular step, and only the promising cells are refined in the finer levels. A new
  hypothesis starts at every candidate that is far enough from the existing hypotheses.
- **Destruction:** a hypothesis is removed if it is out of the free space of the map or its quality
  is too low.
- **Merge:** hypotheses that converge to the same pose are merged.
- **Output:** the pose (and covariance) of the hypothesis with the best *quality*. Another
  hypothesis takes over only if it is clearly better than the current one.

The *quality* of a hypothesis is the best fraction of the last perception that falls on an obstacle
of the map from any of its particles. It describes how well a hypothesis explains what the robot
sees much better than the covariance does.

This allows to localize the robot **without knowing where it is** and to **recover from kidnapping
or wrong estimates**.

Every hypothesis is a regular particle filter with the phases run independently:

| Phase | Where | Frequency |
|---|---|---|
| Prediction | `update_rt` | `rt_freq` |
| Correction | `update` | `freq` |
| Reseed | `update` | `reseed_freq` |
| Hypotheses management / map matching | `update` (matching runs in a background thread) | `hypotheses_freq` |

Reseed also adapts the number of particles of each hypothesis in `[min_particles, max_particles]`:
it grows when the quality is low and shrinks when it is high.

### Differences with the Nav2 version

- The observation are the fused `PointPerception`s of NavState (like the other EasyNav localizers)
  instead of a `LaserScan`. Every point is seen along the ray from the robot to it.
- The map is the `map.base` `Costmap2D` of NavState, not an `OccupancyGrid` topic.
- The odometry is not read from its own subscription: it is the odometry perception of `easynav_sensors` (`OdometryPerceptionHandler`), falling back to `odom -> base_footprint` in the `RTTFBuffer`. TF and the initial pose are handled like `easynav_costmap_localizer`.
- The map matching refines the candidates down to the original resolution and runs in a background
  thread. The candidates are moved with the odometry received meanwhile.
- Fixes with respect to the original implementation: parents in reseed are really selected among
  the winners, reseed noise is a standard deviation and not a variance, hypotheses are removed and
  merged safely, and new hypotheses start with the quality of their candidate.

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

- **Plugin Name:** `easynav_mhamcl_localizer/MHAMCLLocalizer`
- **Type:** `easynav::mhamcl::MHAMCLLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `easynav_mhamcl_localizer`
- **Description:** Multi-Hypothesis AMCL localizer over a `Costmap2D` map.

See [config/example_params.yaml](config/example_params.yaml). To feed the odometry as a perception, add a sensor to `sensors_node`:

```yaml
sensors_node:
  ros__parameters:
    sensors: [laser1, odom]
    odom:
      topic: odom
      type: nav_msgs/msg/Odometry
```

## Parameters

All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_mhamcl_localizer/MHAMCLLocalizer/...`.

### Initial pose

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.initial_pose.x` | `double` | `0.0` | Initial X position (m). |
| `<plugin>.initial_pose.y` | `double` | `0.0` | Initial Y position (m). |
| `<plugin>.initial_pose.yaw` | `double` | `0.0` | Initial yaw (rad). |
| `<plugin>.initial_pose.std_dev_xy` | `double` | `0.5` | Std dev used to sample initial X/Y (m). |
| `<plugin>.initial_pose.std_dev_yaw` | `double` | `0.5` | Std dev used to sample initial yaw (rad). |

### Particle filter of each hypothesis

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.max_particles` | `int` | `200` | Maximum number of particles of a hypothesis. |
| `<plugin>.min_particles` | `int` | `30` | Minimum number of particles of a hypothesis. |
| `<plugin>.particles_step` | `int` | `30` | Particles added (bad quality) or removed (good quality) on every reseed. |
| `<plugin>.reseed_freq` | `double` | `0.33` | Reseed frequency (Hz). |
| `<plugin>.reseed_percentage_losers` | `double` | `0.8` | Fraction of particles replaced on every reseed. |
| `<plugin>.reseed_percentage_winners` | `double` | `0.03` | Fraction of particles (the best ones) that can be parents of the new ones. |
| `<plugin>.reseed_noise_xy` | `double` | `0.05` | Std dev of the XY noise of a reseeded particle (m). |
| `<plugin>.reseed_noise_yaw` | `double` | `0.05` | Std dev of the yaw noise of a reseeded particle (rad). |
| `<plugin>.distance_perception_error` | `double` | `0.05` | Precision (sigma) of the sensor (m). Obstacles farther than 3 sigma from a point do not count. |
| `<plugin>.correct_max_points` | `int` | `500` | Maximum number of points per correction (after downsampling). |

### Motion model

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.noise_translation` | `double` | `0.01` | Translational noise factor. |
| `<plugin>.noise_rotation` | `double` | `0.01` | Rotational noise factor. |
| `<plugin>.noise_translation_to_rotation` | `double` | `0.01` | Translation-to-rotation noise coupling. |
| `<plugin>.min_noise_xy` | `double` | `0.05` | Minimum XY noise when the covariance of the initial pose is smaller (m). |
| `<plugin>.min_noise_yaw` | `double` | `0.05` | Minimum yaw noise when the covariance of the initial pose is smaller (rad). |
| `<plugin>.odom_key` | `string` | `odom` | NavState key of the odometry perception. If it is not available, odom->base_footprint is read from the `RTTFBuffer`. |

### Hypotheses

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.multihypothesis` | `bool` | `true` | If false, only the first hypothesis is used (plain AMCL). |
| `<plugin>.max_hypotheses` | `int` | `5` | Maximum number of simultaneous hypotheses. |
| `<plugin>.hypotheses_freq` | `double` | `0.33` | Frequency of the map matching and the hypotheses management (Hz). |
| `<plugin>.min_candidate_weight` | `double` | `0.5` | Minimum match of a map pose to be a candidate to a new hypothesis. |
| `<plugin>.min_candidate_distance` | `double` | `1.0` | A candidate closer than this (m) *and* than `min_candidate_angle` to a hypothesis does not create a new one. |
| `<plugin>.min_candidate_angle` | `double` | `1.5708` | See above (rad). |
| `<plugin>.new_hypothesis_std_xy` | `double` | `0.1` | Std dev of the particles of a new hypothesis (m). |
| `<plugin>.new_hypothesis_std_yaw` | `double` | `0.1` | Std dev of the particles of a new hypothesis (rad). |
| `<plugin>.low_q_hypo_threshold` | `double` | `0.25` | Below it a hypothesis is low quality: it is removed if the maximum number of hypotheses is reached, and its particles grow on reseed. |
| `<plugin>.very_low_q_hypo_threshold` | `double` | `0.1` | Below it a hypothesis is always removed (unless it is the last one). |
| `<plugin>.hypo_merge_distance` | `double` | `0.3` | Two hypotheses closer than this (m) *and* than `hypo_merge_angle` are merged. |
| `<plugin>.hypo_merge_angle` | `double` | `0.5` | See above (rad). |
| `<plugin>.good_hypo_threshold` | `double` | `0.6` | Minimum quality to become the selected hypothesis. Above it, the particles shrink on reseed. |
| `<plugin>.min_hypo_diff_winner` | `double` | `0.2` | A hypothesis must be this much better than the selected one to replace it. |
| `<plugin>.matcher_levels` | `int` | `4` | Levels of the resolution pyramid of the map matching. |
| `<plugin>.matcher_angle_step` | `double` | `0.3927` | Angular resolution of the map matching (rad). |

Note: the map matching cost grows with the size of the map and `1 / matcher_angle_step`. It runs in a
background thread, but on very large maps consider lowering `hypotheses_freq` or increasing the step.

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Discards all the hypotheses and starts one at the received pose, using its covariance. | depth=10 |
| Publisher | `<node_fqn>/<plugin>/particles` | `geometry_msgs/msg/PoseArray` | Particles of the selected hypothesis. | depth=10 |
| Publisher | `<node_fqn>/<plugin>/hypotheses` | `visualization_msgs/msg/MarkerArray` | Particles (one color each) and pose of every hypothesis. The selected one has a thicker arrow. Only built if someone subscribes. | depth=10 |
| Publisher | `<node_fqn>/<plugin>/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Pose with covariance of the selected hypothesis. | depth=10 |

### Services

This package does not create service servers or clients.

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `points` | `PointPerceptions` | **Read** | Perception point clouds used in correction and map matching. |
| `map.base` | `Costmap2D` | **Read** | Base costmap. `LETHAL_OBSTACLE` cells are the obstacles, and hypotheses must be in `FREE_SPACE` cells. |
| `odom` (`<plugin>.odom_key`) | `nav_msgs::msg::Odometry` | **Read** | Odometry perception, stored by the `OdometryPerceptionHandler` of `easynav_sensors`. Optional: without it, TF is used. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Write** | Pose of the selected hypothesis. |

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | `map -> odom` | Aligns the odometry frame with the map frame, using the selected hypothesis. |
| Requires (optional) | `odom -> base_footprint` | Only if there is no odometry perception in NavState. |
| Requires | `base_footprint -> <sensor_frame>` | To bring the perceptions to the robot frame. |

## License

Apache-2.0
