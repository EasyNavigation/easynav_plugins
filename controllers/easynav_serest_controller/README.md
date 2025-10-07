# easynav_serest_controller

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
A SeReST (Smooth Error-Responsive Speed and Turning) controller for path tracking.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_serest_controller/SerestController`
- **Type:** `easynav::SerestController`
- **Base Class:** `easynav::ControllerMethodBase`
- **Library:** `easynav_serest_controller`
- **Description:** A SeReST (Smooth Error-Responsive Speed and Turning) controller for path tracking.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_serest_controller/SerestController/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.a_acc` | `double` | `0.8` | Comfortable forward acceleration (m/s²). |
| `<plugin>.a_brake` | `double` | `1.2` | Comfortable braking deceleration (m/s²). |
| `<plugin>.a_lat_max` | `double` | `1.5` | Maximum lateral acceleration (m/s²). |
| `<plugin>.a_lat_soft` | `double` | `1.1` | Soft lateral acceleration bound (m/s²). |
| `<plugin>.allow_reverse` | `bool` | `false` | Allow reversing when beneficial. |
| `<plugin>.apex_ey_des` | `double` | `0.05` | Desired lateral error at apex for shaping (m). |
| `<plugin>.blend_base` | `double` | `0.6` | Base blending factor for curvature tracking. |
| `<plugin>.blend_k_per_v` | `double` | `0.6` | Additional blending proportional to speed. |
| `<plugin>.corner_boost_omega` | `double` | `0.8` | Omega boost when cornering. |
| `<plugin>.corner_gain_eth` | `double` | `0.7` | Cornering gain for heading error. |
| `<plugin>.corner_gain_ey` | `double` | `1.5` | Cornering gain for lateral error. |
| `<plugin>.corner_gain_kappa` | `double` | `0.4` | Cornering gain for curvature error. |
| `<plugin>.corner_guard_enable` | `bool` | `true` | Enable cornering guard logic. |
| `<plugin>.corner_min_alpha` | `double` | `0.35` | Minimum blending near corners. |
| `<plugin>.d0_margin` | `double` | `0.30` | Base obstacle clearance margin (m). |
| `<plugin>.d_hard` | `double` | `0.20` | Hard-stop distance threshold (m). |
| `<plugin>.dist_search_radius` | `double` | `2.0` | Search radius when matching to path (m). |
| `<plugin>.ell` | `double` | `` | Lookahead distance (m). |
| `<plugin>.eps` | `double` | `1e-3` | Small epsilon to avoid division by zero. |
| `<plugin>.final_align_k` | `double` | `2.0` | Gain used during final alignment. |
| `<plugin>.final_align_wmax` | `double` | `0.6` | Max angular speed during final alignment (rad/s). |
| `<plugin>.goal_pos_tol` | `double` | `0.05` | Goal position tolerance (m). |
| `<plugin>.goal_yaw_tol_deg` | `double` | `5.0` | Goal yaw tolerance (degrees). |
| `<plugin>.k_s` | `double` | `` | Gain on progress along path (s). |
| `<plugin>.k_s_share_max` | `double` | `0.5` | Max share of speed from s-progress term. |
| `<plugin>.k_theta` | `double` | `` | Heading error gain. |
| `<plugin>.k_y` | `double` | `` | Lateral error gain. |
| `<plugin>.kappa_max` | `double` | `2.5` | Maximum allowed curvature (1/m). |
| `<plugin>.max_angular_acc` | `double` | `2.0` | Angular acceleration limit (rad/s²). |
| `<plugin>.max_angular_speed` | `double` | `1.5` | Angular speed limit (rad/s). |
| `<plugin>.max_linear_acc` | `double` | `0.8` | Linear acceleration limit (m/s²). |
| `<plugin>.max_linear_speed` | `double` | `0.6` | Speed limit (m/s). |
| `<plugin>.slow_min_speed` | `double` | `0.03` | Minimum speed within slow zone (m/s). |
| `<plugin>.slow_radius` | `double` | `0.60` | Radius to start slowing down near goal (m). |
| `<plugin>.t_emerg` | `double` | `0.25` | Emergency stop time horizon (s). |
| `<plugin>.tau_latency` | `double` | `0.10` | Actuation latency (s). |
| `<plugin>.v_progress_min` | `double` | `0.05` | Minimum forward speed to ensure progress (m/s). |
| `<plugin>.v_ref` | `double` | `0.6` | Nominal reference speed (m/s). |


## Interfaces (Topics and Services)

### Subscriptions and Publications
This controller communicates through `NavState` (no direct ROS topics in this plugin).


### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `path` | `nav_msgs::msg::Path` | **Read** | Reference path. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Robot odometry. |
| `points` | `PointPerceptions` | **Read** | Obstacle point cloud(s). |
| `closest_obstacle_distance` | `double` | **Read** | Precomputed dynamic obstacle metric. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** | Commanded velocity. |
| `serest.debug.e_y` | `double/int` | **Write** | Debug metric |
| `serest.debug.e_theta` | `double/int` | **Write** | Debug metric |
| `serest.debug.kappa_hat` | `double/int` | **Write** | Debug metric |
| `serest.debug.d_closest` | `double/int` | **Write** | Debug metric |
| `serest.debug.v_safe` | `double/int` | **Write** | Debug metric |
| `serest.debug.v_curv` | `double/int` | **Write** | Debug metric |
| `serest.debug.alpha` | `double/int` | **Write** | Debug metric |
| `serest.debug.allow_reverse` | `double/int` | **Write** | Debug metric |
| `serest.debug.dist_to_end` | `double/int` | **Write** | Debug metric |
| `serest.debug.goal.dist_xy` | `double/int` | **Write** | Debug metric |
| `serest.debug.goal.gamma_slow` | `double/int` | **Write** | Debug metric |
| `serest.debug.goal.in_final_align` | `double/int` | **Write** | Debug metric |
| `serest.debug.goal.arrived` | `double/int` | **Write** | Debug metric |


## TF Frames
This controller reads pose from `nav_msgs/Odometry` (NavState key `robot_pose`). TF is not directly used in this plugin.

## License
GPL-3.0-only
