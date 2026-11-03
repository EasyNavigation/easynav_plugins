# easynav_mpc_controller

[![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#) 


## Description

Linear time-varying Model Predictive Controller (MPC) for differential, omnidirectional, Ackermann and aerial robots. Each cycle it predicts `horizon_steps` steps of `dt` and solves a sparse quadratic program with [OSQP](https://osqp.org) (`osqp_vendor`).

- **Motion models** (`motion_model`): `diff` (x, y, yaw; `linear.x` and `angular.z`), `omni` (also `linear.y`), `ackermann` (like `diff`, with the turning radius) and `aerial` (x, y, z, yaw; also `linear.z`). The model is linearized around a nominal trajectory: the previous solution shifted one step, or the reference when the robot lags behind it. `time_constant` models how the robot's velocity lags behind the command.
- **Reference:** the path is projected forward from the robot (never backward) and given a speed profile within the robot's limits, the path's curvature (`max_lateral_acc`, `max_angular_vel`), the goal and the obstacles ahead, where it stops. For robots with a turning radius, corners are smoothed (`path_smoothing`).
- **Constraints:** velocity and acceleration of each axis, from the robot limits; `|angular.z| <= linear.x / min_turning_radius` for Ackermann; and, for each step, half-planes that keep the robot's shape (`robot_geometry`, the footprint or the radius) `safety_margin` away from the nearest obstacles. The bounds always include the nominal trajectory, so the program always has a solution.
- **At the goal:** within `xy_goal_tolerance` it turns to the goal's heading (robots that can turn in place) and stops. A differential robot facing away from the path turns in place first (`rotate_to_heading_angle`).

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Juan S. Cely G. <juanscelyg@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_mpc_controller/MPCController`
- **Type:** `easynav::MPCController`
- **Base Class:** `easynav::ControllerMethodBase`
- **Library:** `easynav_mpc_controller`
- **Description:** Linear time-varying MPC (OSQP) for differential, omnidirectional, Ackermann and aerial robots.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/<plugin>/...`. Invalid values fail to configure.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.motion_model` | `string` | `diff` | `diff`, `omni`, `ackermann` or `aerial`. |
| `<plugin>.horizon_steps` | `int` | `20` | Steps in the prediction horizon (1-500). |
| `<plugin>.dt` | `double` | `0.1` | Time of each step (s). |
| `<plugin>.time_constant` | `double` | `0.0` | First-order lag of the robot's velocity behind the command (s). 0: it follows the command at once. |
| `<plugin>.latency` | `double` | `0.0` | Delay until a command acts (s): the robot is predicted forward this long before solving. |
| `<plugin>.use_odometry_velocity` | `bool` | `false` | Take the current velocity from `robot_pose`'s twist instead of from the commands sent (through `time_constant`). |
| `<plugin>.weight_xy` | `double` | `10.0` | Weight of the position error (per m²). |
| `<plugin>.weight_z` | `double` | `10.0` | Weight of the height error (aerial). |
| `<plugin>.weight_yaw` | `double` | `2.0` | Weight of the heading error (per rad²). |
| `<plugin>.terminal_factor` | `double` | `5.0` | Weight of the last pose, relative to the others. |
| `<plugin>.near_weight` | `double` | `1.0` | The pose weights grow linearly along the horizon from this factor (first step) to 1. Below 1, the robot is pressed less to be on the path right now than further ahead: less weaving. In (0, 1]. |
| `<plugin>.weight_linear_vel` | `double` | `0.5` | Weight of the linear velocity off the reference's. |
| `<plugin>.weight_angular_vel` | `double` | `0.2` | Weight of the angular velocity off the reference's. |
| `<plugin>.weight_linear_rate` | `double` | `1.0` | Weight of the change of the linear command between steps. |
| `<plugin>.weight_angular_rate` | `double` | `1.0` | Weight of the change of the angular command between steps. |
| `<plugin>.max_lateral_acc` | `double` | `1.0` | Lateral acceleration (speed² × curvature, m/s²) the reference's speed keeps to in curves. |
| `<plugin>.approach_speed` | `double` | `0.1` | Lowest reference speed (m/s) while the goal is not reached. |
| `<plugin>.curvature_window` | `double` | `1.0` | Half length (m) of the window the path's heading is averaged over: planners' paths zigzag on their grid. |
| `<plugin>.path_smoothing` | `double` | `-1.0` | Rounding of the path's corners (m). Negative: automatic, 1.45 × `min_turning_radius` for robots with a turning radius (rounds a 90° corner to that radius), none for the rest. |
| `<plugin>.obstacle_range` | `double` | `3.0` | Minimum distance (m) obstacles are taken from; it grows with the distance the horizon covers at full speed plus the braking distance. |
| `<plugin>.obstacles_per_step` | `int` | `2` | Half-planes per step, from obstacles in different directions. |
| `<plugin>.safety_margin` | `double` | `0.1` | Clearance (m) between the robot's shape and the obstacles. |
| `<plugin>.min_height` | `double` | `0.1` | Perceived points lower than this (m, robot frame) are the floor, not obstacles (ground robots). Lower it for sensors mounted lower. |
| `<plugin>.xy_goal_tolerance` | `double` | `0.2` | Distance (m) to the goal at which the robot stops moving along the path. `goal_tolerance.position` in NavState, if lower, takes over. |
| `<plugin>.yaw_goal_tolerance` | `double` | `0.2` | Heading error (rad) at the goal to stop turning. `goal_tolerance.yaw` in NavState, if lower, takes over. |
| `<plugin>.goal_align_gain` | `double` | `1.0` | Angular velocity per radian of heading error when turning in place (at the goal, or towards the path). |
| `<plugin>.rotate_to_heading_angle` | `double` | `0.785` | A differential robot whose heading is farther than this (rad) from the path's turns in place first, until it is within half of it. |
| `<plugin>.max_iterations` | `int` | `400` | OSQP iterations per cycle: bounds the solve time. |
| `<plugin>.tolerance` | `double` | `0.001` | OSQP absolute and relative tolerance. |

Velocity and acceleration limits are not this plugin's: they are the robot limits of `controller_node` (`robot_limits.*`: linear, lateral, vertical and angular velocity, acceleration and deceleration, and `min_turning_radius`), queried with `ControllerMethodBase::get_robot_limits()` and also enforced by ControllerNode's velocity smoother. The robot's shape is `robot_geometry` (footprint or radius, and height).

- `ackermann` without `robot_limits.min_turning_radius` warns: nothing keeps the curvature within the steering.
- `aerial` with `robot_limits.max_vertical_vel` 0 warns: it will not change height.

> **Deprecated:** `fallback_goal_pos_tol` and `fallback_goal_yaw_tol` still apply, with a warning, where `xy_goal_tolerance` and `yaw_goal_tolerance` are not given; so do the former limit parameters (`max_linear_velocity`, `max_angular_velocity`) where `robot_limits.*` does not set that limit. `safety_radius`, `use_collision_checker` and `verbose` no longer have any effect and only warn: obstacles are constraints, from `robot_geometry` and `safety_margin`.

## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node>/<plugin>/predicted` | `nav_msgs/msg/Path` | The poses the solution leads to. | depth 10 |
| Publisher | `<node>/<plugin>/reference` | `nav_msgs/msg/Path` | The reference the solution follows. | depth 10 |
| Publisher | `<node>/<plugin>/obstacles` | `sensor_msgs/msg/PointCloud2` | The obstacle points taken into account. | depth 10 |

### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `path` | `nav_msgs::msg::Path` | **Read** | Path to follow; its last pose is the goal. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current pose (and velocity, with `use_odometry_velocity`). |
| `goal_tolerance.position`, `goal_tolerance.yaw` | `double` | **Read** | The current goal's tolerances, if set. |
| Point perceptions | `PointPerception` | **Read** | Obstacles. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** | The command. |

## TF Frames
The obstacle points are fused in the robot frame (to keep those around the robot) and in the map frame.

## License
Apache-2.0
