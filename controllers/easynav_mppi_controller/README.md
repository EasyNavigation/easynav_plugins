# easynav_mppi_controller

## Description

A Model Predictive Path Integral (MPPI) controller for Easy Navigation, for differential and
holonomic (omnidirectional, mecanum) robots, round or not.

- G. Williams, A. Aldrich, E. Theodorou, **Model Predictive Path Integral Control: From Theory to Parallel Computation**. Journal of Guidance, Control, and Dynamics, 2017.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Algorithm

Every cycle, the controller keeps a sequence of `horizon_steps` velocity commands (`dt` seconds
each), the solution of the previous cycle shifted by the time elapsed since. It simulates
`num_samples` rollouts of the robot:

- the previous solution, and braking to a stop;
- deterministic candidates: arcs, turns in place, turning and then going straight (differential),
  straight lines in eight directions (holonomic), and straight to the goal braking to stop there
  when the goal is within reach;
- the rest, the previous solution plus noise correlated in time (`noise_correlation`), so that
  the rollouts really turn instead of zigzagging; a tenth of them around zero, to explore.

Each rollout respects the robot's velocity and acceleration limits, and is scored (lower is
better):

| Term | Weight | |
|---|---|---|
| Distance to the path | `path_weight` | From the path pose nearest the robot, as far as the rollouts reach |
| Path left to the end of that local path | `progress_weight` | Rewards going far along the path, soon |
| Distance to the goal, when within reach | `goal_weight` | |
| Heading error to the heading of `heading_mode` | `heading_weight` | Gives way to the goal's heading within `goal_heading_distance` of the goal (holonomic) |
| Heading error to the goal's heading, near it | `goal_heading_weight` | Holonomic |
| Within the safety margin of an obstacle | `obstacle_weight` | Quadratic, from `safety_radius - radius` to touching it |
| Touching an obstacle | `collision_cost` | Per step: such a rollout is discarded |
| Backward velocity | `forward_weight` | Differential |
| Accelerations | `smoothness_weight` | |

When an obstacle lies on the path within reach, the path and heading weights drop to a tenth: the
path is only a guide around it. The new solution is the average of the rollouts weighted by
`exp(-(cost - best) / lambda)`; when that average is worse than the best rollout (two equally good
options, such as turning left or right, average to a bad one), the best rollout is the solution.
Its first command is sent.

At the goal position (half the goal tolerance, or the tolerance once stopped), the robot turns in
place to the goal's heading, braking in time; within both tolerances it stops. The tolerances are
the strictest of this controller's and the system's (`goal_tolerance.*` in NavState).

### Differential and holonomic robots

`motion_model` selects the robot:

- `diff`: commands `linear.x` and `angular.z`. It always faces along the path.
- `omni`: commands `linear.x`, `linear.y` and `angular.z`. The sequence is kept in the map frame,
  so turning does not change the course: the robot moves along the path and turns as
  `heading_mode` says, independently. Its translation is limited by the ellipse of
  `max_linear_vel` (`min_linear_vel` backwards) and `max_lateral_vel`: no direction is faster than
  the configured ones, diagonals included.

`heading_mode` sets the orientation of a holonomic robot while it moves (a differential robot
always uses `path`). The goal's orientation is always reached at the end.

| `heading_mode` | Orientation while moving |
|---|---|
| `path` | Facing the path ahead (`heading_lookahead`); it corrects sideways |
| `free` | Any: the one that costs least |
| `goal` | The goal's, from the start |
| `hold` | The robot's when the goal was received |
| `fixed` | `heading_yaw` (map frame) |

While the NavState key `heading_reference` (`double`, rad, map frame) exists, a holonomic robot
keeps that heading instead, whatever `heading_mode` says (e.g. set by a behavior tree).

### Obstacles and the robot's shape

The obstacles are the perception points within `obstacle_range` of the robot, all around it, from
`z_min_filter` to `robot_geometry.height`. Their distances are computed once per cycle in a grid
(`grid_resolution`), so each rollout step looks them up in constant time.

With `robot_geometry.footprint`, the robot's polygon is checked, turned with each rollout: a long
robot does not sweep a wall turning in place, and a holonomic one turns to cross a door narrower
than its length. The radii of the footprint are used, not the configured ones (which may be
planning margins). Without a footprint, the robot is a circle of `robot_geometry.radius`.

## Plugin (pluginlib)

- **Plugin Name:** `easynav_mppi_controller/MPPIController`
- **Type:** `easynav::MPPIController`
- **Base Class:** `easynav::ControllerMethodBase`
- **Library:** `easynav_mppi_controller`

## Parameters

All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_mppi_controller/MPPIController/...`.

> This plugin derives from [`easynav::ControllerMethodBase`](https://github.com/EasyNavigation/EasyNavigation/tree/rolling/easynav_core#easynavcontrollermethodbase).  \
> See that section for shared collision-checking parameters and debug markers common to all controllers.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.motion_model` | `string` | `diff` | `diff` or `omni`. |
| `<plugin>.heading_mode` | `string` | `path` | `path`, `free`, `goal`, `hold` or `fixed` (holonomic robots). |
| `<plugin>.heading_yaw` | `double` | `0.0` | Heading of `heading_mode: fixed` (rad, map frame). |
| `<plugin>.num_samples` | `int` | `500` | Rollouts per cycle. |
| `<plugin>.horizon_steps` | `int` | `40` | Steps of each rollout. |
| `<plugin>.dt` | `double` | `0.1` | Duration of each step (s). |
| `<plugin>.lambda` | `double` | `0.05` | Temperature: lower follows the best rollouts more closely. |
| `<plugin>.vx_std`, `vy_std`, `wz_std` | `double` | `0.2`, `0.2`, `0.4` | Sampling noise of each velocity. |
| `<plugin>.noise_correlation` | `double` | `0.8` | Correlation of the noise between steps, [0, 1). |
| `<plugin>.path_weight` | `double` | `15.0` (`omni`: `40.0`) | Distance to the path. |
| `<plugin>.progress_weight` | `double` | `8.0` | Path left to the end of the local path. |
| `<plugin>.heading_weight` | `double` | `10.0` (`omni`: `20.0`) | Error to the heading of `heading_mode`. |
| `<plugin>.heading_lookahead` | `double` | `0.5` | `heading_mode: path` faces the path this far ahead (m). |
| `<plugin>.goal_weight` | `double` | `5.0` | Distance to the goal, when within reach. |
| `<plugin>.goal_heading_weight` | `double` | `3.0` | Error to the goal's heading near it (holonomic). |
| `<plugin>.goal_heading_distance` | `double` | `1.0` | Near the goal, its heading takes over (m). |
| `<plugin>.obstacle_weight` | `double` | `20.0` | Within the safety margin of an obstacle. |
| `<plugin>.smoothness_weight` | `double` | `0.05` | Accelerations. |
| `<plugin>.forward_weight` | `double` | `2.0` | Backward velocity (differential). |
| `<plugin>.safety_radius` | `double` | `0.6` | Rollouts closer than `safety_radius - radius` to an obstacle (from the robot's body) are penalized (m). |
| `<plugin>.obstacle_range` | `double` | `2.0` | Obstacle points are taken within this distance of the robot, in its frame, all around it (m). |
| `<plugin>.z_min_filter` | `double` | `0.0` | Points below this height (robot frame) are the ground; above `robot_geometry.height` they are ignored too. |
| `<plugin>.grid_resolution` | `double` | `0.05` | Resolution of the obstacle and path distance grids (m). |
| `<plugin>.xy_goal_tolerance` | `double` | `0.25` | Positional tolerance to the goal (m). |
| `<plugin>.yaw_goal_tolerance` | `double` | `0.25` | Angular tolerance to the goal (rad). |
| `<plugin>.max_visualized_rollouts` | `int` | `20` | Rollouts published in `/mppi/candidates` (0: none). |
| — | — | — | Velocity and acceleration limits are not this plugin's: they are the robot limits of `controller_node` (`robot_limits.max_linear_vel`, `min_linear_vel`, `max_lateral_vel`, `max_angular_vel`, and their accelerations and decelerations), queried with `ControllerMethodBase::get_robot_limits()` and also enforced by ControllerNode's velocity smoother. A safely limited speed applies at once. |
| — | — | — | The robot's shape is `system_node.robot_geometry` (`radius`, `height`, `footprint`). |

The weights default to those of the motion model: a holonomic robot follows the path closely and
keeps its heading on its own; a differential one steers with its heading, so it needs room to turn
and leave the path. Weights configured explicitly apply to either.

> **Deprecated:** this plugin's former limit parameters (`max_linear_velocity`, `max_angular_velocity`, `max_linear_acceleration`, `max_angular_acceleration`, under the plugin's name) still apply, with a warning, where `controller_node.robot_limits.*` does not set that limit. They will stop working soon: move them to `robot_limits`. `fov` has no effect any more.

### Example: a holonomic robot keeping the goal's heading

```yaml
controller_node:
  ros__parameters:
    robot_limits:
      max_linear_vel: 0.5
      min_linear_vel: -0.5
      max_lateral_vel: 0.5
      max_angular_vel: 1.0
    controller_types: [mppi]
    mppi:
      rt_freq: 30.0
      plugin: easynav_mppi_controller/MPPIController
      motion_model: omni
      heading_mode: goal
      safety_radius: 0.55

system_node:
  ros__parameters:
    robot_geometry:
      footprint: [0.36, 0.25, -0.36, 0.25, -0.36, -0.25, 0.36, -0.25]
      height: 1.6
```

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `/mppi/candidates` | `visualization_msgs/msg/MarkerArray` | Some rollouts of the last cycle (only with subscribers). | QoS depth=10 |
| Publisher | `/mppi/optimal_path` | `visualization_msgs/msg/MarkerArray` | The solution of the last cycle (only with subscribers). | QoS depth=10 |

### Services

This package does not create service servers or clients.

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `path` | `nav_msgs::msg::Path` | **Read** | Target path to track. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose. |
| `points` | `PointPerceptions` | **Read** | Perception point cloud(s): the obstacles. |
| `navigation_state` | `GoalManager::State` | **Read** | Idle: stop. |
| `goal_tolerance.position`, `goal_tolerance.yaw` | `double` | **Read** | The system's goal tolerances. |
| `heading_reference` | `double` | **Read** | Optional: heading kept while moving (holonomic). |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** | The command, in the robot frame. |

## TF Frames

This controller does not explicitly publish or require TF frames in code.

## License

Apache-2.0
