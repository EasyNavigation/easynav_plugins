# easynav_mppi_controller

## Description

A Model Predictive Path Integral (MPPI) controller implementation for Easy Navigation.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![kilted](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_mppi_controller/MPPIController`
- **Type:** `easynav::MPPIController`
- **Base Class:** `easynav::ControllerMethodBase`
- **Library:** `easynav_mppi_controller`
- **Description:** A Model Predictive Path Integral (MPPI) controller implementation for Easy Navigation.

## Parameters

All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_mppi_controller/MPPIController/...`.

> This plugin derives from [`easynav::ControllerMethodBase`](https://github.com/EasyNavigation/EasyNavigation/tree/rolling/easynav_core#easynavcontrollermethodbase).  \
> See that section for shared collision-checking parameters and debug markers common to all controllers.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.num_samples` | `int` | `100` | Number of trajectory rollouts per iteration. |
| `<plugin>.horizon_steps` | `int` | `10` | Number of time steps in the prediction horizon. |
| `<plugin>.dt` | `double` | `0.1` | Integration time step (seconds). |
| `<plugin>.lambda` | `double` | `0.1` | Temperature / control noise scaling factor. |
| — | — | — | Velocity and acceleration limits are not this plugin's: they are the robot limits of `controller_node` (`robot_limits.max_linear_vel`, `min_linear_vel`, `max_angular_vel`, `max_linear_acc`, `max_linear_decel`, `max_angular_acc`, `max_angular_decel`), queried with `ControllerMethodBase::get_robot_limits()` and also enforced by ControllerNode's velocity smoother. |
| `<plugin>.fov` | `double` | `M_PI/2.0` | Field of view used in trajectory sampling (radians). |
| `<plugin>.safety_radius` | `double` | `0.6` | Safety radius around the robot (meters). |
| `<plugin>.obstacle_range` | `double` | `2.0` | Obstacle points are taken (robot frame) from just behind the robot (`robot_geometry.radius`) up to this distance ahead, and this far to each side; mirrored when moving backward. |
| `<plugin>.z_min_filter` | `double` | `0.0` | Points below this height (robot frame) are the ground; above `robot_geometry.height` they are ignored too. |

> **Deprecated:** this plugin's former limit parameters (`max_linear_velocity`, `max_angular_velocity`, `max_linear_acceleration`, `max_angular_acceleration`, under the plugin's name) still apply, with a warning, where `controller_node.robot_limits.*` does not set that limit. They will stop working soon: move them to `robot_limits`.

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `/mppi/candidates` | `visualization_msgs/msg/MarkerArray` | MPPI candidate trajectories as markers. | QoS depth=10 |
| Publisher | `/mppi/optimal_path` | `visualization_msgs/msg/MarkerArray` | Optimal MPPI trajectory as markers. | QoS depth=10 |

### Services

This package does not create service servers or clients.

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `path` | `nav_msgs::msg::Path` | **Read** | Target path to track. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose/state. |
| `points` | `PointPerceptions` | **Read** | Perception point cloud(s) used for costs. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Read** | Last commanded velocity (if provided in state). |

## TF Frames

This controller does not explicitly publish or require TF frames in code.

## License

Apache-2.0
