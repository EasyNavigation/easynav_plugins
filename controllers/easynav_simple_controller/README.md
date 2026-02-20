# easynav_simple_controller

## Description

Simple path-following controller that uses PID controllers and a look-ahead reference pose to follow a planned path. It produces velocity commands (`cmd_vel`) based on the reference pose sampled at a look-ahead distance and limits linear/angular speeds and accelerations.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab  
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---:|
| humble | ![kilted](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![kilted](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_simple_controller/SimpleController`
- **Type:** `easynav::SimpleController`
- **Base Class:** `easynav::ControllerMethodBase`
- **Library:** `easynav_simple_controller`
- **Description:** Path-following controller using PID (linear and angular) and a look-ahead strategy.

## Parameters

All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_simple_controller/SimpleController/...`.

> This plugin derives from [`easynav::ControllerMethodBase`](https://github.com/EasyNavigation/EasyNavigation/tree/rolling/easynav_core#easynavcontrollermethodbase).  \
> See that section for shared collision-checking parameters and debug markers common to all controllers.

| Name | Type | Default | Description |
|---|---|---:|---|
| `max_linear_speed` | `double` | `1.0` | Maximum linear speed (m/s). |
| `max_angular_speed` | `double` | `1.0` | Maximum angular speed (rad/s). |
| `max_linear_acc` | `double` | `0.3` | Maximum linear acceleration (m/s²). |
| `max_angular_acc` | `double` | `0.3` | Maximum angular acceleration (rad/s²). |
| `look_ahead_dist` | `double` | `1.0` | Look-ahead distance to sample the reference pose on the path (m). |
| `tolerance_dist` | `double` | `0.05` | Distance threshold to switch to pure orientation tracking (m). |
| `final_goal_angle_tolerance` | `double` | `0.1` | Angular tolerance (rad) used to decide final-goal arrival. |
| `k_rot` | `double` | `0.5` | Gain used to reduce linear speed based on angular velocity (higher: stronger reduction while turning). |
| `linear_kp` | `double` | `0.95` | Proportional gain for the linear PID controller. |
| `linear_ki` | `double` | `0.03` | Integral gain for the linear PID controller. |
| `linear_kd` | `double` | `0.08` | Derivative gain for the linear PID controller. |
| `angular_kp` | `double` | `1.5` | Proportional gain for the angular PID controller. |
| `angular_ki` | `double` | `0.03` | Integral gain for the angular PID controller. |
| `angular_kd` | `double` | `0.08` | Derivative gain for the angular PID controller. |

## Interfaces (NavState, Topics and Services)

### NavState

This controller uses the shared `NavState` bag provided by the `easynav_core` framework. The following keys are used at runtime by `SimpleController`:

| Key | Type | Access | Notes |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot odometry used to compute the robot pose and yaw. |
| `path` | `nav_msgs::msg::Path` | **Read** | Planned path to follow. The controller samples a reference pose at `look_ahead_dist` along this path. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** | Output velocity command. Header.frame_id is set to `path.header.frame_id` when available and stamp to the controller node clock. |

### Topics / Services

The controller itself does not create ROS publishers/subscribers or service servers. It interacts via the `NavState` abstraction; how `NavState` is exposed (topics or other IPC mechanisms) depends on the integrating node.

## TF Frames

This controller reads pose from `nav_msgs/Odometry` (NavState key `robot_pose`). TF is not directly used in this plugin.

## License

Apache-2.0
