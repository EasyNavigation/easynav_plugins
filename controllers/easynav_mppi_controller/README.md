# easynav_mppi_controller

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
A Model Predictive Path Integral (MPPI) controller implementation for Easy Navigation.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
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

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.num_samples` | `int` | `100` | Number of trajectory rollouts per iteration. |
| `<plugin>.horizon_steps` | `int` | `10` | Number of time steps in the prediction horizon. |
| `<plugin>.dt` | `double` | `0.1` | Integration time step (seconds). |
| `<plugin>.lambda` | `double` | `0.1` | Temperature / control noise scaling factor. |
| `<plugin>.max_linear_velocity` | `double` | `1.0` | Maximum linear velocity (m/s). |
| `<plugin>.max_angular_velocity` | `double` | `1.0` | Maximum angular velocity (rad/s). |
| `<plugin>.max_linear_acceleration` | `double` | `0.5` | Maximum linear acceleration (m/s²). |
| `<plugin>.max_angular_acceleration` | `double` | `1.0` | Maximum angular acceleration (rad/s²). |
| `<plugin>.fov` | `double` | `M_PI/2.0` | Field of view used in trajectory sampling (radians). |
| `<plugin>.safety_radius` | `double` | `0.6` | Safety radius around the robot (meters). |


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
GPL-3.0-only
