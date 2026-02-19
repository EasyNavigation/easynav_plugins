# easynav_mpc_controller

[![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#) 


## Description
A Model Predictive Controller (MPC) implementation for Easy Navigation.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Juan S. Cely G. <juanscelyg@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_mpc_controller/MPCController`
- **Type:** `easynav::MPCController`
- **Base Class:** `easynav::ControllerMethodBase`
- **Library:** `easynav_mpc_controller`
- **Description:** A Model Predictive Controller (MPC) implementation for Easy Navigation.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_mpc_controller/MPCController/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.horizon_steps` | `int` | `5` | Number of time steps in the prediction horizon. |
| `<plugin>.dt` | `double` | `0.1` | Integration time step (seconds). |
| `<plugin>.safety_radius` | `double` | `0.35` | Safety radius to check possible collisions. |
| `<plugin>.max_linear_velocity` | `double` | `1.5` | Maximum linear velocity (m/s). |
| `<plugin>.max_angular_velocity` | `double` | `1.5` | Maximum angular velocity (rad/s). |
| `<plugin>.verbose` | `bool` | `false` | Show data on terminal about Optimization. |


## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `/mpc/path` | `nav_msgs/msg/path` | MPC trajectories generate by Predictive Component. | QoS depth=10 |
| Publisher | `/mpc/detection` | `sensor_msgs::msg::PointCloud2` | Detections used by collision checker. | QoS depth=10 |


### Services
This package does not create service servers or clients.


## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `path` | `nav_msgs::msg::Path` | **Read** | Target path to track. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose/state. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Read** | Last commanded velocity (if provided in state). |
| `points` | `PointPerceptions` | **Read** | Perception point cloud(s) used for costs. |


## TF Frames
This controller does not explicitly publish or require TF frames in code.

## License
Apache-2.0
