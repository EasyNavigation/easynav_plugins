# easynav_vff_controller

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
Vector Field Histogram (VFF) controller for obstacle avoidance and goal tracking.  
Implements a reactive navigation approach that computes motion commands based on the vector sum of attractive and repulsive forces derived from goal and obstacle information.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab  
- **Maintainers:** Jose Miguel Guerrero Hernández <josemiguel.guerrero@urjc.es>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_vff_controller/VffController`  
- **Type:** `easynav::VffController`  
- **Base Class:** `easynav::ControllerMethodBase`  
- **Library:** `vff_controller`  
- **Description:** Reactive controller that generates velocity commands using the Vector Field Histogram method.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_vff_controller/VffController/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.distance_obstacle_detection` | `float` | `3.0` | Distance threshold for obstacle detection. |
| `<plugin>.distance_to_goal` | `float` | `1.0` | Minimum distance to consider the goal reached. |
| `<plugin>.obstacle_detection_x_min` | `float` | `0.5` | Minimum X limit for obstacle detection (m). |
| `<plugin>.obstacle_detection_x_max` | `float` | `10.0` | Maximum X limit for obstacle detection (m). |
| `<plugin>.obstacle_detection_y_min` | `float` | `-10.0` | Minimum Y limit for obstacle detection (m). |
| `<plugin>.obstacle_detection_y_max` | `float` | `10.0` | Maximum Y limit for obstacle detection (m). |
| `<plugin>.obstacle_detection_z_min` | `float` | `0.10` | Minimum Z limit for obstacle detection (m). |
| `<plugin>.obstacle_detection_z_max` | `float` | `1.00` | Maximum Z limit for obstacle detection (m). |
| `<plugin>.max_speed` | `double` | `0.8` | Maximum linear velocity (m/s). |
| `<plugin>.max_angular_speed` | `double` | `1.5` | Maximum angular velocity (rad/s). |

## Interfaces (Topics and Services)

### Subscriptions and Publications
This controller communicates exclusively through `NavState`; it does not create direct ROS 2 publishers or subscribers.

### Services
This package does not create service servers or clients.

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `goals` | `nav_msgs::msg::Goals` | **Read** | Target goals to reach. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose used to compute forces. |
| `points` | `PointPerceptions` | **Read** | Point cloud of obstacles. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** | Output velocity command computed by VFF. |

## TF Frames
This controller reads pose from `nav_msgs/Odometry` (`robot_pose` key). No TF lookups are performed.

## License
GPL-3.0-only
