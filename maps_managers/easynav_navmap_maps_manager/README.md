# easynav_navmap_maps_manager

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

## Description
Maps Manager that maintains a [NavMap](https://github.com/EasyNavigation/NavMap) (triangulated 3D surface) and publishes full maps and layer updates; supports importing from YAML OccupancyGrid or point clouds.

## Authors and Maintainers
- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_navmap_maps_manager/NavMapMapsManager`
- **Type:** `easynav::NavMapMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_navmap_maps_manager`
- **Description:** Maps Manager that maintains a NavMap (triangulated 3D surface) and publishes full maps and layer updates; supports importing from YAML OccupancyGrid or point clouds.

## Parameters
All parameters are declared under the plugin namespace, i.e., `/<node_fqn>/easynav_navmap_maps_manager/NavMapMapsManager/...`.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name to resolve relative paths via ament index. |
| `<plugin>.occmap_path_file` | `string` | `""` | Relative path (inside the package) to a ROS YAML OccupancyGrid to import as NavMap. |
| `<plugin>.navmap_path_file` | `string` | `""` | Relative path (inside the package) to a PCD/PLY used to build NavMap. |


## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/map` | `navmap_ros_interfaces/msg/NavMap` | Published full NavMap. | QoS depth=1 |
| Publisher | `<node_fqn>/<plugin>/map_updates` | `navmap_ros_interfaces/msg/NavMapLayer` | Incremental NavMap layer updates. | QoS depth=100 |
| Subscription | `<node_fqn>/<plugin>/incoming_occ_map` | `nav_msgs/msg/OccupancyGrid` | Input occupancy grid to import into NavMap. | QoS depth=1, transient_local, reliable |
| Subscription | `<node_fqn>/<plugin>/incoming_pc2_map` | `sensor_msgs/msg/PointCloud2` | Input point cloud to build/update NavMap. | QoS depth=100 |


### Services
| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Save current NavMap to disk. |


## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `map` | `::navmap::NavMap` | **Read** | Existing NavMap from state, if available. |
| `map.navmap` | `::navmap::NavMap` | **Write** | Stores/broadcasts the maintained NavMap. |


## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | `—` | No TF broadcasting in this manager; outputs are stamped in their configured frame. |


## License
GPL-3.0-only
