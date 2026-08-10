# easynav_bonxai_maps_manager

## Description

Maps Manager that maintains a [Bonxai](https://github.com/facontidavide/Bonxai) probabilistic 3D occupancy map and exposes it via ROS topics, NavState, and a save-map service.

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

- **Plugin Name:** `easynav_bonxai_maps_manager/BonxaiMapsManager`
- **Type:** `easynav_bonxai::BonxaiMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_bonxai_maps_manager`
- **Description:** Maps Manager that maintains a Bonxai probabilistic 3D occupancy map and exposes it via ROS topics, NavState, and a save-map service.

## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/easynav_bonxai_maps_manager/BonxaiMapsManager/...`)

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name to resolve relative map paths via ament index. |
| `<plugin>.bonxai_path_file` | `string` | `""` | Relative path (inside the package) to a PCD file with 3D points to build the Bonxai map. |
| `<plugin>.occmap_path_file` | `string` | `""` | Relative path (inside the package) to a ROS-style YAML occupancy map to import. |
| `<plugin>.resolution` | `double` | `0.3` | Voxel resolution for the Bonxai map (meters). |
| `<plugin>.frame_id` | `string` | `"map"` | Frame ID stamped on published map messages. |

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `<node_fqn>/<plugin>/incoming_pc2_map` | `sensor_msgs/msg/PointCloud2` | Input point cloud to build/update the Bonxai map. | QoS depth=100 |
| Subscription | `<node_fqn>/<plugin>/incoming_occ_map` | `nav_msgs/msg/OccupancyGrid` | Input occupancy grid (YAML + PGM) to import into Bonxai. | QoS depth=1, transient_local, reliable |
| Publisher | `<node_fqn>/<plugin>/map` | `sensor_msgs/msg/PointCloud2` | Published Bonxai map as a point cloud. | QoS depth=1, transient_local, reliable |

### Services

| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Save current Bonxai map to disk. |

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `map.bonxai` | `Bonxai::ProbabilisticMap` | **Write** | Stores the current Bonxai map instance. |

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | `—` | No TF broadcast; outputs use the configured frame_id. |

## License

Apache-2.0
