# easynav_simple_maps_manager

## Description

Simple Maps Manager that demonstrates the Maps Manager API. It forwards/republishes a basic occupancy map flow and exposes standard topics and a save-map service.

At the heart of this stack is the SimpleMap data structure. It represents the environment as a 2D occupancy grid where each cell can be either 0 (free), 1 (occupied), or -1 (unknown).

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

- **Plugin Name:** `easynav_simple_maps_manager/SimpleMapsManager`
- **Type:** `easynav::SimpleMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_simple_maps_manager`
- **Description:** Simple (no-op) Maps Manager that demonstrates the Maps Manager API. It forwards/republishes a basic occupancy map flow and exposes standard topics and a save-map service.

## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/easynav_simple_maps_manager/SimpleMapsManager/...`)

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name used to resolve relative map paths via `ament_index`. |
| `<plugin>.map_path_file` | `string` | `""` | Relative path (inside the package) to a simple map. |

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `<node_fqn>/<plugin>/incoming_map` | `nav_msgs/msg/OccupancyGrid` | Incoming occupancy map (used to feed the dynamic map). | QoS depth=1, transient_local, reliable |
| Publisher | `<node_fqn>/<plugin>/map` | `nav_msgs/msg/OccupancyGrid` | Published static map. | QoS depth=1 |
| Publisher | `<node_fqn>/<plugin>/dynamic_map` | `nav_msgs/msg/OccupancyGrid` | Published dynamic (live) map. | QoS depth=100 |

### Services

| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Saves current map(s) to disk. |

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `points` | `PointPerceptions` | **Read** | Optional perception points bundle (not strictly required for this no-op manager). |
| `map.static` | `SimpleMap` | **Write** | Static map loaded from file / parameter configuration. |
| `map.dynamic` | `SimpleMap` | **Write** | Dynamic map after applying incoming updates. |

## TF Frames

This manager does not broadcast TF; messages are stamped directly with their frame as provided.

## License

Apache-2.0
