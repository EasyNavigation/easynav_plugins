# easynav_octomap_maps_manager

## Description

Maps Manager that maintains an [OctoMap](https://octomap.github.io/) (probabilistic 3D occupancy tree), supports filter plugins (e.g., inflation and obstacle filters), and exposes the map through ROS 2 topics. This README documents **plugin-level parameters only**.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab  
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_octomap_maps_manager/OctomapMapsManager`
- **Type:** `easynav::octomap::OctomapMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_octomap_maps_manager`
- **Description:** Maintains an OctoMap instance, handles map loading/import, point-cloud ingestion, and filter application.

---

## Parameters

### Plugin Parameters

The current code (OctomapMapsManager.cpp) has the high-level parameter declarations for package and map paths commented out. Active runtime parameters are therefore limited to filter plugin namespaces populated by entries in `filters` (if/when enabled). Until the declarations are restored, only filter-local parameters are honored.

### Filter Parameters

Each entry in `<plugin>.filters` defines a sub-namespace `<plugin>.<filter>` with at least the key `plugin`, plus any specific parameters.

#### InflationFilter

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.inflation.inflation_radius` | `double` | `0.3` | Inflation radius (m) used to expand occupied cells. |
| `<plugin>.inflation.cost_scaling_factor` | `double` | `3.0` | Exponential decay controlling the reduction of cost with distance. |
| `<plugin>.inflation.inscribed_radius` | `double` | `0.3` | Inscribed radius (m) used by the inflation model. |

#### ObstacleFilter

This filter does not declare additional ROS parameters apart from `plugin`.

## Example Configuration

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [octomap]
    octomap:
      plugin: easynav_octomap_maps_manager/OctomapMapsManager
      package: my_maps_pkg
      octomap_path_file: maps/site.pcd
      filters: [inflation, obstacles]
      inflation:
        plugin: easynav_octomap_maps_manager/InflationFilter
        inflation_radius: 0.3
        cost_scaling_factor: 3.0
        inscribed_radius: 0.3
      obstacles:
        plugin: easynav_octomap_maps_manager/ObstacleFilter
```

---

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/map` | `octomap_msgs/msg/Octomap` | Publishes the current OctoMap. | depth=1 |
| Subscription | `<node_fqn>/<plugin>/incoming_pc2_map` | `sensor_msgs/msg/PointCloud2` | Input point cloud used to build/update the OctoMap. | depth=100 |

### Services

| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Saves the current OctoMap to disk. |

---

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `map` | `::octomap::Octomap` | **Read** | If present in NavState, used as an input/seed map. (Plugin currently does not write back to NavState.) |

---

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | — | This manager does not broadcast TF; outputs are stamped using the configured frame(s) in the map data. |

---

## License

Apache-2.0
