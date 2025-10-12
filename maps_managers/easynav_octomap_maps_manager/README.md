# easynav_octomap_maps_manager

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#) [![ROS 2: rolling](https://img.shields.io/badge/ROS%202-rolling-blue)](#)

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

### Plugin Parameters (namespace: `/<node_fqn>/easynav_octomap_maps_manager/OctomapMapsManager/...`)
| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name used to resolve relative map paths via `ament_index`. |
| `<plugin>.occmap_path_file` | `string` | `""` | Relative path (inside the package) to a ROS YAML occupancy grid to import as OctoMap. |
| `<plugin>.octomap_path_file` | `string` | `""` | Relative path (inside the package) to a PCD/PLY/OT file used to build/load the OctoMap. |
| `<plugin>.filters` | `string[]` | `[]` | List of filter identifiers to be instantiated (see “Filter Parameters”). |
| `<plugin>.<filter>.plugin` | `string` | `""` | Type of filter plugin (e.g., `easynav_octomap_maps_manager/InflationFilter`). |

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

**Example Configuration**

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
| Publisher | `<node_fqn>/<plugin>/map` | `octomap_msgs/msg/Octomap` | Publishes the current OctoMap. | `depth=1` |
| Subscription | `<node_fqn>/<plugin>/incoming_pc2_map` | `sensor_msgs/msg/PointCloud2` | Input point cloud used to build/update the OctoMap. | `depth=100` |

### Services
| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Saves the current OctoMap to disk. |

---

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `map` | `::octomap::Octomap` | **Read** | If present in NavState, used as input/seed map. This plugin does not write a NavState key. |

---

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | — | This manager does not broadcast TF; outputs are stamped using the configured frame(s) in the map data. |

---

## License
GPL-3.0-only
