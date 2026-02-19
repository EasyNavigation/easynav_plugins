# easynav_costmap_maps_manager

## Description

Maps Manager that maintains 2D costmaps (static and dynamic), supports filter plugins (such as inflation and obstacle filters), and exposes maps through ROS topics and NavState integration.

At the core of this stack lies the Costmap2D data structure. `Costmap2D` extends the binary occupancy grid into a graded cost representation with values in the range [0–255]:

- 0: Free space, no cost to traverse.
- 1–252: Gradual cost values, representing increasing difficulty or proximity to obstacles.
- 253: “Near obstacle” (inscribed obstacle) cost, traversal strongly discouraged.
- 254: Lethal obstacle, occupied cell.
- 255: Unknown space.

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

- **Plugin Name:** `easynav_costmap_maps_manager/CostmapMapsManager`
- **Type:** `easynav::CostmapMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_costmap_maps_manager`
- **Description:** Maintains a Costmap2D instance and manages map loading, updates, and filtering operations.

---

## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/easynav_costmap_maps_manager/CostmapMapsManager/...`)

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name used to resolve relative map paths via `ament_index`. |
| `<plugin>.map_path_file` | `string` | `""` | Relative path (inside the package) to a ROS 1–style YAML map (with `image`, `resolution`, `origin`, etc.). |
| `<plugin>.filters` | `string[]` | `[]` | List of filter identifiers to be instantiated (see section below). |
| `<plugin>.<filter>.plugin` | `string` | `""` | Type of filter plugin (e.g., `easynav_costmap_maps_manager/InflationFilter`). |

---

### Filter Plugins

Each entry in `<plugin>.filters` defines a sub-namespace `<plugin>.<filter>` with at least the key `plugin`, plus any filter-specific parameters.

#### **ObstacleFilter**

- **Plugin Name:** `easynav_costmap_maps_manager/ObstacleFilter`
- **Type:** `easynav::ObstacleFilter`
- **Description:**  
  Detects occupied cells from input point clouds (`points` key in `NavState`) and marks them as `LETHAL_OBSTACLE` in the dynamic costmap.  
  The filter fuses incoming 3D points into the map frame, downsamples them to the costmap resolution, filters out ground-level points (z < 0.1 m), and sets corresponding cells to lethal cost. Additionally, it computes and stores bounding box (`ObstacleBounds`) of updated obstacles to enable efficient incremental inflation.

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---:|---|
| _(None)_ | — | — | This filter does not declare additional ROS parameters beyond `plugin`. Downsampling resolution and frame fusion use the costmap's own resolution and `map` frame. |

**NavState Keys:**

| Key | Type | Access | Description |
|---|---|---|---|
| `points` | `sensor_msgs::msg::PointCloud2` | **Read** | Input point clouds to detect obstacles. |
| `map.dynamic.filtered` | `Costmap2D` | **Write** | Marks cells as `LETHAL_OBSTACLE` (254). |
| `map.dynamic.obstacle_bounds` | `ObstacleBounds` | **Write** | Bounding box of updated obstacles for incremental inflation. |

#### **InflationFilter**

- **Plugin Name:** `easynav_costmap_maps_manager/InflationFilter`
- **Type:** `easynav::InflationFilter`
- **Description:**  
  Expands obstacle information in the costmap by assigning graded costs around `LETHAL_OBSTACLE` cells based on distance. Uses a breadth-first wavefront propagation algorithm (distance bins) to efficiently inflate obstacles up to `inflation_radius`.  
  The filter reads both the static map and the dynamic filtered map, applies inflation to each, and merges results. If `ObstacleBounds` is available in `NavState`, inflation is restricted to the updated region for performance.

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.inflation_radius` | `double` | `0.3` | Maximum inflation distance (m) from obstacles. Cells farther than this receive no inflation cost. |
| `<plugin>.inscribed_radius` | `double` | `0.25` | Radius of the inscribed zone (m). Cells within this distance of an obstacle are marked with high constant cost (`INSCRIBED_INFLATED_OBSTACLE`, value 253) before exponential decay begins. |
| `<plugin>.cost_scaling_factor` | `double` | `3.0` | Exponential decay rate controlling how quickly cost decreases with distance beyond the inscribed radius. Higher values produce steeper cost gradients. |

**NavState Keys:**

| Key | Type | Access | Description |
|---|---|---|---|
| `map.static` | `Costmap2D` | **Read** | Static costmap to inflate. |
| `map.dynamic.filtered` | `Costmap2D` | **Read/Write** | Dynamic costmap input and output after inflation. |
| `map.dynamic.obstacle_bounds` | `ObstacleBounds` | **Read** (optional) | Restricts inflation to updated region for performance. |

**Cost Model:**  
Uses exponential decay: `cost = exp(-cost_scaling_factor * (distance - inscribed_radius)) * 253` for distances beyond inscribed radius.

### Example Configuration

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [costmap]
    costmap:
      plugin: easynav_costmap_maps_manager/CostmapMapsManager
      package: my_maps_pkg
      map_path_file: maps/warehouse.yaml
      filters: [inflation, obstacles]
      inflation:
        plugin: easynav_costmap_maps_manager/InflationFilter
        inflation_radius: 0.3         # default in code
        cost_scaling_factor: 3.0      # default in code
      obstacles:
        plugin: easynav_costmap_maps_manager/ObstacleFilter
```

---

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `<node_fqn>/<plugin>/incoming_map` | `nav_msgs/msg/OccupancyGrid` | Input occupancy map used to update the dynamic map. | `depth=1, transient_local, reliable` |
| Publisher | `<node_fqn>/<plugin>/map` | `nav_msgs/msg/OccupancyGrid` | Publishes the static costmap. | `depth=1` |
| Publisher | `<node_fqn>/<plugin>/dynamic_map` | `nav_msgs/msg/OccupancyGrid` | Publishes the dynamic (live) costmap. | `depth=100` |

### Services

| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Saves the current costmap(s) to disk. |

---

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `map.static` | `Costmap2D` | **Write** | Static map loaded from YAML. |
| `map.dynamic` | `Costmap2D` | **Write** | Dynamic map after applying filters. |
| `map.dynamic.filtered` | `Costmap2D` | **Read** | Previously filtered map used as input if available. |
| `map.dynamic.obstacle_bounds` | `ObstacleBounds` | **Read** | Bounding box of updated obstacles (used to limit inflation region). |

---

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | — | This manager does not broadcast TF; costmaps use their internal `frame_id`. |

---

## License

Apache-2.0
