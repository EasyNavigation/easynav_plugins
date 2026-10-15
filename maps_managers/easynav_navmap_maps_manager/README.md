# easynav_navmap_maps_manager

## Description

Maps Manager that maintains a [NavMap](https://github.com/EasyNavigation/NavMap) (triangulated 3D surface) and publishes full maps and layer updates; supports importing from YAML OccupancyGrid or point clouds.

This package also includes map filters implemented as plugins (`ObstacleFilter` and `InflationFilter`) that operate on NavMap layers to detect obstacles and inflate their costs, enabling cost-aware path planning.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab  
- **Maintainer:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_navmap_maps_manager/NavMapMapsManager`
- **Type:** `easynav::navmap::NavMapMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_navmap_maps_manager`
- **Description:**  
  Maps Manager that maintains a NavMap (triangulated 3D surface) and publishes full maps and layer updates; supports importing from YAML OccupancyGrid or point clouds, and applying dynamic filters to generate obstacle and inflated layers.

## Parameters

All parameters are declared under the plugin namespace, i.e.  
`/<node_fqn>/easynav_navmap_maps_manager/NavMapMapsManager/...`

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name to resolve relative paths via `ament_index`. |
| `<plugin>.freq` | `double` | `10.0` | Update frequency (Hz) for map publishing and filter execution. |
| `<plugin>.navmap_path_file` | `string` | `""` | Relative path (inside the package) to a `.navmap` file to load at startup. |
| `<plugin>.occmap_path_file` | `string` | `""` | Relative path (inside the package) to a ROS YAML OccupancyGrid to import as NavMap. |
| `<plugin>.pcd_path_file` | `string` | `""` | Relative path (inside the package) to a point cloud (PCD/PLY) used to build a NavMap surface. |
| `<plugin>.filters` | `list<string>` | `[]` | Ordered list of filter plugin names to be applied after map loading (e.g. `["obstacles", "inflation"]`). |

### Local mode

With `<plugin>.local.enabled`, the maps manager keeps a NavMap around the robot, in its frame, rebuilt continuously from the point perceptions in NavState (scans accumulated in `odom` over `accumulate_time`, cropped to the window and meshed with `navmap_ros::from_points`). Each rebuild is published on `<plugin>/map` and replaces the map in NavState; the filters run on it as usual (the ObstacleFilter works in the NavMap's own frame). A map loaded from a file is replaced at the first rebuild.

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.local.enabled` | `bool` | `false` | Enable the local mode. |
| `<plugin>.local.frame` | `string` | robot frame | Frame of the NavMap (e.g. `base_footprint`, ground at z = 0). |
| `<plugin>.local.range` | `double` | `10.0` | Half side (m) of the square window around the robot. |
| `<plugin>.local.min_z`, `<plugin>.local.max_z` | `double` | `-3.0`, `3.0` | Height band (m, in `frame`) of the points used. |
| `<plugin>.local.accumulate_time` | `double` | `1.0` | Scans put together before meshing (s): a single 3D scan is sparse rings. |
| `<plugin>.local.freq` | `double` | `2.0` | Rebuilds per second (`<= 0`: every update). |
| `<plugin>.local.downsample` | `double` | `0.1` | Voxel (m) the accumulated points are reduced to. |
| `<plugin>.local.resolution`, `max_dz`, `max_slope_deg`, `min_clearance`, `min_surface_area`, `max_hole_area` | `double` | `0.3`, `0.25`, `30`, `1.0`, `1.0`, `1.0` | Mesher parameters (`navmap_ros::BuildParams`). |

### Filter Plugins

#### **ObstacleFilter**

- **Plugin Name:** `easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter`
- **Type:** `easynav::navmap::ObstacleFilter`
- **Description:**  
  Detects occupied NavCels from the point perceptions in `NavState` and marks them as `LETHAL_OBSTACLE` in the `"obstacles"` layer, on top of the static map's `"occupancy"` layer (if any).  
  The points are downsampled, those within `max_range` and below `max_height` (robot frame) are grouped in 0.3 m columns, and a column is an obstacle when its highest point is more than `min_height` (plus `min_height_per_meter` for each meter from the robot) above the NavMap surface under it (along the surface's normal). So a 2D laser detects obstacles too, and the ground or a ramp, being on the surface, is not one.  
  The layer is rebuilt from the static map only when the map changes; each cycle writes just the cells that become obstacles or stop being one.  
  With `memory.enabled`, the evidence is accumulated per NavCel in log-odds (`"obstacles_logodds"`, int16, clamped as OctoMap): a point above the surface is a hit, the cells a ray from the robot crosses (and where a ground point lies) are misses, and unobserved cells can decay. `"obstacles_sensor"` says what the sensors know (obstacle, free or unknown), and `"obstacles"` is the static map with the sensors on top (`navmap::combine_layer`, max): the sensors fill in cells the static map does not know, and never erase a static obstacle. `NO_INFORMATION` is the declared unknown value of these layers.

| Parameter | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.max_range` | `double` | `10.0` | Only points within this distance (x and y, robot frame) are used. |
| `<plugin>.min_height` | `double` | `0.1` | Height above the NavMap surface below which points are the ground. |
| `<plugin>.min_height_per_meter` | `double` | `0.0` | `min_height` grows this much per meter of horizontal distance from the robot (`< 0` or not finite: `0`). On rough terrain, a small error in the robot's tilt lifts the ground more the farther it is; e.g. `0.05` adds 0.25 m at 5 m. |
| `<plugin>.max_height` | `double` | `NaN` | Points above this height (robot frame) are ignored; `NaN` disables it. |
| `<plugin>.downsample_resolution` | `double` | `0.3` | Voxel size (m) the points are reduced to first; `<= 0` disables it. |
| `<plugin>.memory.enabled` | `bool` | `false` | Remember obstacles that are out of sight (see below). Off: only what is seen in the current cycle, on top of the static map. |
| `<plugin>.memory.clearing` | `bool` | `true` | Memory: rays from the robot to each point, along the surface, are evidence of free space in the cells they cross. |
| `<plugin>.memory.decay_time` | `double` | `0.0` | Memory: seconds for a cell no longer observed to fade back to unknown (from the most certain occupied); `0` never. |
| `<plugin>.memory.p_hit`, `memory.p_miss` | `double` | `0.7`, `0.4` | Memory: occupancy probability of a hit and of a cell a ray crosses (as OctoMap). |
| **Input Key:** | | | Reads list of point clouds from `NavState` group `"points"`. |
| **Output Layer:** | | | Updates or creates NavMap layer `"obstacles"`. |

#### **InflationFilter**

- **Plugin Name:** `easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter`
- **Type:** `easynav::navmap::InflationFilter`
- **Description:**  
  Expands obstacle information from the `"obstacles"` layer into an `"inflated_obstacles"` layer, assigning graded costs depending on distance to obstacles and map boundaries (NavCels with missing neighbors).  
  This filter mimics the behavior of `costmap_2d::InflationLayer` but operates on the NavMap triangular mesh. Distances are exact and measured along the surface (`navmap::DistanceField`), and they are updated incrementally: only the obstacle cells that appear or disappear move the field, and only the cells whose cost changes are rewritten.

| Parameter | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.inflation_radius` | `double` | `0.3` | Maximum inflation distance (m) from obstacles. |
| `<plugin>.cost_scaling_factor` | `double` | `3.0` | Exponential decay rate controlling how fast cost decreases with distance. |
| `<plugin>.inscribed_radius` | `double` | — | **Deprecated**: the robot's `system_node.robot_geometry.inscribed_radius`, used unless only this one is configured (with a warning). Radius of inscribed zone (constant high cost before decay). |
| **Input Layer:** | | | Reads from `"obstacles"`. |
| **Output Layer:** | | | Writes to `"inflated_obstacles"`. |

The filters update the map in `NavState` in place and record which cells they change; the maps manager publishes those changes and clears them at the end of each cycle.

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/map` | `navmap_ros_interfaces/msg/NavMap` | Publishes the full NavMap. | depth=1 |
| Publisher | `<node_fqn>/<plugin>/map_updates` | `navmap_ros_interfaces/msg/NavMapLayer` | Layer updates of the filters: only the cells that changed (sparse `NavMapLayer`, or the whole layer if many changed), only when someone subscribes; a new subscriber gets the whole layers. | depth=100 |
| Subscription | `<node_fqn>/<plugin>/incoming_occ_map` | `nav_msgs/msg/OccupancyGrid` | Input occupancy grid to import into NavMap. | depth=1, transient_local, reliable |
| Subscription | `<node_fqn>/<plugin>/incoming_pc2_map` | `sensor_msgs/msg/PointCloud2` | Input point cloud to build/update NavMap. | depth=100 |

### Services

| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Saves the current NavMap and layers to disk. |

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `map.navmap` | `::navmap::NavMap` | **Read** | Reads the NavMap if already present in NavState. |
| `map.navmap` | `::navmap::NavMap` | **Write** | Stores the maintained NavMap, including generated layers `"obstacles"` and `"inflated_obstacles"`. |

## TF Frames

| Role | Transform | Notes |
|---|---|---|
| Publishes | — | No TF broadcasting in this manager; outputs are stamped in their configured frame. |

## Example Configuration

```yaml
maps_manager_node:
  ros__parameters:
    use_sim_time: true
    map_types: [navmap]
    navmap:
      freq: 10.0
      plugin: easynav_navmap_maps_manager/NavMapMapsManager
      package: easynav_indoor_testcase
      navmap_path_file: maps/excavation_urjc.navmap
      filters: [obstacles, inflation]
      obstacles:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/ObstacleFilter
        height_threshold: 0.25
      inflation:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter
        inflation_radius: 1.0
        cost_scaling_factor: 2.0
```

## License

Apache-2.0
