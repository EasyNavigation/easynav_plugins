# easynav_navmap_maps_manager

## Description

Maps Manager that maintains a [NavMap](https://github.com/EasyNavigation/NavMap) (triangulated 3D surface) and publishes full maps and layer updates; supports importing from YAML OccupancyGrid or point clouds.

This package also includes map filters implemented as plugins (`ObstacleFilter`, `InflationFilter` and `FractalFilter`) that operate on NavMap layers to detect obstacles and inflate their costs, enabling cost-aware path planning, and keep a simplified copy of the map for planning on large maps.

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

#### **FractalFilter**

- **Plugin Name:** `easynav_navmap_maps_manager/NavMapMapsManager/FractalFilter`
- **Type:** `easynav::navmap::FractalFilter`
- **Description:**  
  Keeps a simplified copy of `map.navmap` for planning on large maps: neighbour NavCels with the same value of `layer` merge into larger triangles, and split back down to the source resolution where the value changes. Free space far from obstacles becomes a few large NavCels, while obstacles, the inflation band around them and changes of slope stay at full resolution, so the planner explores far fewer NavCels and still sees every narrow passage.  
  `map.navmap` is not modified: the other filters keep working on it at full resolution, and the simplified map is written to another NavState key (`output_key`, `map.navmap.lod`). The planner reads it through its own `map_key` parameter (e.g. `easynav_navmap_planner/AStarPlanner`'s `map_key: map.navmap.lod`).  
  It goes **after** the filters that write its `layer` (`filters: [obstacles, inflation, fractal]`).

  How it works: a binary triangle tree (RTIN) is laid over the source map. Two sibling triangles merge into their parent (twice the area) when every cell below has the same value of `layer` (strict equality), is flat (slopes within `max_angle_deg`, vertices within `height_tolerance` of the merged plane) and has no hole. Otherwise the triangle is split along its hypotenuse; a split forces the neighbour across the hypotenuse to split too, so the mesh has no T-junctions and the planner can cross between levels. Each cycle only reads the costs within `window_radius` of the robot (where the sensors change them) plus `sweep` NavCels of the whole map, taken in turns (what changes far away, such as obstacles an obstacle memory forgets, is caught within a few seconds). The cost per cycle does not depend on the size of the map. The output map is rebuilt in place by tiles of about 50 m: only the tiles where something changed are walked and written again.

  Requirements: the source must be a regular grid of right isosceles triangles (two per square cell), as `pointcloud_to_navmap` and `navmap_ros::from_occupancy_grid` produce, and `layer` must be a `uint8` layer. Otherwise the filter logs an error and writes no output.

  For RViz, the simplified map is published on `<node_fqn>/<map_type>/<filter>/map` (e.g. `/maps_manager_node/navmap/fractal/map`), and between two full maps only what changed is published on `.../map_patch` (`navmap_ros_interfaces/msg/NavMapPatch`), which the NavMap display applies. Nothing is published without subscribers. The `lod_level` layer (merges per NavCel, 0 = source resolution) and the `modified` layer show how much was simplified; turn on the display's *Draw Edges* to see the triangles.

| Parameter | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.output_key` | `string` | `"map.navmap.lod"` | NavState key of the simplified map; must not be `map.navmap`. |
| `<plugin>.layer` | `string` | `"inflated_obstacles"` | `uint8` layer whose value decides what merges; it is copied to the simplified map. |
| `<plugin>.max_angle_deg` | `double` | `5.0` | NavCels merge only if their slopes differ at most this (deg). |
| `<plugin>.height_tolerance` | `double` | `0.05` | Source vertices at most this far (m) from the plane of a merged NavCel. |
| `<plugin>.window_radius` | `double` | `8.0` | Costs read every cycle within this distance (m) of the robot: at least the sensor range plus the inflation radius. |
| `<plugin>.sweep` | `int` | `100000` | NavCels of the whole map read per cycle, in turns; `0` reads only the window. |
| `<plugin>.rebuild_period` | `double` | `0.5` | Minimum time (s) between two rebuilds of the simplified map. |
| `<plugin>.tile_size` | `double` | `50.0` | Side (m, about) of the tiles the simplified map is rebuilt by. |
| `<plugin>.publish_period` | `double` | `2.0` | Minimum time (s) between two publications for RViz (map or patch). |
| `<plugin>.full_period` | `double` | `0.0` | Time (s) between two full maps (resync); `0`: only for a new subscriber or a rebuilt map, patches otherwise. |
| `<plugin>.log_period` | `double` | `10.0` | Time (s) between two logs of the simplified map's statistics; `0`: every rebuild. |
| **Input:** | | | Reads `map.navmap` and its `layer`, and `robot_pose` for the window. |
| **Output Key:** | | | Writes `output_key` (`::navmap::NavMap`), with layers `<layer>`, `lod_level` and `modified`. |

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
| `map.navmap.lod` | `::navmap::NavMap` | **Write** | With `FractalFilter`: the simplified map for planning (its `output_key`). |

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
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter
        height_threshold: 0.25
      inflation:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter
        inflation_radius: 1.0
        cost_scaling_factor: 2.0
```

### With FractalFilter (large maps)

The maps manager keeps the full map and its simplified copy; A* plans on the copy:

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [navmap]
    navmap:
      freq: 10.0
      plugin: easynav_navmap_maps_manager/NavMapMapsManager
      package: my_robot_maps
      navmap_path_file: maps/warehouse.navmap
      filters: [obstacles, inflation, fractal]   # fractal after the filters that write its layer
      obstacles:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter
        max_range: 6.0
        max_height: 1.5            # m: ignore what the robot passes under (door lintels)
      inflation:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter
        inflation_radius: 1.5
        cost_scaling_factor: 2.0
      fractal:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/FractalFilter
        output_key: map.navmap.lod   # where A* reads (its map_key)
        layer: inflated_obstacles    # neighbour NavCels with the same value merge
        max_angle_deg: 5.0
        height_tolerance: 0.05
        window_radius: 8.0           # sensor range (6 m) + inflation radius (1.5 m)
        sweep: 100000                # the whole map is read again every few seconds
        rebuild_period: 0.5
        publish_period: 2.0          # for RViz: full map, then patches

planner_node:
  ros__parameters:
    planner_types: [astar]
    astar:
      freq: 1.0
      plugin: easynav_navmap_planner/AStarPlanner
      map_key: map.navmap.lod      # plan on the simplified map
```

In RViz, show `/maps_manager_node/navmap/fractal/map` with the NavMap display (its *Patch Topic* defaults to `<topic>_patch`).

## License

Apache-2.0
