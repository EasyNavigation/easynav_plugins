# easynav_routes_maps_manager

## Description

Maps Manager that maintains a set of navigation routes (in 2D or 3D), exposes them as
visualization markers and interactive markers in RViz, and makes the current
`RoutesMap` available through the NavState so that other managers and filters
can consume it (for example, costmap filters that constrain navigation to a
given corridor).

Routes are represented as a list of straight-line segments between two poses.
Nothing prevents you from using non-zero Z coordinates in the poses, so
routes can be fully 3D if downstream consumers support it.
They are typically loaded from a YAML file stored in a ROS package and can be
edited at runtime using interactive markers. A default single route segment is
created automatically when no routes file is configured.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_routes_maps_manager/RoutesMapsManager`
- **Type:** `easynav::RoutesMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_routes_maps_manager`
- **Description:** Maintains an in-memory set of route segments (RoutesMap),
  loads/saves them from/to YAML files, exposes them through ROS markers and
  interactive markers, and writes the current `RoutesMap` into the NavState.

In addition, this package provides the **RoutesCostmapFilter** plugin, which
can be used by the costmap Maps Manager to raise costs outside the defined
routes, effectively constraining navigation to a configurable corridor around
the segments. Other routes filters (for example, for NavMap or Octomap
representations) can be implemented in the same way, each using its own
NavState keys and outputs.

## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/easynav_routes_maps_manager/RoutesMapsManager/...`)

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name used to resolve relative route YAML paths via `ament_index`. When empty and `<plugin>.map_path_file` is also empty, a default in-memory route is created. |
| `<plugin>.map_path_file` | `string` | `""` | Relative path (inside the package) or absolute path to a YAML file defining the routes (see *Routes YAML Format* below). |
| `<plugin>.filters` | `string[]` | `[]` | List of routes filter identifiers to be instantiated (see *Routes Filter Plugins*). |
| `<plugin>.<filter>.plugin` | `string` | `""` | Type of routes filter plugin (e.g., `easynav_routes_maps_manager/RoutesCostmapFilter`). |

If both `<plugin>.package` and `<plugin>.map_path_file` are empty, the manager
does not attempt to read a file and instead initializes a single default segment
from `(0, 0, 0)` to `(1, 0, 0)`.

If only one of the parameters is set (package without path or path without
package for relative paths), initialization fails with an error.

### Routes YAML Format

Routes are loaded from a YAML file with the following structure:

```yaml
routes: [route0, route1]

route0:
	start: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}
	end:   {x: 1.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}

route1:
	start: {x: 1.0, y: 1.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}
	end:   {x: 2.0, y: 1.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}
```

- `routes`: ordered list of route segment identifiers.
- Each entry under `routeX` defines the start and end poses of a segment.

When saving routes (via the save service, see below), the manager writes back a
file with the same structure.

### Routes Filter Plugins

Each entry in `<plugin>.filters` defines a sub-namespace `<plugin>.<filter>`
with at least the key `plugin`, plus any filter-specific parameters. Filters
implement the `RoutesFilter` interface and are executed after the `RoutesMap`
has been written to the NavState on each update cycle.

#### RoutesCostmapFilter

- **Plugin Name:** `easynav_routes_maps_manager/RoutesCostmapFilter`
- **Type:** `easynav::RoutesCostmapFilter`
- **Description:**
	Reads the current `RoutesMap` and a dynamic costmap (`map.dynamic.filtered`)
	from the NavState and raises the cost of all cells that do not lie within a
	corridor around any route segment. This effectively constrains path planners
	to stay close to the defined routes.

**Parameters (namespace: `/<node_fqn>/.../<plugin>.<filter>/...`):**

| Parameter | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.min_cost` | `int` | `50` | Minimum cost assigned to cells outside the route corridor if their current cost is lower. |
| `<plugin>.route_width` | `double` | `0.0` | Corridor half-width around the route segments in meters. When set to `0.0`, a default width equal to half the costmap cell diagonal is used. |

**NavState usage in RoutesCostmapFilter:**

| Key | Type | Access | Description |
|---|---|---|---|
| `routes` | `RoutesMap` | **Read** | In-memory set of route segments written by `RoutesMapsManager` on each update. |
| `map.dynamic.filtered` | `Costmap2D` | **Read/Write** | Dynamic costmap that is modified in place: cells outside the corridor are raised to at least `min_cost`. |

In addition, the filter publishes a debug `nav_msgs/msg/OccupancyGrid` topic
with the filtered costmap for visualization. Other routes filters may use
different NavState keys or map representations; those should document their
own usage in their respective fichas.

## Interfaces (Topics and Services)

### Subscriptions and Publications

| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/routes` | `visualization_msgs/msg/MarkerArray` | Publishes line and arrow markers representing all route segments for RViz visualization. | `depth=10, transient_local, reliable` |
| Publisher | `<node_fqn>/<plugin>/routes_imarkers` | `visualization_msgs/msg/InteractiveMarker` | Publishes interactive markers for editing route endpoints (creation, removal, and dragging of segment endpoints). | `depth=1` |
| Publisher | `<node_fqn>/<plugin>/routes_map` | `nav_msgs/msg/OccupancyGrid` | (via `RoutesCostmapFilter`) Debug occupancy grid showing the costmap after applying the route-based corridor filter. | `depth=1` |

### Services

| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/save_routes` | `std_srvs/srv/Trigger` | Serializes the current set of routes to the configured YAML file on disk. |

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `routes` | `RoutesMap` | **Write** | In-memory set of route segments loaded from YAML or created interactively. Written by `RoutesMapsManager` on each update. |

## TF Frames

Routes are expressed in the navigation `map` frame (or in the frame configured
in other managers that consume the `RoutesMap`). This plugin does not broadcast
TF transforms; it relies on the rest of the navigation stack to provide the
necessary TF tree.

## License

Apache-2.0
