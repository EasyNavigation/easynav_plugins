# easynav_bigmap_maps_manager

A maps manager for NavMaps too big to plan on at full detail (a warehouse, a town): the map is
fine around the robot and coarse far from it, and the obstacles the sensors see are added near the
robot and forgotten when it is far from them.

Planning on it, against planning on the full map: 2 x 2 km around the Puerta del Sol, Madrid
(ground from the IGN, buildings from the Catastro, 13.8 M NavCels at 0.5 m), 30 routes of 230 to
1481 m, the same A* and safety band on every map, the robot replanning every 30 m on the map made
around it (the `city_routes` campaign of
[bigmap_experiments](https://github.com/jmguerreroh/bigmap_experiments)):

| Map | NavCels | MB | Success | Through a building | Length | A* per plan | NavCels expanded |
|---|---|---|---|---|---|---|---|
| full, 0.5 m | 13,765,636 | 321 | 30/30 | 0 | 1.000 | 351 ms | 1,166,402 |
| grid 1 m | 3,275,294 | 77 | 21/30 | 0 | 1.006 | 58 ms | 190,060 |
| grid 2 m | 744,768 | 18 | 28/30 | 0 | 1.008 | 16 ms | 71,510 |
| grid 4 m | 156,118 | 3.9 | 13/30 | 0 | 1.354 | 6.7 ms | 24,202 |
| full map simplified once (0.4 m) | 393,251 | 10.3 | 30/30 | 0 | 1.022 | 14 ms | 35,082 |
| bigmap (0.8 to 8 m, 20 to 300 m) | ~135,000 | 3.5 | 30/30 | 0 | 1.007 | 5.6 ms | 8,528 |

Paths within 1 % of those on the full map, planned 63 times faster, expanding 137 times fewer
NavCels, on a map 92 times smaller, made again (with the planner's graph) in 226 ms every 30 m.
Coarser fixed grids close streets (1, 2 and 4 m fail routes) or go around them (4 m: 35 % longer);
the map simplified once as a whole finds every route, 2.5 times slower to plan on.

## Configuration

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [bigmap]
    bigmap:
      plugin: easynav_bigmap_maps_manager/BigMapMapsManager
      package: easynav_large_warehouse            # optional: navmap_path_file is relative to it
      navmap_path_file: maps/large_warehouse_collision.navmap
      cell_size:
        min: 0.5          # m: NavCels at the obstacles near the robot
        max: 8.0          # m: the largest NavCel; far away the map is within max / 2
      min_radius: 15.0    # m: within it, the obstacles have NavCels of cell_size.min
      max_radius: 60.0    # m: from it, the map is as coarse as it gets
      forget_distance: 60.0   # m: sensed obstacles farther than this are forgotten (default max_radius)
      filters: [inflation]
      inflation:
        plugin: easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter
        inflation_radius: 2.0
        cost_scaling_factor: 1.5
```

That is all it needs. A few more parameters have defaults that suit most robots:

| Parameter | Default | |
|---|---|---|
| `obstacle_max_height` | 2.0 m | higher points do not touch the robot: set it to the robot's height (e.g. so that the shelves of a rack it passes under are not obstacles) |
| `obstacle_min_height` | 0.1 m | lower points above the map's surface are the ground |
| `obstacle_min_height_per_meter` | 0.02 | `obstacle_min_height` grows this much per metre from the robot: a slight tilt of the robot or the sensor puts the far ground above it |
| `sensor_range` | `min_radius` | points farther from the robot are not used (at most `forget_distance`) |
| `rebuild_period` | 0.5 s | the map is made again at most this often |
| `memory.clearing` | true | the rays from the robot to what it sees clear the sensed obstacles they cross (one that moves away stops blocking) |
| `memory.p_hit`, `memory.p_miss` | 0.7, 0.4 | probability that a cell is an obstacle after a point on it, and after a ray through it (as OctoMap): how many sightings mark it, and clear it |
| `memory.decay_time` | 0 s | a sensed obstacle not seen again for this long is forgotten (0: only `forget_distance` and the clearing forget it) |

The filters are those of `easynav_navmap_maps_manager`, run on the map every cycle. The inflation
radius also sets how far around an obstacle the NavCels are fine (see below).

## How

**Once, at start** (seconds, on every core: 9 s for the 2.5 M NavCels of the warehouse, 45 s for the
14 M of 2 x 2 km of Madrid, with cells of 0.5 to 16 m in the benchmark of bigmap_experiments):

1. The NavMap is cut along a grid (`navmap::cut_along_lines`) into square tiles of `min_radius / 2`.
2. Each tile is simplified (`navmap::simplify`) within `cell_size.min / 2`, no edge longer than
   `cell_size.max`: level 0.
3. Level k: four tiles of level k - 1 put together and simplified within twice the error, up to
   `cell_size.max / 2` (a quadtree; 0.5 / 1 / 2 / 4 / 8 m are five levels).

`simplify` keeps the map's topology (no hole closes, no wall opens, however coarse) and its layers.
The corners of a tile and the points where the map's border leaves its sides are kept
(`SimplifyParams::keep_vertices`), so its sides only lose vertices along them.

**Every cycle**, when the robot has moved a few metres or the sensed obstacles changed (at most
twice a second):

1. **Select** the coarsest tile each place allows: level 0 within `min_radius`, the coarsest from
   `max_radius`, geometric in between.
2. **Assemble** them in one NavMap. Where a tile has a vertex on a side its neighbour does not,
   the neighbour's NavCel there is split to reach it, so the map is conforming (no cracks) and the
   planner crosses from tile to tile.
3. **Detail** (`navmap::refine`, longest-edge bisection): within `min_radius`, the borders of the
   map (walls, racks, the edges of the holes) and the sensed obstacles get NavCels of
   `cell_size.min`, and so does every NavCel within the inflation radius of them (the cost of a
   NavCel is that of its centroid: a large one next to an obstacle would read as free). Free
   ground away from them stays as large as it was. Farther obstacles get the cell size of their
   place.
4. The sensed obstacles are marked `LETHAL_OBSTACLE` in the layer `obstacles`; the filters run.

| | Warehouse (2.5 M NavCels) | Madrid, 2 x 2 km (13.8 M) |
|---|---|---|
| Map around the robot | ~27 k NavCels (~35 k with the detail) | ~157 k |
| Assemble | 7 ms | 45-65 ms |
| Detail | 30 ms | 70-190 ms |

**Sensed obstacles**: the points of the perceptions within `sensor_range`, in the robot's footprint
frame, between `obstacle_min_height` (plus `obstacle_min_height_per_meter` per metre) and
`obstacle_max_height` above the map's surface under them. A point over no surface, or within
`cell_size.min` of a place with none (a rack, a building, seen from the side), is the map's own
obstacle and is left out. They are remembered by cells of `cell_size.min / 2` as evidence in
log-odds, as the NavMap maps manager's obstacle memory: a point on a cell is a hit, a ray from the
robot through it (to the ground or to an obstacle beyond) a miss, so an obstacle that moves away is
cleared once the robot sees through where it was. A cell is an obstacle while it is more likely
occupied than not; it is forgotten when the robot is `forget_distance` from it (or, with
`memory.decay_time`, when it is not seen again for that long).

## Topics

| Topic | Type | |
|---|---|---|
| `<node>/<plugin>/map` | `navmap_ros_interfaces/msg/NavMap` | the map, when it changes (at most once a second) |
| `<node>/<plugin>/map_updates` | `navmap_ros_interfaces/msg/NavMapLayer` | what the filters change, as the NavMap maps manager |
| `<node>/<plugin>/stats` | `std_msgs/msg/Float64MultiArray` | every cycle, while someone listens: time (s), rebuilds so far, the last rebuild's select, assemble and detail (ms) and NavCels, this cycle's sensing and filters (ms), sensed obstacles |

## From NavMap

It is built on NavMap's tools: `simplify` (with `keep_vertices`), `cut_along_lines` and `refine`
(`navmap_core/Subdivide.hpp`).

## Tests

`colcon test --packages-select easynav_bigmap_maps_manager`: the engine (`test_tiled_lod`: levels,
no cracks, holes and narrow passages kept far away, layers) and the plugin as EasyNav runs it
(`test_bigmap_plugin`: fine at the borders near the robot and coarse far, the map follows the robot,
a sensed obstacle is added fine, inflated and forgotten far away, and cleared when it moves away).
