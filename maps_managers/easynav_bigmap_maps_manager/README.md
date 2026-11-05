# easynav_bigmap_maps_manager

An [EasyNav](https://github.com/EasyNavigation) **maps manager plugin** (`easynav_bigmap_maps_manager/BigMapMapsManager`,
parameter namespace `bigmap`) for maps too big to plan on at full resolution. It holds one NavMap
that is **fine around the robot and coarse far away**, re-meshes it as the robot moves, and puts
the obstacles the sensors see on it. Planner and controller plug-ins read it through the NavState
(`map.navmap`) like any other NavMap.

What it gives the rest of the stack:

- a NavMap (layers `occupancy`, `obstacles`, `inflated_obstacles`, `obstacle_hits`, `elevation`,
  `lod_size`, `traversed`, vertex colours) published on `<node>/bigmap/map` and set in the NavState;
- NavCels no bigger than needed: `lengths.cell_size.min` near the robot, up to `cell_size.max` far
  away, and always fine over ramps, narrow passages and the ground the robot has driven over;
- obstacles from the sensors with a memory (`obstacle_hits`), and the NavMap filters of
  `easynav_navmap_maps_manager` (inflation...) applied on top;
- services to save the map and to forget the obstacles (see "Run time").

Two engines, chosen by `multilevel.enabled`: the **grid engine** (outdoor terrain, one height per
x, y: a height grid meshed with RTIN) and the **hierarchy engine** (buildings with floors over each
other: the NavMap's own triangles merged by clustering). The details are in the
`bigmap_bringup` README ("How the NavMap is reduced"), until they move here.

## Use it

In the parameter file of EasyNav, next to the other nodes:

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [bigmap]
    bigmap:
      plugin: easynav_bigmap_maps_manager/BigMapMapsManager
      terrain:
        initial_navmap: maps/my_map.navmap     # where the map comes from: see the next section
```

Everything else has a default. Two complete, documented examples are installed with the package
(`share/easynav_bigmap_maps_manager/config/`, and in `config/` here):

| File | For | Map from | Localization |
|---|---|---|---|
| [config/example_indoor.yaml](config/example_indoor.yaml) | a building, several floors and ramps, real robot | a NavMap file (`terrain.initial_navmap`) | external (anything that gives `map -> odom`) |
| [config/example_outdoor.yaml](config/example_outdoor.yaml) | open terrain with GPS (the simulation's file) | a DEM downloaded once (`terrain.dem_file`) | GPS + IMU + odometry (UKF) |

Each file is a whole EasyNav configuration (system, sensors, localizer, controller, planner, maps
manager); the part of this plugin is `maps_manager_node.ros__parameters.bigmap`, in numbered
sections (2.1 terrain, 2.2 lengths, 2.3 lod, 2.4 traversed, 2.5 obstacles, 2.6 dynamic_map,
2.7 filters, 2.8 output). The header of each file has the table of lengths and says which node
reads what. Run EasyNav with one:

```bash
ros2 run easynav_system system_main --ros-args --params-file <file>
```

The examples are copies of `bigmap_bringup/config/params_indoor.yaml` and `params_outdoor.yaml`,
which the simulations use (a test keeps them equal).

## Where the map comes from

The plugin starts from **one** of these, in this order of precedence (`terrain.*`):

1. `terrain.initial_navmap`: a NavMap file. Any resolution, indoors or outdoors.
2. `terrain.dem_file`: a DEM (height grid) file that `navmap_gis_tool --dem-out` writes.
   The plugin waits `terrain.dem_wait_timeout` s for it to exist.

With neither, the plugin does not start (it says which parameter is missing).

### A NavMap file (indoors, or any place you have a map of)

`terrain.initial_navmap` is a path (`~` allowed), or relative to the share directory of
`terrain.package` if you give one (`package: my_robot_maps`, `initial_navmap: maps/site.navmap`).

- **Grid engine** (`multilevel.enabled: false`, one floor): the NavMap is resampled onto a height
  grid of `terrain.initial_navmap_resolution` m (0 = `lengths.cell_size.min`) centred on the map's
  own (0, 0, 0): keep the map centred. A node is ground if a NavCel covers it; a gap of one node is
  filled, a gap of two or more stays; NaN heights are no data.
- **Hierarchy engine** (`multilevel.enabled: true`, floors over each other): the NavMap's own
  triangles are used, so nothing is resampled. Required for a building with several floors.
- `terrain.drop_unreachable_ground: true` keeps only the largest connected piece of ground the robot
  can drive on (slope <= `terrain.max_slope_deg`), plus the walls that touch it: roofs, plateaus with
  no way up and floors not joined to the rest disappear, so a goal picked there (RViz takes the
  topmost surface under the mouse) is impossible instead of unreachable. Use it in cities and
  buildings, not for maps of separate pieces; the robot must start on the largest one.
- A `traversed` layer in the file starts that ground as known. A saved map's `obstacle_hits`
  counts come back into the obstacle memory.
- `<map>.yaml` next to the file (same name) holds where the map's (0, 0, 0) is on Earth; without
  it the map has no geolocation (origin 0, 0, 0):

  ```yaml
  georeferenced: true
  origin_lat: 40.284796        # WGS84 deg
  origin_lon: -3.821407
  reference_altitude: 650.9    # m ASL of z = 0
  min_resolution: 1.0
  ```

**How to get a NavMap file:**

| You have | Do |
|---|---|
| a point cloud (`.pcd`, LiDAR, SLAM, photogrammetry) | `ros2 run navmap_tools pointcloud_to_navmap --input cloud.pcd --output map.navmap --resolution 0.4 --min-clearance 1.0 --recenter` (below) |
| a map the plugin saved | `ros2 service call /maps_manager_node/bigmap/save_map std_srvs/srv/Trigger` writes `map_all_<date>.navmap` and its `.yaml` in `output.save_directory`; use it as `initial_navmap` |
| a mesh of the terrain (STL) | the DEM generator with `elevation_source: mesh` (outdoors, next section) |
| a NavMap from another tool | use it as it is: `navmap_core` format |

`pointcloud_to_navmap` (package `navmap_tools`, in the NavMap repository) streams clouds of tens of millions of points and writes
`map.navmap` and `map.yaml`. The options that matter:

- `--resolution`: the finest NavCel; use your `lengths.cell_size.min`.
- `--min-clearance`: free height that separates two floors; it must be **below** the headroom of
  the lowest ramp (floors 2.2 m apart: 1.0 to 1.5 keeps them joined, 2.0 cuts the ramp).
- `--max-slope-deg`: surfaces steeper than this stay as gaps (walls).
- `--recenter`: shifts the map so its centre is (0, 0); the shift goes to the `.yaml` as
  `offset_x` / `offset_y`.
- `--origin lat,lon,alt`: makes the map georeferenced (the `.yaml` above).
- `--crop x0,y0,x1,y1`, `--zmin`, `--zmax`: keep only a window (a height window drops roofs and
  canopies).
- Keep the cloud in local coordinates: a PCD stores float32, 0.5 m of precision at UTM northings.

The tool tells you if the data holds the resolution you asked. All options: `--help`.

### Outdoor: getting the map

Outdoors the plugin reads a **DEM file**: a square grid of heights around a GPS origin (the
`DemGrid` of the plugin, `src/dem_grid.cpp`). The tool that builds it, from open data and
once, is `navmap_gis_tool` of the NavMap repository (package `navmap_tools`) with `--dem-out`. All
the sources are free and need no account or API key:

| `--dem-source` | Covers | Resolution | Data |
|---|---|---|---|
| `ign` | Spain | 5 m | IGN terrain model (PNOA-LiDAR); the imagery is the PNOA orthophoto (`--imagery-source pnoa`) |
| `copernicus30` (default) | the whole world | 30 m | Copernicus DEM GLO-30 (AWS Open Data) |
| `mesh` | wherever your STL is | your mesh | `--mesh-file`, in the map's local frame |
| `synthetic` | nowhere (a test terrain) | | offline, to try the rest |

**Step by step** (no launch needed):

1. Pick the place (`--center lat,lon`, WGS84; it becomes the map's (0, 0)) and the size (`--size`,
   the side of the square in metres; the grid has `(size / resolution + 1)^2` nodes), and run the
   tool once:

   ```bash
   ros2 run navmap_tools navmap_gis_tool --center 40.284796,-3.821407 --size 1000 \
       --resolution 1.0 --dem-source ign --dem-out $HOME/.ros/bigmap/elevation_cache.bin
   ```

   - `--resolution`: m between DEM nodes = the smallest NavCel (`lengths.cell_size.min` should be the
     same). The source is sampled at its native spacing (`--dem-data-resolution`, 5 m for `ign`) and
     interpolated: a finer grid is smoother, not more detailed.
   - `--dem-imagery` (with `--imagery-source pnoa|esri|xyz`): paint the satellite image on the
     vertices (RViz colours). They play no part in navigation.
   - `--reference-altitude`: the altitude above sea level that is z = 0; by default the height at
     the origin.
   - The file is written where `--dem-out` says. **It must be
     `maps_manager_node.bigmap.terrain.dem_file`** (the example parameter file has it).
   - If the file exists and was made with the same origin, size, resolution and source it is reused
     and nothing is downloaded (`--force-refresh` builds it again).
2. The tool prints the *reference altitude* (`reference altitude 648.0 m`), the altitude of z = 0.
   Set the UKF origin of the localizer to the `--center` and that altitude:
   `localizer_node.ukf.latitude_origin`, `longitude_origin` and `altitude_origin`
   (`bigmap_bringup`'s launch writes an overrides file with them for you).
3. Run EasyNav with the parameter file. The plugin loads `terrain.dem_file`, builds the NavMap around
   the robot and publishes it:

   ```bash
   ros2 run easynav_system system_main --ros-args --params-file my_outdoor.yaml
   ```

Other things the tool takes:

| Option | What |
|---|---|
| `--dem-source mesh` + `--mesh-file`, `--mesh-world-elevation`, `--mesh-cell` | an STL already in the map's local frame (x east, y north, z up) and the altitude ASL of its z = 0; the area is capped to the mesh's footprint |
| `--terrain-out DIR` + `--terrain-resolution` | also export the DEM as a Gazebo terrain (`terrain.stl` for the collision, `terrain.obj/.mtl/.png` for the visual), in the map frame |
| `--world-template F` + `--world-out W` | with `--terrain-out`: fill `@LATITUDE@`, `@LONGITUDE@`, `@ELEVATION@` and `@TERRAIN_DIR@` of a world template (the simulations of `bigmap_bringup` use it) |

The same tool makes a Gazebo package of the place (`--gazebo`) or a NavMap of its terrain
(`--navmap`): see its `--help`.

**Outside Spain.**

```bash
ros2 run navmap_tools navmap_gis_tool --center 46.0207,7.7491 --size 1000 --resolution 1.0 \
    --dem-source copernicus30 --dem-out $HOME/.ros/bigmap/elevation_cache.bin
```

(that place, Zermatt, gives a reference altitude of about 1600 m, as it should.) What to know:

- The data is 30 m, so the terrain is smoother than the IGN's: ditches, kerbs and walls of a few
  metres do not exist in it. For those, make a NavMap of your own (previous section).
- Heights are above sea level; z = 0 is the terrain at the origin unless `--reference-altitude`
  says otherwise, as with the IGN.
- **Satellite colours** are optional. `--imagery-source esri` (worldwide, no key) or
  `--imagery-source xyz --imagery-url https://<server>/<path>/{z}/{x}/{y}.jpg` (any XYZ tile server;
  `--zoom` must be one the server has; which server to use, and under what terms, is your choice:
  check the provider's terms of use). Without `--dem-imagery` the terrain is built without colours.
- The area world of `easynav_playground_bigmap` (buildings, trees, walls) is Spain only for the Catastro; elsewhere OpenStreetMap.

**Alternatives outdoors**, if you do not use the download: make a NavMap file of the terrain
(previous section) and set `terrain.initial_navmap` instead of `dem_file`; set the UKF origin by
hand (or from the map's `.yaml`: `origin_lat`, `origin_lon`, `reference_altitude`).

## Run time

Services (`/<maps_manager_node>/bigmap/...`, all `std_srvs/srv/Trigger`):

| Service | What |
|---|---|
| `save_map` | writes the whole map (at the current resolutions) to `output.save_directory` as `map_all_<date>.navmap` + `.yaml` |
| `save_traversed` | the same, only the ground the robot has driven over |
| `clear_obstacles` | forgets every obstacle of the memory, the fixed ones too (for a recovery) |

The write happens in the plugin's next update; the result is in the log.

Parameters that can be changed while running: `obstacles.memory.enabled` (off: the fixed obstacles
are released at once and detections block as without the memory), `output.view_max_z` (cuts the
map RViz shows, to choose a floor), and the usual `ros2 param set /maps_manager_node bigmap.<name>`.

Topics: `<node>/bigmap/map` (NavMap, transient local), `.../map_updates`, `.../dynamic_map`,
`.../markers`.

## Reliability, speed and memory

- **Holes are obstacles, exactly.** From a NavMap the grid engine keeps, per grid cell, whether the
  map has ground there: at full resolution every hole of the map is where it is, however small (a
  block of one cell is an obstacle of one cell). `lod.hole_error` (m) makes the border of the holes
  count as a height error, so they keep their shape farther away.
- **Obstacles seen from near stay seen** (`lod.observed.enabled`): the hole borders within
  `lengths.radii.observed_mark` of the robot stay at full resolution (within
  `lengths.radii.observed_full_res` of them) wherever the robot goes, until it is farther than
  `traversed_keep` (mode `circles`). A coarse NavCel can no longer close, from far, a hole the
  robot saw from near: no plan through it, no back and forth between two meshes.
- **No path, finer map** (`recovery.*`): with a goal and no path for `no_path_cycles` cycles,
  `full_res` is doubled (up to `max_full_res_factor` times) and the map re-meshed; back to normal
  when the goal is reached or dropped.
- **Re-mesh in a worker** (`lod.async_remesh`, off by default): the cycle that asks for a re-mesh
  does not wait for it; the previous map is used until the new one is ready.
- **Hierarchy engine: incremental extraction.** The far cut (what the mesh is with the robot
  infinitely far) is computed once; each re-mesh climbs again only the vertices near the robot and
  near the driven-over / observed ground. Its cost is that of the region around the robot.
- **Memory.** The NavMap file is read as its arrays (never built as a NavMap only to be read); the
  error map is 2 bytes per node (centimetres) and holds no copy of the heights; the hierarchy keeps
  compact arrays (colours as bytes, slopes in hundredths of a degree, cells as sorted arrays). The
  log prints the size of the mesher at start.

## Obstacles and filters, in one paragraph

`obstacles.detector` says who finds the obstacles: `bigmap_dynamic` (the plugin analyses the
sensors: a detection that stays 5 s over a NavCel is an obstacle, `max_hits` of those fix it for
good) or `navmap_filter` (an `ObstaclesFilter` of `easynav_navmap_maps_manager` does, bigmap does
not). Then `removal` takes the obstacle NavCels (and a ring of `lengths.radii.obstacle_clearance`)
out of the map, the **filters** run in the order of the `filters` list (`inflation` writes
`inflated_obstacles`, which the planner reads), and `tint` paints the obstacles. A filter runs if
and only if it is in `filters`. At start the log prints the pipeline in use and refuses (or warns
about) combinations that do nothing, such as the `navmap_filter` detector without an obstacle
filter. Full description: the `bigmap_bringup` README, "Dynamic map and obstacles".

## Parameters

All of them, with units, accepted values and defaults, are in the examples above. Invalid values
stop the node at start with a message naming the parameters. Names used by earlier versions
(`lod_settings.*`, `traversed_ground.*`, `obstacle_source`...) are still accepted, with a warning
that says what replaces them. `bigmap.inflation.inflation_radius` must be at least
`system_node.robot_geometry.inscribed_radius`.

## Related packages

| Package | What it is | Needed by the plugin? |
|---|---|---|
| `navmap_tools` (NavMap) | `pointcloud_to_navmap` and `navmap_gis_tool`: the tools that make the NavMap and DEM files | no (offline tools) |
| `bigmap_bringup` | the application: launch file (the DEM, EasyNav, RViz), the complete parameter files the examples are copied from, maps; runs on a real robot and on a simulated one | no |
| `easynav_playground_bigmap` | the simulations: Gazebo worlds of a place with the Summit | no |

## Tests

`colcon test --packages-select easynav_bigmap_maps_manager`: unit tests of the meshers, the obstacle
memory, the pipeline check and the DEM from a NavMap; a test of the plugin itself (parameters,
`initialize()`, `update()` with a pose and sensor points); and a regression test of the hierarchy
engine on the real parking-garage map. Tests that need a real map of your own are skipped unless
`BIGMAP_TEST_NAVMAP=<file>` is set. Build with `-DCMAKE_BUILD_TYPE=Release` (the meshers are
slow in Debug).

## Author and license

Copyright 2026 José Miguel Guerrero (<josemiguel.guerrero@urjc.es>), author and maintainer.
Licensed under the Apache License, Version 2.0 (see `LICENSE` in the repository root).
