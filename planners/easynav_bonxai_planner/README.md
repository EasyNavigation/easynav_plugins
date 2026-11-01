# easynav_bonxai_planner

## Description

3D path planner for aerial robots: A* over a [Bonxai](https://github.com/facontidavide/Bonxai) map (`map.bonxai`, from `easynav_bonxai_maps_manager`), keeping a minimum distance from every occupied voxel.

A Bonxai map holds what the sensors saw: surfaces, with gaps, and free space behind them (under the ground, inside buildings). `fill_below` makes what is below them solid. From the occupied voxels it builds a grid of the distance to the nearest obstacle (an exact Euclidean distance transform), once per map. A* (26-connected) only goes through cells at least `min_clearance` from the obstacles, and each step costs more the closer it is than `preferred_clearance`. The path is then made of straight segments wherever the clearance along them allows, sampled every `path_step` and smoothed: corners become curves without coming closer to the obstacles. A start or goal closer than `min_clearance` to an obstacle is moved to the nearest cell that is not, within `snap_radius`.

## Authors and Maintainers

- **Authors:** Intelligent Robotics Lab
- **Maintainers:** Francisco Martín Rico <fmrico@gmail.com>

## Supported ROS 2 Distributions

| Distribution | Status |
|---|---|
| humble | ![humble](https://img.shields.io/badge/humble-supported-brightgreen) |
| jazzy | ![jazzy](https://img.shields.io/badge/jazzy-supported-brightgreen) |
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |
| lyrical | ![lyrical](https://img.shields.io/badge/lyrical-supported-brightgreen) |
| rolling | ![rolling](https://img.shields.io/badge/rolling-supported-brightgreen) |

## Plugin (pluginlib)

- **Plugin Name:** `easynav_bonxai_planner/BonxaiPlanner`
- **Type:** `easynav::BonxaiPlanner`
- **Base Class:** `easynav::PlannerMethodBase`
- **Library:** `easynav_bonxai_planner`

## Parameters

All under the plugin's name (`<plugin>.`):

| Name | Type | Default | Description |
|---|---|---:|---|
| `resolution` | `double` | `1.0` | Cell size of the planning grid (m). The grid covers the map's bounding box plus `margin`: at 1 m a town of 400 x 400 x 60 m is about 10 million cells. |
| `margin` | `double` | `10.0` | Space around the map's bounding box the robot may use (m). |
| `min_altitude` | `double` | NaN | Lowest z of the grid (m); NaN: the map's lowest point minus `margin`. |
| `fill_below` | `string` | `lowest` | What is solid below the map's points in each column: `none` (only the points), `lowest` (below the lowest one: the ground, so a gap in its scan is no way underground) or `highest` (below the highest one: buildings as solid blocks, as an elevation map; nothing can be flown under). Columns with no points take their neighbours' height. |
| `max_altitude` | `double` | NaN | Highest z of the grid (m); NaN: the map's highest point plus `margin`. |
| `min_clearance` | `double` | `1.5` | Distance from the obstacles the path never comes closer than (m). |
| `preferred_clearance` | `double` | `4.0` | Closer than this, a step costs more (m); 0: no preference. |
| `clearance_weight` | `double` | `2.0` | Extra cost per meter of a step with no clearance (it falls quadratically to 0 at `preferred_clearance`). |
| `snap_radius` | `double` | `5.0` | How far a start or goal too close to an obstacle may be moved (m). |
| `max_expansions` | `int` | `3000000` | A* gives up after expanding this many cells. |
| `path_step` | `double` | `0.5` | Distance between the path's poses (m). |
| `smooth_iterations` | `int` | `200` | Relaxation passes of the smoother (0: the straight segments, with their corners). |
| `smooth_weight` | `double` | `0.5` | Pull of each point towards its neighbours' mean, per pass. |
| `smooth_data_weight` | `double` | `0.01` | Pull back towards where A* put it (0.01 rounds a 90 deg corner to about 2.5 m of radius; higher, tighter). The smoother never moves a point closer to an obstacle than `min_clearance`; the ends do not move. |
| `continuous_replan` | `bool` | `true` | Plan in every cycle; false: only for a new goal. |

## Interfaces

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/path` | `nav_msgs/msg/Path` | The path (only when subscribed). |

## NavState Keys

| Key | Type | Access | Notes |
|---|---|---|---|
| `map.bonxai` | `Bonxai::ProbabilisticMap` | **Read** | Its occupied voxels are the obstacles. |
| `goals` | `nav_msgs::msg::Goals` | **Read** | The first one, in the map frame. |
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Start of the path. |
| `path` | `nav_msgs::msg::Path` | **Write** | Poses facing along the path; the last one with the goal's orientation. Empty if there is no path. |

## License

Apache-2.0
