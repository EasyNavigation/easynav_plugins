# EasyNav Plugins

[![Doxygen Deployment](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/doxygen-doc.yml/badge.svg)](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/doxygen-doc.yml)
[![rolling](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/rolling.yaml)
[![kilted](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/kilted.yaml/badge.svg?branch=kilted)](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/kilted.yaml)
[![jazzy](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/jazzy.yaml/badge.svg?branch=jazzy)](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/jazzy.yaml)
[![humble](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/humble.yaml/badge.svg?branch=humble)](https://github.com/EasyNavigation/easynav_plugins/actions/workflows/humble.yaml)

📋 Roadmap Project: [RoadMap](https://github.com/EasyNavigation/EasyNavigation/blob/rolling/ROADMAP.md)

## Description

**EasyNav Plugins** provides the official collection of plugins for the [Easy Navigation (EasyNav)](https://github.com/EasyNavigation) framework.  
These plugins extend the navigation core with planners, controllers, map managers, and localizers compatible with ROS 2.

Each plugin resides in its own ROS 2 package and is registered via `pluginlib`, allowing dynamic loading at runtime.

---

## Repository Structure

### 🧭 Planners

Path planning plugins implementing A*, costmap, or NavMap–based methods.

| Package | Description | Link |
|---|---|---|
| `easynav_costmap_planner` | A* planner over `Costmap2D`. | [README](./planners/easynav_costmap_planner/README.md) |
| `easynav_simple_planner` | Simple A* planner for `SimpleMap`. | [README](./planners/easynav_simple_planner/README.md) |
| `easynav_navmap_planner` | A* planner over a NavMap mesh. | [README](./planners/easynav_navmap_planner/README.md) |

---

### ⚙️ Controllers

Motion controllers for trajectory tracking and reactive behaviors.

| Package | Description | Link |
|---|---|---|
| `easynav_vff_controller` | Vector Field Force (VFF) reactive controller. | [README](./controllers/easynav_vff_controller/README.md) |
| `easynav_mppi_controller` | Model Predictive Path Integral (MPPI) controller. | [README](./controllers/easynav_mppi_controller/README.md) |
| `easynav_simple_controller` | Simple proportional controller for testing. | [README](./controllers/easynav_simple_controller/README.md) |
| `easynav_serest_controller` | SeReST (Safe Reactive Steering) controller. | [README](./controllers/easynav_serest_controller/README.md) |
| `easynav_mpc_controller` | Model Predictive Controller (MPC). | [README](./controllers/easynav_mpc_controller/README.md) |

---

### 🗺️ Maps Managers

Map management plugins that provide, update, and store different environment representations.

| Package | Description | Link |
|---|---|---|
| `easynav_navmap_maps_manager` | Manages NavMap mesh layers. | [README](./maps_managers/easynav_navmap_maps_manager/README.md) |
| `easynav_bonxai_maps_manager` | Manages Bonxai probabilistic voxel maps. | [README](./maps_managers/easynav_bonxai_maps_manager/README.md) |
| `easynav_octomap_maps_manager` | Manages OctoMap 3D occupancy trees. | [README](./maps_managers/easynav_octomap_maps_manager/README.md) |
| `easynav_costmap_maps_manager` | Manages Costmap2D layers with filters. | [README](./maps_managers/easynav_costmap_maps_manager/README.md) |
| `easynav_simple_maps_manager` | Minimal example map manager (SimpleMap). | [README](./maps_managers/easynav_simple_maps_manager/README.md) |

---

### 📍 Localizers

Localization plugins based on different map types and sensors.

| Package | Description | Link |
|---|---|---|
| `easynav_gps_localizer` | GPS-based localizer for outdoor navigation. | [README](./localizers/easynav_gps_localizer/README.md) |
| `easynav_simple_localizer` | Basic localizer for SimpleMap–based setups. | [README](./localizers/easynav_simple_localizer/README.md) |
| `easynav_navmap_localizer` | AMCL-like localizer operating on NavMap meshes. | [README](./localizers/easynav_navmap_localizer/README.md) |
| `easynav_costmap_localizer` | AMCL-like localizer using Costmap2D. | [README](./localizers/easynav_costmap_localizer/README.md) |
| `easynav_fusion_localizer` | Multi-sensor fusion localizer (e.g., GPS + odometry + map). | [README](./localizers/easynav_fusion_localizer/README.md) |

---

## License

All packages in this repository are released under **Apache-2.0** unless stated otherwise in the individual package.
