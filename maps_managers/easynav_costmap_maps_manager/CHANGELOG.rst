^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_costmap_maps_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Runtime reconfiguration
* Robot geometry from system_node
* Configurable min_height in the ObstacleFilter (was a fixed 0.1 m)
* Fixed std::bad_alloc
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Contributors: Francisco Martín Rico

0.4.2 (2026-07-26)
------------------
* Complete deps
* Update dynamic_map\_ with map key at navstate
* Avoid rewrite map with dynamic_map
* Navstate key among filters are always map, not an arbitrary key
* Update calls to deprecated get_package_share_directory
* Update dynamic_map\_ with map key at navstate
* Avoid rewrite map with dynamic_map
* Navstate key among filters are always map, not an arbitrary key
* Merge fix
* Any change in the costmap set it as modified
* Publish changes in base map and be ready to save updated map
* Costamp base map sincronized with nav_state
* Change map static to base
* Adaptations to `#94 <https://github.com/EasyNavigation/easynav_plugins/issues/94>`_
* Update plugins to new sensors API
* GPLv3 -> Apache 2.0
* Get timestamp from perception for stamping
* Improve navstate print including the time
* Use last available TF for obstacle filter
* Adjust output time in costamp stack
* Remove C++20/C++23 features and update to new MethodBase interface
* TFInfo in RTTFBuffer
* Refactor to use TFInfo
* linting docs
* fix: Prevent A* planner from traversing through obstacles
* Final optimization
* First UpdateCosts optimization
* Update perception bound
* Cleanup unused headers
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Jose Miguel, Juan S. Cely, Juan S. Cely G., Miguel, migueldm

0.0.2 (2025-10-12)
------------------
* Reorganization initial
* Contributors: Francisco Martín Rico
