^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_navmap_maps_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Runtime reconfiguration (rejected ones are reported)
* Robot geometry from system_node
* ObstacleFilter: keeps the static map; range and height limits; a column is an obstacle when it rises more than min_height above the NavMap surface (so a 2D laser sees obstacles, and the ground and ramps are not ones); min_height_per_meter and downsample_resolution parameters
* InflationFilter: inflates across unknown cells
* Fixed std::bad_alloc
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Contributors: Francisco Martín Rico

0.4.2 (2026-07-26)
------------------
* Complete deps
* Navstate key among filters are always map, not an arbitrary key
* Update calls to deprecated get_package_share_directory
* Adaptations to `#94 <https://github.com/EasyNavigation/easynav_plugins/issues/94>`_
* Update plugins to new sensors API
* GPLv3 -> Apache 2.0
* Improve navstate print including the time
* Remove C++20/C++23 features and update to new MethodBase interface
* Merge branch 'set_robot_frame' into frames-fix-pr-40
* TFInfo in RTTFBuffer
* Final adjustements
* Refactor to use TFInfo
* Merge branch 'rolling' into improve_astar_costmap
* linting docs
* Optimize execution
  Cleanup unused headers
* Dependencies were fixed
* Update sheets
* First functional version of Inflation filter
* Update to Occ constants
* Add obstacle layer
* Optimized function
* First functional verison, but expensive
* Update sheets
* First functional version of Inflation filter
* Update to Occ constants
* Add obstacle layer
* Optimized function
* First functional verison, but expensive
* Add params to navmap building map
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely, Juan S. Cely G., Miguel, migueldm

0.0.2 (2025-10-12)
------------------
* Reorganization initial
* Contributors: Francisco Martín Rico
