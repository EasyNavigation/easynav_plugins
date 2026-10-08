^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_navmap_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Runtime reconfiguration
* Planners never keep a stale path
* The path ends at the goal, inscribed cells are not crossed, and cost_weight balances cost against length
* A* over NavCels that share a vertex (no direction favored by the mesh) and line-of-sight shortcut of the path
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Contributors: Francisco Martín Rico

0.4.2 (2026-07-26)
------------------
* Complete deps
* GPLv3 -> Apache 2.0
* Improve navstate print including the time
* Remove C++20/C++23 features and update to new MethodBase interface
* Merge branch 'set_robot_frame' into frames-fix-pr-40
* TFInfo in RTTFBuffer
* Refactor to use TFInfo
* Optimize execution
* Cleanup unused headers
* Update sheets
* Path planner aware of obstacles and inflation
* A Star uses obstacle layer
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely, Juan S. Cely G., Miguel, migueldm

0.0.2 (2025-10-12)
------------------
* Reorganization initial
* Contributors: Francisco Martín Rico
