^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_simple_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Runtime reconfiguration
* Configurable min_height (was a fixed 0.1 m)
* num_particles initialized before use
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.4.2 (2026-07-26)
------------------
* Complete deps
* Avoid creating objects every loop cycle
* Fix std::normal_distribution when stdev is zero
* Navstate key among filters are always map, not an arbitrary key
* Check if std is non-positive before creating std::normal_distribution
* Check if std is non-positive before creating std::normal_distribution
* Navstate key among filters are always map, not an arbitrary key
* Update README.md for all localizers
* Reset pose from RViz2 for every localizer
* Update plugins to new sensors API
* GPLv3 -> Apache 2.0
* Improve navstate print including the time
* Add a base_footprint frame in TFInfo
* Remove C++20/C++23 features and update to new MethodBase interface
* Correct publication stamp to input stamp
* Merge branch 'set_robot_frame' into frames-fix-pr-40
* TFInfo in RTTFBuffer
* Refactor to use TFInfo
* Optimize execution
* Cleanup unused headers
* Update sheets
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely, Juan S. Cely G., Miguel, migueldm

0.0.2 (2025-10-12)
------------------
* Reorganization initial
* Contributors: Francisco Martín Rico
