^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_costmap_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-07-26)
------------------
* Add missing easynav_sensors deps
* Avoid creating objects every loop cycle
* Fix std::normal_distribution when stdev is zero
* Check if std is non-positive before creating std::normal_distribution
  Dynamic base costmap
* Change map static to base
* Reset pose from RViz2 for every localizer
* Adaptations to `#94 <https://github.com/EasyNavigation/easynav_plugins/issues/94>`_
* Merge branch 'EasyNavigation:rolling' into rolling
* GPLv3 -> Apache 2.0
* Sync to current EasyNavigation
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely G., Miguel, migueldm

0.2.1 (2026-02-27)
------------------
* 0.2.0
* GPLv3 -> Apache 2.0
* Documentation was corrected
* Add a base_footprint frame in TFInfo
* Remove C++20/C++23 features and update to new MethodBase interface
* TFInfo in RTTFBuffer
* Refactor to use TFInfo
* Referencing base class if ot void
* Optimize execution
* Cleanup unused headers
* Update sheets
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely G., Miguel

0.0.2 (2025-10-12)
------------------
* Reorganization initial
* Contributors: Francisco Martín Rico
