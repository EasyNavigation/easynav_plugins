^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_fusion_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2026-07-26)
------------------
* Add missing easynav_sensors deps
* deleted tests related to initial pose param which was removed. Removed spaces
* fix: add gnss group to gps config and ge_group has group to meet new easynav features. Initial pose now changes initial utm pose to change robot position in the map
* Update README.md for all localizers
* Reset pose from RViz2 for every localizer
* Adaptations to `#94 <https://github.com/EasyNavigation/easynav_plugins/issues/94>`_
* Added config file for gazebo smulation demo
* Only starts filter if params are found
* Added simple configuration to launch full demo
* Initialization considers IMU orientation
* Call set pose srvc for first pose for fastest convergence. Updated params to stabilice the filter in summit
* GPLv3 -> Apache 2.0
* fixed odom topic
* Several bugs fixed. Now it correctly integrates GPS measurements to the dual filter.
* Added dual ukf for local and global localization
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely, Miguel, midemig, migueldm

0.0.2 (2025-11-23)
------------------
* First version
* Contributors: Miguel Ángel de Miguel
