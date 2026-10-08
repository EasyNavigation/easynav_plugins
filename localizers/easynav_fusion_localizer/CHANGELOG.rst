^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_fusion_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Runtime reconfiguration
* Fixed NaN in the local filter and a node leak
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Contributors: Francisco Martín Rico, Miguel Ángel de Miguel Paraíso

0.4.2 (2026-07-26)
------------------
* Complete deps
* deleted tests related to initial pose param which was removed. Removed spaces
* fix: add gnss group to gps config and ge_group has group to meet new easynav features. Initial pose now changes initial utm pose to change robot position in the map
* Adaptations to `#94 <https://github.com/EasyNavigation/easynav_plugins/issues/94>`_
* Added config file for gazebo smulation demo
* Uncrustify changes
* Only starts filter if params are found
* params updated
* Added simple configuration to launch full demo
* Initialization considers IMU orientation
* Call set pose srvc for first pose for fastest convergence. Updated params to stabilice the filter in summit
* GPLv3 -> Apache 2.0
* fixed odom topic
* Several bugs fixed. Now it correctly integrates GPS measurements to the dual filter.
* Added dual ukf for local and global localization
* Add a base_footprint frame in TFInfo
* Remove C++20/C++23 features and update to new MethodBase interface
* Merge branch 'set_robot_frame' into frames-fix-pr-40
* TFInfo in RTTFBuffer
* Fixed frames. In robot_localization, world frame must be odom or map depending on filter mode: local or global. This first version works as only global filter, so I changed world frame to map
* Refactor to use TFInfo
* fixed subscribers cb group, not in rt
* added pkg to workflow and ament_uncrustify code
* First working version
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, Juan S. Cely, Miguel, midemig, migueldm

0.0.2 (2025-11-23)
------------------
* First version
* Contributors: Miguel Ángel de Miguel
