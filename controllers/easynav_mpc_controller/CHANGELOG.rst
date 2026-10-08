^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_mpc_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Velocity smoother support and runtime reconfiguration
* NLopt: its own copy (NLopt 2.11.0, static and private) when the system has none; no NLopt rosdep keys (none exist for RHEL)
* Configurable min_height (was a fixed 0.1 m)
* No conversion of an empty cloud (undefined behavior)
* The collision checker is no longer part of the controller
* Removed unused dependencies
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.4.2 (2026-07-26)
------------------
* Complete deps
* Adaptations to `#94 <https://github.com/EasyNavigation/easynav_plugins/issues/94>`_
* Update plugins to new sensors API
* GPLv3 -> Apache 2.0
* Collision checker scope was changed
* Remove C++20/C++23 features and update to new MethodBase interface
* Merge branch 'set_robot_frame' into frames-fix-pr-40
* TFInfo in RTTFBuffer
* Refactor to use TFInfo
* Collision avoidance was improved
* Final alignements
* Trim path for MPC
* Stop controllers at IDLE
* Obstacles are detected
* Methods were added to access to private attributes
* Optimizer header file was added
* Dependencies were fixed
* MPC with differential model is working
* MPC Controller minimizes path following
* Visualization Marker was added
* MPC was tunned
* MPC controller is working
* Optimizer was changed
* Dependecies were added
* MPC terminated
* MPC was added
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, José Miguel Guerrero, Juan S. Cely, Juan S. Cely G., Miguel, migueldm

0.0.2 (2025-10-12)
------------------
