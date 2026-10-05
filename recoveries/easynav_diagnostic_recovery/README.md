# easynav_diagnostic_recovery

A diagnosis-driven recovery system for EasyNav: `easynav_diagnostic_recovery/DiagnosticRecoveryManager`.
For the full design, see `docs/recoveries_easynav.md` in EasyNavigation.

A recovery system is a `RecoveryManagerBase` plugin hosted by `recovery_node`. This one is made of
plugins too, at two levels:

- **Level 0, RT: safety reflexes** (`SafetyReflexBase`). Every RT cycle, right before publishing, each
  reflex checks the command about to be sent, whoever produced it (the controller or a mitigation).
  On imminent danger it overrides that command (highest priority, not smoothed).
- **Level 1, non-RT: evaluators and mitigations.**
  - Evaluators (`RecoveryEvaluatorBase`) diagnose. Each one writes a
    `diagnostic_msgs/DiagnosticStatus` (OK/WARN/ERROR) to NavState.
  - Mitigations (`RecoveryMitigationBase`) handle diagnostics in `ERROR`. On each cycle, the manager:
    - takes the first diagnostic in `ERROR` and starts the first mitigation, by `priority` (lower
      first), whose `can_handle()` accepts it;
    - runs at most one mitigation at a time, until it returns `SUCCEEDED` or `FAILED`;
    - when one fails, excludes it for that diagnostic, so the next one is tried (escalation).
  - A mitigation that `requires_control()` drives the robot: it takes over the controller.
  - Mitigations ask EasyNav through the manager to abort the mission or shut down.
  - While a mitigation is active or a diagnostic is in `ERROR`, the mission's progress is held: no goal
    is taken as reached.

Topics: `diagnostics` (`diagnostic_msgs/DiagnosticArray`) and `mitigation` (`rcl_interfaces/Log`,
what the active mitigation reports). The EasyNav TUI shows both.

## Package layout

Everything is in the `easynav_diagnostic_recovery` package and library. Headers are under
`include/easynav_diagnostic_recovery/`, sources under `src/easynav_diagnostic_recovery/` and tests
under `tests/`, with the same subfolders:

| Subfolder | Contents |
|---|---|
| (top level) | The manager, the three plugin interfaces, `ObstacleProximity`, and dummy plugins (`easynav_diagnostic_recovery` namespace) |
| `reflexes/` | `CollisionSafetyReflex` |
| `evaluators/` | `NoPathEvaluator`, `ObstacleTooCloseEvaluator`, `ControllerStuckEvaluator`, `RosGraphEvaluator`, `SafetyChannelEvaluator` |
| `mitigations/` | `SafeRetreatRecovery`, `AdvanceRecovery`, `ShutdownRecovery`, `HumanAssistanceRecovery`, `CancelMissionRecovery` |

A component can also ship recovery for its own failures. For example, `easynav_costmap_localizer`
ships `AmclConvergenceEvaluator` and `AmclRelocalizeMitigation`.

## Plugins

Each plugin's parameters are under `recovery_manager.<type>.`. Every mitigation also takes `priority`
(default `100`).

| Plugin | Does | Parameters (default) |
|---|---|---|
| `easynav_diagnostic_recovery/CollisionSafetyReflex` | Brakes if the commanded motion would hit an obstacle within its stopping distance. The robot's radius and height come from `system_node.robot_geometry`. | `brake_acc` (0.5), `safety_margin` (0.1), `z_min_filter` (0.0), `downsample_leaf_size` (0.1), `debug_markers` (false) |
| `easynav_diagnostic_recovery/NoPathEvaluator` | `ERROR` (`planner`) if there is a goal but the path is empty (`WARN` until the first path). | — |
| `easynav_diagnostic_recovery/ObstacleTooCloseEvaluator` | `ERROR` (`obstacle_proximity`) if the robot is stopped closer than `safe_distance` (from the robot center) to an obstacle (after `debounce_duration` s stopped). Points below `z_min_filter` or above the robot height (`robot_geometry`) are ignored. | `safe_distance` (0.6), `z_min_filter` (0.0), `debounce_duration` (0.2), `linear_velocity_epsilon` (0.02), `angular_velocity_epsilon` (0.05) |
| `easynav_diagnostic_recovery/ControllerStuckEvaluator` | `ERROR` (`controller_stuck`) if commanded to move but not progressing (not while paused or during a protective stop). | `linear_velocity_threshold` (0.02), `progress_distance_threshold` (0.05), `stuck_time_threshold` (2.0) |
| `easynav_diagnostic_recovery/RosGraphEvaluator` | `ERROR` (`ros_graph`) if an EasyNav subscription has no publisher, or its velocity output has no consumer. | `freq` (10.0), `startup_grace` (15.0), `error_debounce` (2.0), `ignored_topics`, `ignored_consumers` |
| `easynav_diagnostic_recovery/SafetyChannelEvaluator` | `WARN` (`safety_channel`) during a protective stop of the safety channel, or with its status lost (`safety_status`, see `system_node`'s `safety.status.timeout`); `ERROR` once it lasts `max_stop_time` s. | `max_stop_time` (0: never `ERROR`) |
| `easynav_diagnostic_recovery/SafeRetreatRecovery` | Handles `obstacle_proximity`: moves straight away until `safe_distance` (from the robot center): backward from an obstacle ahead, forward from one behind or beside (the other way if that one is blocked and the obstacle is beside). Fails, stopped, without perception or a clear way (`min_clearance` along its corridor). | `safe_distance` (0.6), `retreat_speed` (0.15), `min_clearance` (0.05), `z_min_filter` (0.0) |
| `easynav_diagnostic_recovery/AdvanceRecovery` | Handles `controller_stuck`: moves forward a little. Gives up after `escalate_after` s of repeated attempts. | `advance_distance` (0.3), `advance_speed` (0.1), `escalate_after` (15.0), `episode_gap` (10.0) |
| `easynav_diagnostic_recovery/ShutdownRecovery` | Handles the listed diagnostics by terminating EasyNav. | `handled_hardware_ids` ([`ros_graph`]) |
| `easynav_diagnostic_recovery/HumanAssistanceRecovery` | Handles any `ERROR` not in `ignored_hardware_ids`: stops and waits until those clear. `timeout` 0 waits forever. | `timeout` (0.0), `ignored_hardware_ids` ([`ros_graph`]) |
| `easynav_diagnostic_recovery/CancelMissionRecovery` | Handles any `ERROR`: aborts the mission. The last resort. | — |
| `easynav_costmap_localizer/AmclConvergenceEvaluator` | `ERROR` (`localizer.amcl`) if AMCL's covariance trace exceeds the threshold. | `covariance_threshold` (1.0) |
| `easynav_costmap_localizer/AmclRelocalizeMitigation` | Handles `localizer.amcl`: rotates until AMCL converges, or fails after `timeout`. | `covariance_threshold` (1.0), `rotation_speed` (0.3), `timeout` (5.0) |

## Usage

```yaml
recovery_node:
  ros__parameters:
    recovery_manager:
      plugin: easynav_diagnostic_recovery/DiagnosticRecoveryManager
      safety_reflex_types: [collision]
      collision:
        plugin: easynav_diagnostic_recovery/CollisionSafetyReflex
      evaluator_types: [no_path, obstacle_close, controller_stuck, amcl_convergence, ros_graph]
      no_path:
        plugin: easynav_diagnostic_recovery/NoPathEvaluator
      obstacle_close:
        plugin: easynav_diagnostic_recovery/ObstacleTooCloseEvaluator
      controller_stuck:
        plugin: easynav_diagnostic_recovery/ControllerStuckEvaluator
      amcl_convergence:
        plugin: easynav_costmap_localizer/AmclConvergenceEvaluator
      ros_graph:
        plugin: easynav_diagnostic_recovery/RosGraphEvaluator
      # Tried in priority order (lower first) for each ERROR they can handle:
      # specific fixes -> terminate on a miswired graph -> wait for a human -> cancel the mission
      mitigation_types: [retreat, amcl_relocalize, advance, shutdown, human_assistance, cancel_mission]
      retreat:
        plugin: easynav_diagnostic_recovery/SafeRetreatRecovery
        priority: 10
      amcl_relocalize:
        plugin: easynav_costmap_localizer/AmclRelocalizeMitigation
        priority: 10
      advance:
        plugin: easynav_diagnostic_recovery/AdvanceRecovery
        priority: 10
      shutdown:
        plugin: easynav_diagnostic_recovery/ShutdownRecovery
        priority: 100
      human_assistance:
        plugin: easynav_diagnostic_recovery/HumanAssistanceRecovery
        priority: 1000
        timeout: 30.0
      cancel_mission:
        plugin: easynav_diagnostic_recovery/CancelMissionRecovery
        priority: 2000
```

## Writing a plugin

Derive from one of the interfaces, in any package:

| Interface | Implement |
|---|---|
| `easynav_diagnostic_recovery::SafetyReflexBase` | `check()`: is the command about to be sent (`commanded_velocity()`) dangerous? `mitigate()`: `override_velocity()` or `stop_robot()`. |
| `easynav_diagnostic_recovery::RecoveryEvaluatorBase` | `update()`: diagnose and `publish_diagnostic()` every time, OK included. |
| `easynav_diagnostic_recovery::RecoveryMitigationBase` | `can_handle(status)`; `on_start()`, `on_cycle()` (returns `RUNNING`, `SUCCEEDED` or `FAILED`) and `on_stop()`. Use `command_velocity()` if it `requires_control()`, and `report()` what it does. |

Register it against `easynav_diagnostic_recovery`, with its full base class:

```xml
<class name="my_pkg/MyEvaluator" type="my_ns::MyEvaluator"
       base_class_type="easynav_diagnostic_recovery::RecoveryEvaluatorBase">
```

```cmake
pluginlib_export_plugin_description_file(easynav_diagnostic_recovery my_pkg_plugins.xml)
```
