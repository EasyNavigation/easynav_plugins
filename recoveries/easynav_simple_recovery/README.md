# easynav_simple_recovery

A simple recovery system for EasyNav, written to be read: a starting point for your own.
Read `SimpleRecoveryManager.cpp` from top to bottom.

A recovery system is a `RecoveryManagerBase` plugin hosted by `recovery_node`. EasyNav calls:

- `update_rt()`, every RT cycle, after the controller and before publishing. Keep it fast. This one:
  - **brakes dead** before an obstacle right ahead (`override_velocity()`: highest priority,
    not smoothed);
  - **executes the mitigation** chosen by `update()` (`command_velocity()`: takes over the
    controller, smoothed). A proposal lasts one cycle, so it is commanded on every cycle.
- `update()`, every non-RT cycle. Diagnose and decide, most severe case first:

| Case | Action |
|---|---|
| No new sensor data for `sensors_timeout` (counted from activation until the first data arrives) | `request_shutdown()` |
| Localization lost (x or y variance > `max_position_variance`) | `hold_mission_progress(true)` and rotate; `abort_mission()` after `relocalize_timeout` |
| Stuck (commanded but not moving for `stuck_time`) | back up for `backup_time`; `abort_mission()` after `max_backup_attempts` |
| Otherwise | nothing: the controller drives |

## Usage

```yaml
recovery_node:
  ros__parameters:
    recovery_manager:
      plugin: easynav_simple_recovery/SimpleRecoveryManager
      stop_distance: 0.3          # [m] beyond robot_radius
      robot_radius: 0.3           # [m]
      min_obstacle_z: 0.05        # [m]
      max_obstacle_z: 2.0         # [m]
      sensors_timeout: 5.0        # [s]
      max_position_variance: 1.0  # [m^2]
      relocalize_timeout: 20.0    # [s]
      rotate_speed: 0.5           # [rad/s]
      stuck_time: 10.0            # [s]
      stuck_distance: 0.05        # [m]
      backup_speed: 0.1           # [m/s]
      backup_time: 2.0            # [s]
      max_backup_attempts: 3
```
