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
| Stuck (commanded but not moving for `stuck_time`) | back up for `backup_time`; after `max_backup_attempts`, slow down (`request_reconfigure()` of `controller_node.robot_limits.max_linear_vel` to `slow_down_max_linear_vel`); still stuck, `abort_mission()`. The speed is restored when the mission ends (`request_restore_parameters()`) |
| Otherwise | nothing: the controller drives |

## Usage

The area checked ahead is the robot's: its radius and height come from
`system_node.robot_geometry` (`robot_radius` and `max_obstacle_z` here are deprecated).

```yaml
recovery_node:
  ros__parameters:
    recovery_manager:
      plugin: easynav_simple_recovery/SimpleRecoveryManager
      stop_distance: 0.3          # [m] beyond the robot radius
      min_obstacle_z: 0.05        # [m]
      sensors_timeout: 5.0        # [s]
      max_position_variance: 1.0  # [m^2]
      relocalize_timeout: 20.0    # [s]
      rotate_speed: 0.5           # [rad/s]
      stuck_time: 10.0            # [s]
      stuck_distance: 0.05        # [m]
      backup_speed: 0.1           # [m/s]
      backup_time: 2.0            # [s]
      max_backup_attempts: 3
      slow_down_max_linear_vel: 0.1  # [m/s], 0: never slow down
```

## Reconfiguring as a mitigation

`request_reconfigure()` changes parameters of any EasyNav node and reconfigures EasyNav to apply
them, between cycles: the mission goes on, and the robot only stops during the transitions. The
recovery system is reloaded too, so a new instance starts with fresh members: keep in NavState
what you need to remember. EasyNav lists the parameters changed so far in
`reconfigured_parameters` (`"node/parameter"`), and `request_restore_parameters()` restores them.
