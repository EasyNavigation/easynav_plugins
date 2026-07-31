# easynav_regulated_pp_controller

Port of the [Nav2 Regulated Pure Pursuit Controller](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)
to the EasyNav plugin architecture (`easynav::ControllerMethodBase`), including its optional
Dynamic Window Pure Pursuit (DWPP) extension.

- S. Macenski, S. Singh, F. Martin, J. Gines, [**Regulated Pure Pursuit for Robot Path Tracking**](https://arxiv.org/abs/2305.20026). Autonomous Robots, 2023.
- Fumiya Ohnishi and Masaki Takahashi, **DWPP: Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints**, arXiv:2601.15006, 2026.

```bibtex
@article{macenski2023regulated,
      title={Regulated Pure Pursuit for Robot Path Tracking},
      author={Steve Macenski and Shrijit Singh and Francisco Martin and Jonatan Gines},
      year={2023},
      journal = {Autonomous Robots}
}
```

## Algorithm

The controller finds a lookahead ("carrot") point on the path at a given distance from the
robot and drives towards it, following the classic Pure Pursuit geometry: the curvature needed
to reach the carrot point is `k = 2y / (x^2+y^2)` in the robot's local frame. On top of that, it
adds the *regulation* terms from the paper above:

- **Velocity-scaled lookahead distance** (Adaptive Pure Pursuit): the lookahead distance grows
  with speed, clamped between `min_lookahead_dist` and `max_lookahead_dist`.
- **Curvature regulation**: linear velocity is reduced on sharp turns (small turning radius).
- **Obstacle-proximity regulation**: linear velocity is reduced close to obstacles.
- **Approach-to-goal regulation**: linear velocity is reduced smoothly as the robot nears the
  end of the path.
- **Rotate to rough heading / rotate to goal heading**: the robot rotates in place first if the
  carrot (or the final goal) is at a large angle from the current heading.
- **Dynamic Window Pure Pursuit (optional)**: instead of directly applying `angular_vel = linear_vel * curvature`,
  it searches the reachable (velocity, angular velocity) space under the configured
  acceleration/deceleration limits for the point that best matches the desired curvature.

## Adaptations from the Nav2 version

EasyNav's design differs from Nav2's `controller_server` in ways that require some adaptations:

- **No costmap.** EasyNav controllers only have access to the robot pose, the path, and fused
  point-cloud perceptions (`PointPerception`) via the `NavState` blackboard — there is no
  costmap or inflation layer to query. The cost-based linear velocity regulation term
  (`use_cost_regulated_linear_velocity_scaling` in Nav2) is therefore replaced by an equivalent
  term, `use_obstacle_regulated_linear_velocity_scaling`, driven by the distance to the nearest
  point in the fused perception cloud within the lookahead corridor, rather than by inflated
  costmap cost (see `heuristics::obstacleConstraint` in
  [`regulation_functions.hpp`](include/easynav_regulated_pp_controller/regulation_functions.hpp)).
- **No per-controller collision-arc checking.** Nav2's RPP throws a `NoValidControl` exception
  when it predicts a collision along a forward-simulated arc. EasyNav already performs this kind
  of safety stop uniformly for every controller in `ControllerMethodBase` (see the shared
  `colision_checker.*` parameters), so this controller does not duplicate it.
- **No separate goal-checker plugin.** Nav2 relies on an independent `GoalChecker` plugin to
  decide when the robot has reached the goal. This controller checks the distance/angle to the
  last path pose directly, using `goal_tolerance.position` / `goal_tolerance.yaw` from the
  `NavState` if `GoalManager` has published them, or the `xy_goal_tolerance` /
  `yaw_goal_tolerance` parameters otherwise — consistent with how other EasyNav controllers
  (e.g. `easynav_simple_controller`, `easynav_mpc_controller`) handle it.
- **No TF-based path transform.** EasyNav controllers assume `path` and `robot_pose` are already
  expressed in the same frame (see `easynav_simple_controller`, `easynav_mpc_controller`), so
  there is no `transformPathInTargetFrame` step. The path-to-robot-frame conversion needed by the
  Pure Pursuit geometry (curvature is only meaningful with the robot at the local origin facing
  +x) is done analytically from the robot pose instead of via `tf2`.
- **No plan pruning upstream.** Nav2's `controller_server` prunes the global plan to a local
  window starting at (or near) the robot before handing it to the controller. EasyNav's `path` in
  `NavState` is the full, un-pruned plan, so the carrot search (`getLookAheadPoint`) explicitly
  starts from the path pose closest to the robot (`findClosestPoseIndex`) rather than from index 0
  of the array. Skipping this step is a real trap, not just a style choice: once the robot has
  travelled more than a lookahead distance away from the path's start pose, that start pose itself
  would satisfy "farther than the lookahead distance", so a naive search from index 0 keeps
  returning it as the carrot for the rest of the path — freezing the carrot behind the robot,
  producing bogus large curvature/angle values, and making the controller rotate-in-place far more
  than it should.
- **No `setSpeedLimit` API.** EasyNav has no equivalent to Nav2's dynamic speed-limiting
  interface, so it is not ported.

## Parameters

| Parameter | Description |
|-----|----|
| `max_linear_vel` | Maximum linear velocity. |
| `min_linear_vel` | Minimum linear velocity, used when `use_dynamic_window` is `true`. |
| `max_angular_vel` / `min_angular_vel` | Angular velocity bounds, used when `use_dynamic_window` is `true`. |
| `max_linear_accel` / `max_linear_decel` | Linear acceleration/deceleration bounds, used when `use_dynamic_window` is `true`. |
| `max_angular_accel` / `max_angular_decel` | Angular acceleration/deceleration bounds; also used by `rotate_to_heading`. |
| `lookahead_dist` | Fixed lookahead distance to find the carrot point. |
| `min_lookahead_dist` / `max_lookahead_dist` | Bounds for the velocity-scaled lookahead distance. |
| `lookahead_time` | Gain used to scale the lookahead distance by the current speed. |
| `use_velocity_scaled_lookahead_dist` | Use velocity-scaled lookahead distance instead of the fixed `lookahead_dist`. |
| `rotate_to_heading_angular_vel` | Angular velocity used while rotating in place. |
| `use_rotate_to_heading` | Enable rotate-in-place behaviors (rough path heading and final goal heading). |
| `rotate_to_heading_min_angle` | Angle to the carrot beyond which the robot rotates in place first. |
| `use_regulated_linear_velocity_scaling` | Enable curvature-based velocity regulation. |
| `regulated_linear_scaling_min_radius` | Turning radius below which curvature regulation kicks in. |
| `regulated_linear_scaling_min_speed` | Minimum velocity kept under regulation. |
| `use_fixed_curvature_lookahead` | Use a separate, fixed lookahead distance to compute the regulation curvature. |
| `curvature_lookahead_dist` | Distance of the fixed curvature lookahead, if enabled. |
| `interpolate_curvature_after_goal` | Extrapolate the curvature carrot past the goal to avoid end-of-path oscillation. |
| `use_obstacle_regulated_linear_velocity_scaling` | Enable obstacle-proximity velocity regulation (EasyNav adaptation of Nav2's cost-based term). |
| `obstacle_scaling_dist` | Distance below which obstacle regulation is triggered. |
| `obstacle_scaling_gain` | Gain (`<= 1.0`) applied when scaling down the velocity near obstacles. |
| `min_approach_linear_velocity` | Minimum velocity while approaching the goal. |
| `approach_velocity_scaling_dist` | Remaining path distance at which approach-to-goal slow-down starts. |
| `allow_reversing` | Allow driving backwards when the carrot point is behind the robot. |
| `use_dynamic_window` | Enable the Dynamic Window Pure Pursuit (DWPP) extension. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | Fallback goal tolerances used if `GoalManager` has not published `goal_tolerance.*` in the `NavState`. |

## Topics

| Topic | Type | Description |
|-----|----|----|
| `lookahead_point` | `geometry_msgs/PointStamped` | The current carrot point on the path. |
| `curvature_lookahead_point` | `geometry_msgs/PointStamped` | The fixed curvature lookahead point, when `use_fixed_curvature_lookahead` is enabled. |
| `is_rotating_to_heading` | `std_msgs/Bool` | Whether the controller is currently rotating in place. |

## Example configuration

```yaml
controller:
  ros__parameters:
    controller_plugin: "easynav_regulated_pp_controller/RegulatedPurePursuitController"
    RegulatedPurePursuitController:
      max_linear_vel: 0.5
      min_linear_vel: -0.5
      max_angular_vel: 2.5
      min_angular_vel: -2.5
      max_linear_accel: 2.5
      max_linear_decel: 2.5
      max_angular_accel: 3.2
      max_angular_decel: 3.2
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      use_velocity_scaled_lookahead_dist: false
      rotate_to_heading_angular_vel: 1.8
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 1.0
      interpolate_curvature_after_goal: false
      use_obstacle_regulated_linear_velocity_scaling: false
      obstacle_scaling_dist: 0.3
      obstacle_scaling_gain: 1.0
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 1.0
      allow_reversing: false
      use_dynamic_window: false
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
```

## Tuning notes

Same guidance as upstream Nav2 applies:

- For Adaptive Pure Pursuit behavior, disable all boolean regulation parameters except
  `use_velocity_scaled_lookahead_dist`, and tune `lookahead_time`, `min_lookahead_dist` and
  `max_lookahead_dist`.
- For plain Pure Pursuit behavior, disable all boolean parameters and set
  `approach_velocity_scaling_dist: 0.0` to disable approach-to-goal slow-down, then tune
  `lookahead_dist`.
- `use_obstacle_regulated_linear_velocity_scaling` can over-trigger in tightly cluttered
  environments (every point in a narrow corridor is "close"); tune `obstacle_scaling_dist` and
  `obstacle_scaling_gain` for the platform's environment, the same way `cost_scaling_dist` /
  `cost_scaling_gain` are tuned in Nav2.
