# easynav_orbit_localizer

A localizer for EasyNav built on a different premise from Monte Carlo localization: the
hypotheses a robot needs are not samples, they are the poses the map's own symmetries make
indistinguishable, and they can be computed from the map before the robot moves.

The belief is a small set of Gaussian hypotheses on SE(3). Each one is corrected by Gauss-Newton
on the Lie group against the map's distance field (a 2D costmap or a 3D Bonxai map), with a prior
from the odometry and, when available, GPS positions, a depth gauge and the IMU's attitude in the
same solver. Where the map says nothing (a corridor, a flat field) the prior holds the pose and the
covariance stays large in that direction; where it does, the scan fixes it.

The symmetry graph samples the free places of the map, takes the sensor's view of each (a ray cast
on a costmap, the map's points around it on a 3D map) and finds, by a rotation-invariant descriptor
and a verification by the matcher, the other poses where that view fits. Online, the hypothesis at a
place spawns its twins; they live while the scans fit them and die when the map tells them apart.
The same graph answers a global match of a scan: the start without an initial pose, and the
relocalization after a kidnapping, when the winner stops explaining the scans.

Tolerances built in: a Cauchy loss on the points (dynamic obstacles count little), a chi-square
gate on GPS and depth (jumps, multipath), corrections by the scans when the odometry slips or
freezes, scans that stop (odometry alone, growing covariance) and the kidnapping above.

## Modes

| `mode` | state | use |
| --- | --- | --- |
| `planar` | x, y, yaw | a ground robot on a 2D map |
| `planar_terrain` | x, y, yaw; z, roll, pitch from the NavMap surface | a ground robot on a 3D map |
| `free` | all six | an aerial or underwater vehicle |

`map_type` is `costmap` (NavState `map.base`) or `bonxai` (`map.bonxai`). The odometry comes from
TF (`odom -> base_footprint`), an Odometry topic, or dead reckoning of a DVL's velocity and the IMU's
attitude (`odometry_source: dvl_imu`, for a submarine), in which case the plugin publishes
`odom -> base_footprint` too.

See `config/example_params.yaml` for every parameter with its default.

## Outputs

`robot_pose` in NavState (map frame, with the winner's covariance) and the `map -> odom` transform;
`localizer.orbit.match_score` (fraction of the scan on the map), `.hypotheses`, `.ambiguity`
(weight of the runner-up over the winner's), `.lost` and `.cycle_ms`. The topics
`<localizer_node>/<plugin>/pose` and `/hypotheses` show the estimate and the hypotheses.
