// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Model Predictive Path Integral optimizer, for differential and holonomic robots.

#ifndef EASYNAV_MPPI_CONTROLLER__MPPIOPTIMIZER_HPP_
#define EASYNAV_MPPI_CONTROLLER__MPPIOPTIMIZER_HPP_

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "easynav_common/types/RobotGeometry.hpp"
#include "easynav_mppi_controller/NearestGrid.hpp"
#include "easynav_mppi_controller/PointBuckets.hpp"

namespace easynav
{

/// \brief How the robot moves.
enum class MotionModel
{
  Diff,  ///< Differential (or skid steer): vx and wz.
  Omni,  ///< Holonomic (omnidirectional, mecanum): vx, vy and wz.
};

/// \brief Orientation kept while moving (the goal's is always reached at the end).
enum class HeadingMode
{
  Path,   ///< Facing along the path (the only one for a differential robot).
  Free,   ///< Any: the optimizer chooses.
  Goal,   ///< The goal's orientation, all the way.
  Hold,   ///< The robot's orientation when the goal was received.
  Fixed,  ///< A given orientation (map frame).
};

/// \brief Parses "diff"/"omni"; false if unknown.
bool parse_motion_model(const std::string & name, MotionModel & model);
/// \brief Parses "path"/"free"/"goal"/"hold"/"fixed"; false if unknown.
bool parse_heading_mode(const std::string & name, HeadingMode & mode);

/// \brief Velocity in the robot frame.
struct Velocity
{
  double vx {0.0};  ///< Forward (m/s).
  double vy {0.0};  ///< Leftward (m/s), 0 for a differential robot.
  double wz {0.0};  ///< Counterclockwise (rad/s).
};

/// \brief Planar pose.
struct Pose2D
{
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
};

/// \brief Configuration of the optimizer. Weights are per meter/radian, averaged over the horizon.
struct MPPIParams
{
  MotionModel motion_model {MotionModel::Diff};
  int num_samples {500};      ///< Rollouts per cycle.
  int horizon_steps {40};     ///< Steps of each rollout.
  double dt {0.1};            ///< Duration of each step (s).
  double lambda {0.05};       ///< Temperature: lower follows the best rollout more closely.

  double max_vx {0.5};        ///< Forward velocity (m/s).
  double min_vx {-0.2};       ///< Backward velocity (m/s, <= 0).
  double max_vy {0.0};        ///< Lateral velocity, either side (m/s). Omni only: (vx, vy)
                              ///< within the ellipse of max_vx (min_vx backwards) and max_vy.
  double max_wz {1.0};        ///< Angular velocity, either direction (rad/s).
  double acc_vx {0.5};        ///< Accelerations / decelerations (m/s^2, rad/s^2).
  double decel_vx {1.0};
  double acc_vy {0.5};
  double decel_vy {1.0};
  double acc_wz {1.0};
  double decel_wz {2.0};

  double vx_std {0.2};        ///< Sampling noise of each velocity.
  double vy_std {0.2};
  double wz_std {0.4};
  double noise_correlation {0.8};  ///< Of the noise between steps, [0, 1): smoother rollouts.

  double robot_radius {0.3};      ///< Circumscribed radius (m).
  double inscribed_radius {0.3};  ///< Largest circle inside the robot (m).
  /// The robot's polygon (robot frame); empty: a circle of robot_radius.
  std::vector<FootprintPoint> footprint;
  /// Penalized within safety_radius - robot_radius of the robot's body (m).
  double safety_radius {0.6};

  double path_weight {15.0};         ///< Distance to the path. Omni default: 40.
  double progress_weight {8.0};      ///< Path left to the end of the local path.
  double heading_weight {10.0};      ///< Error to the heading of heading_mode. Omni: 20.
  double heading_lookahead {0.5};    ///< "path" faces the path this far ahead (m).
  double goal_weight {5.0};          ///< Distance to the goal, when within reach.
  double goal_heading_weight {3.0};  ///< Error to the goal's heading, near the goal (omni).
  double goal_heading_distance {1.0};  ///< Near the goal: the goal's heading takes over (m).
  double obstacle_weight {20.0};     ///< Inside safety_radius, quadratic to the robot radius.
  double collision_cost {1.0e6};     ///< Each step in collision.
  double smoothness_weight {0.05};   ///< Accelerations between steps.
  double forward_weight {2.0};       ///< Backward velocity (differential robots).

  double grid_resolution {0.05};     ///< Resolution of the obstacle and path grids (m).
  uint32_t seed {42};                ///< Random seed (results are repeatable).

  /// \brief The defaults for \p model. A holonomic robot follows the path closely and keeps
  /// its heading on its own; a differential one steers with its heading, so it needs room to
  /// turn and leave the path.
  static MPPIParams defaults_for(MotionModel model)
  {
    MPPIParams p;
    p.motion_model = model;
    if (model == MotionModel::Omni) {
      p.path_weight = 40.0;
      p.heading_weight = 20.0;
    }
    return p;
  }
};

/// \brief What the optimizer is doing.
enum class MPPIStatus
{
  Following,  ///< Moving along the path.
  Aligning,   ///< At the goal position, turning to the goal's heading.
  Arrived,    ///< At the goal, within both tolerances: stopped.
  Blocked,    ///< Every rollout collides: stopping.
  NoPath,     ///< No valid path or pose: stopped.
};

/// \brief One cycle's input.
struct MPPIInput
{
  Pose2D pose;          ///< Robot pose (path frame).
  Velocity velocity;    ///< Current velocity (robot frame).
  const nav_msgs::msg::Path * path {nullptr};
  const pcl::PointCloud<pcl::PointXYZ> * obstacles {nullptr};  ///< Path frame.
  HeadingMode heading_mode {HeadingMode::Path};  ///< Hold is given as Fixed.
  double heading_yaw {0.0};      ///< Fixed heading (path frame).
  double elapsed {0.0};          ///< Since the previous cycle (s), to shift the last solution.
  double xy_goal_tolerance {0.25};
  double yaw_goal_tolerance {0.25};
};

/// \brief One cycle's output.
struct MPPIResult
{
  Velocity cmd;
  MPPIStatus status {MPPIStatus::NoPath};
};

/// \brief MPPI (Williams et al.): rollouts of noisy control sequences around the previous
/// solution, weighted by the exponential of their cost.
///
/// Every buffer is allocated in configure(): compute() does not allocate (except to grow the
/// seeds of a larger scan or path than ever before).
class MPPIOptimizer
{
public:
  /// \brief Applies \p params and resets the solution.
  void configure(const MPPIParams & params);

  const MPPIParams & params() const {return params_;}

  /// \brief Changes only the velocity and acceleration limits (e.g. a safely limited speed).
  void set_limits(const MPPIParams & limits);

  /// \brief Forgets the previous solution (a stopped robot starts from zero).
  void reset();

  /// \brief Computes the command for this cycle.
  MPPIResult compute(const MPPIInput & input);

  /// \brief Best rollout of the last cycle (the optimal sequence), horizon_steps + 1 poses.
  const std::vector<Pose2D> & optimal_trajectory() const {return optimal_;}

  /// \brief Poses of rollout \p k of the last cycle (horizon_steps + 1).
  const Pose2D * rollout(int k) const {return &poses_[static_cast<size_t>(k) * (steps_ + 1)];}

  /// \brief Rollouts evaluated in the last cycle (0 if it did not optimize).
  int rollouts() const {return last_rollouts_;}

  /// \brief The control sequence (warm start of the next cycle). Omni: vx, vy in the map frame.
  const std::vector<Velocity> & control_sequence() const {return control_;}

private:
  struct PathSample
  {
    double x;
    double y;
    double yaw;  ///< Tangent.
    double s;    ///< Arc length from the first sample.
    int ahead;   ///< Sample heading_lookahead ahead (or the last).
  };

  MPPIParams params_;
  int samples_ {0};
  int steps_ {0};

  std::vector<Velocity> control_;  ///< Mean sequence U (omni: vx, vy in the map frame).
  std::vector<Velocity> shifted_;  ///< Scratch: the warm start shift, the best rollout.
  std::vector<Velocity> sampled_;  ///< K x T.
  std::vector<Pose2D> poses_;      ///< K x (T + 1).
  std::vector<double> costs_;      ///< K.
  /// Constant targets tried every cycle besides the noisy ones: the search does not depend
  /// only on the last solution (local minima, e.g. in front of an obstacle).
  struct Primitive
  {
    Velocity first;   ///< Target until step \p switch_step,
    Velocity then;    ///< and after it.
    int switch_step;
    bool to_goal {false};  ///< Instead: straight to the goal, braking to stop there.
  };
  std::vector<Primitive> primitives_;
  std::vector<Pose2D> optimal_;    ///< T + 1.
  int last_rollouts_ {0};

  NearestGrid obstacle_grid_;
  PointBuckets obstacle_buckets_;  ///< The obstacles near a footprint (exact clearance).
  NearestGrid path_grid_;
  std::vector<PathSample> local_path_;
  bool local_path_reaches_goal_ {false};
  bool path_blocked_ {false};  ///< An obstacle on the local path, within reach.

  // Progress along the current path, to search the nearest pose near the last one.
  size_t closest_index_ {0};
  bool have_path_ {false};
  size_t path_size_ {0};
  double path_front_x_ {0.0};
  double path_front_y_ {0.0};
  double path_back_x_ {0.0};
  double path_back_y_ {0.0};
  bool aligning_ {false};  ///< Turning at the goal (hysteresis).

  std::mt19937 rng_;
  std::normal_distribution<double> normal_ {0.0, 1.0};

  /// \brief Farthest a rollout can go (m).
  double reach() const;
  /// \brief Shifts the solution by \p elapsed seconds (the robot already executed them).
  void shift_control(double elapsed);
  /// \brief Finds the pose of \p path nearest the robot and builds the local path from it.
  void update_local_path(const nav_msgs::msg::Path & path, const Pose2D & pose);
  /// \brief Adds the obstacles (and local path) to their grids, centered on \p pose.
  void build_grids(const Pose2D & pose, const pcl::PointCloud<pcl::PointXYZ> * obstacles);
  /// \brief Samples, simulates and scores rollout \p k; returns its cost. \p explore: noise
  /// around zero; \p primitive: its targets, without noise.
  double rollout_cost(
    int k, const MPPIInput & input, const Pose2D & goal, bool explore,
    const Primitive * primitive);
  /// \brief Deterministic candidates (arcs, turns, turn and go, straight lines), for a robot
  /// at \p yaw.
  void make_primitives(double yaw);
  /// \brief Adds the to-goal primitive when the goal is within reach.
  void add_to_goal_primitive();
  /// \brief Distance from the robot's body at \p pose to the nearest obstacle (< 0: overlap).
  /// Exact for a footprint wherever it matters (closer than the safety margin).
  double body_clearance(const Pose2D & pose) const;
  /// \brief Holonomic: scales (vx, vy) into the ellipse of the limits (no direction faster
  /// than the configured ones, diagonals included).
  void limit_translation(Velocity & v) const;
  /// \brief Target of the to-goal primitive at \p pose (map frame for omni, as the sequence).
  Velocity to_goal(const Pose2D & pose, const Pose2D & goal) const;
  /// \brief Turns in place towards \p yaw_error, braking in time.
  Velocity align(double yaw_error) const;
};

}  // namespace easynav

#endif  // EASYNAV_MPPI_CONTROLLER__MPPIOPTIMIZER_HPP_
