// Copyright 2026 Intelligent Robotics Lab
//
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
/// \brief Declaration of the SimpleRecoveryManager class.

#ifndef EASYNAV_SIMPLE_RECOVERY__SIMPLERECOVERYMANAGER_HPP_
#define EASYNAV_SIMPLE_RECOVERY__SIMPLERECOVERYMANAGER_HPP_

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryManagerBase.hpp"

namespace easynav
{

/**
 * @class SimpleRecoveryManager
 * @brief A small, readable recovery system: a tutorial for writing your own.
 *
 * A recovery system is a RecoveryManagerBase plugin, loaded by recovery_node
 * ("recovery_node.recovery_manager.plugin: easynav_simple_recovery/SimpleRecoveryManager").
 * EasyNav calls it on two loops:
 *
 * - update_rt(), every RT cycle, right after the controller computed its command and before it is
 *   published. Only fast, bounded checks here. This one has:
 *   - A fast recovery: an obstacle right ahead while moving forward -> brake dead
 *     (override_velocity(): highest priority, not smoothed).
 *   - The execution of the mitigation chosen by update() (command_velocity(): takes over the
 *     controller, smoothed within the robot limits). A proposal lasts one RT cycle, so a
 *     mitigation is commanded on every cycle while it lasts.
 * - update(), every non-RT cycle, after the rest of EasyNav. Diagnose and decide, one case at a
 *   time, the most severe first:
 *   1. Sensors lost      -> request_shutdown(): nothing can be done safely. Lost means no new
 *                           data for sensors_timeout (real time, also since activation).
 *   2. Localization lost -> hold_mission_progress() and rotate in place to relocalize;
 *                           abort_mission() if it takes too long.
 *   3. Robot stuck       -> back up for a while. After too many attempts, reduce the robot's
 *                           speed (request_reconfigure(): changes a parameter and reconfigures
 *                           EasyNav, reloading this plugin). Still stuck: abort_mission().
 *                           The speed is restored when the mission ends
 *                           (request_restore_parameters()).
 *   4. Otherwise         -> no mitigation: the controller drives.
 *
 * Parameters (under "recovery_manager."): see on_initialize().
 */
class SimpleRecoveryManager : public RecoveryManagerBase
{
public:
  /// @brief What the robot is doing to recover, decided by update() and executed by update_rt().
  enum class Mitigation {NONE, ROTATE, BACK_UP};

  SimpleRecoveryManager() = default;
  ~SimpleRecoveryManager() override = default;

  void on_initialize() override;

  /// @brief EasyNav starts: sensor data is expected within sensors_timeout.
  void on_activate() override;

  /// @brief EasyNav stops: any mitigation in progress ends.
  void on_deactivate() override;

  /// @brief The mitigation in progress.
  [[nodiscard]] Mitigation get_mitigation() const {return mitigation_;}

protected:
  void update(NavState & nav_state) override;
  bool update_rt(NavState & nav_state) override;

private:
  // Cases, checked by update() (non-RT).
  bool sensors_lost(const NavState & nav_state);
  bool has_mission(const NavState & nav_state) const;
  bool localization_lost(const NavState & nav_state) const;
  bool robot_stuck(const NavState & nav_state);
  bool slowed_down(const NavState & nav_state) const;

  // Checked by update_rt() (RT).
  bool obstacle_ahead(const NavState & nav_state) const;
  bool moving_forward(const NavState & nav_state) const;

  // Mitigations.
  void start(Mitigation mitigation);
  void stop_mitigation();
  double elapsed_in_mitigation() const;
  geometry_msgs::msg::TwistStamped twist(double linear, double angular) const;

  // Parameters.
  double stop_distance_ {0.3};         // Brake if an obstacle is this close ahead [m]
  double robot_radius_ {0.3};          // Half width of the area checked ahead [m] (geometry)
  double min_obstacle_z_ {0.05};       // Ignore points below (floor) [m]
  double max_obstacle_z_ {0.5};        // Ignore points above the robot [m] (geometry)
  double sensors_timeout_ {5.0};       // No sensor data for this long: sensors lost [s]
  double max_position_variance_ {1.0}; // Above it (x or y): localization lost [m^2]
  double relocalize_timeout_ {20.0};   // Rotating longer than this: abort the mission [s]
  double rotate_speed_ {0.5};          // [rad/s]
  double stuck_time_ {10.0};           // Commanded but not moving for this long: stuck [s]
  double stuck_distance_ {0.05};       // Moving less than this is not moving [m]
  double backup_speed_ {0.1};          // [m/s]
  double backup_time_ {2.0};           // [s]
  int max_backup_attempts_ {3};        // Per mission, and again once slowed down
  double slow_down_max_linear_vel_ {0.1};  // Still stuck: max_linear_vel [m/s] (0: never)

  // Read by update_rt(), written by update().
  std::atomic<Mitigation> mitigation_ {Mitigation::NONE};

  // Only used by update().
  rclcpp::Time mitigation_start_;
  // Sensor data is timed with a steady clock: simulated time stops with the simulator.
  rclcpp::Clock steady_clock_ {RCL_STEADY_TIME};
  std::atomic<bool> restart_sensors_watch_ {true};  // Set by on_activate()
  int64_t newest_sensor_stamp_ {0};                 // Newest stamp seen [ns]
  rclcpp::Time newest_sensor_arrival_;              // When it was seen (steady)
  std::optional<geometry_msgs::msg::Point> stuck_reference_;
  rclcpp::Time stuck_reference_time_;
  int backup_attempts_ {0};
};

}  // namespace easynav

#endif  // EASYNAV_SIMPLE_RECOVERY__SIMPLERECOVERYMANAGER_HPP_
