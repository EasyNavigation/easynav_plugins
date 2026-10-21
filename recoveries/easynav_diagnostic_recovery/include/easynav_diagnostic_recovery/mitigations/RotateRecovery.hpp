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

#ifndef EASYNAV_DIAGNOSTIC_RECOVERY__MITIGATIONS__ROTATERECOVERY_HPP_
#define EASYNAV_DIAGNOSTIC_RECOVERY__MITIGATIONS__ROTATERECOVERY_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "easynav_diagnostic_recovery/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class RotateRecovery
 * @brief Level-1 movement mitigation: turns in place (a full turn by default) when there is no
 *        path, so the sensors look all around before anyone is asked for help.
 *
 * With a map that remembers obstacles (e.g. the NavMap ObstacleFilter with memory), an obstacle
 * that has gone stays in the map until a sensor sees through it again; a turn in place clears
 * it, and the planner may find a path again. Selected for ERROR diagnostics whose hardware_id is
 * in "hardware_ids" ("planner": NoPathEvaluator). Takes control of "cmd_vel"; the level-0 reflex
 * still gates the motion.
 *
 * One activation turns "angle" at "angular_speed", then stands still "settle_time" seconds for
 * the planner to replan with what the sensors saw, and SUCCEEDS. It turns only once per episode:
 * if the diagnostic comes back within "episode_gap" seconds of the last activation, the turn did
 * not help, and it FAILS at once so RecoveryManagerNode escalates to the next mitigation (e.g.
 * HumanAssistanceRecovery). It also FAILS if the turn takes longer than "timeout" (blocked).
 */
class RotateRecovery : public easynav_diagnostic_recovery::RecoveryMitigationBase
{
public:
  RotateRecovery() = default;
  ~RotateRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  easynav_diagnostic_recovery::RecoveryStatus on_cycle(NavState & nav_state) override;
  void on_stop(NavState & nav_state) override;

private:
  double yaw(NavState & nav_state) const;

  double angle_ {6.283185307179586};  ///< rad to turn (a full turn); its sign is the direction
  double angular_speed_ {0.5};        ///< rad/s
  double settle_time_ {2.0};          ///< s standing still after the turn
  double timeout_ {0.0};              ///< s for the turn; <= 0: twice the expected plus 5 s
  double episode_gap_ {30.0};         ///< s: a new activation sooner is the same episode
  std::vector<std::string> hardware_ids_ {"planner"};

  bool done_this_episode_ {false};   ///< already turned in this episode
  bool give_up_ {false};             ///< this activation fails at once
  std::optional<rclcpp::Time> last_stop_time_;
  rclcpp::Time start_time_;
  std::optional<rclcpp::Time> settle_start_;
  double turned_ {0.0};   ///< rad turned in this activation (absolute)
  double last_yaw_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_DIAGNOSTIC_RECOVERY__MITIGATIONS__ROTATERECOVERY_HPP_
