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
/// \brief Declaration of the HumanAssistanceRecovery plugin.

#ifndef EASYNAV_HUMAN_ASSISTANCE_RECOVERY__HUMANASSISTANCERECOVERY_HPP_
#define EASYNAV_HUMAN_ASSISTANCE_RECOVERY__HUMANASSISTANCERECOVERY_HPP_

#include <optional>

#include "rclcpp/time.hpp"

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class HumanAssistanceRecovery
 * @brief Level-1 last-resort mitigation: asks a human operator for help.
 *
 * Deliberately simplified: no `teleop` mode, no episode-id `ack` — the "ack" here is physical,
 * whatever fixed the problem shows up as the offending diagnostic going back to OK. Generic and
 * domain-agnostic: unlike SafeRetreatRecovery/AmclRelocalizeMitigation, it does not know or care
 * which component raised the diagnostic — can_handle() accepts any ERROR, meant to be configured
 * with the lowest priority (or listed last in "mitigation_types") so it is only reached once
 * every more specific mitigator has been tried and excluded.
 *
 * Once every diagnostic is observed back at OK — presumably because a human fixed whatever was
 * wrong — it returns control and the robot resumes its current mission; it never fails the
 * mission itself (that is a different mitigation's job).
 *
 * Optionally bounded by "timeout" (seconds, default 0.0 = wait forever): if a human has not
 * fixed things within that time, this mitigation gives up (FAILED) instead of waiting
 * indefinitely, so a lower-priority candidate — e.g. a mission-level "give up" mitigation — can
 * take over.
 */
class HumanAssistanceRecovery : public easynav::RecoveryMitigationBase
{
public:
  HumanAssistanceRecovery() = default;
  ~HumanAssistanceRecovery() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return true;}

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  /// @brief Seconds to wait before giving up. 0.0 (the default) means wait forever.
  double timeout_ {0.0};

  rclcpp::Time start_time_;

  /// @brief When the last "still waiting" report() was sent, to throttle it to once per
  /// wait_report_period_ instead of every RT cycle.
  std::optional<rclcpp::Time> last_wait_report_;

  static constexpr double wait_report_period_ {10.0};
};

}  // namespace easynav

#endif  // EASYNAV_HUMAN_ASSISTANCE_RECOVERY__HUMANASSISTANCERECOVERY_HPP_
