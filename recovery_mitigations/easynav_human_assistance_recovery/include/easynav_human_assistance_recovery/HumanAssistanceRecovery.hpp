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

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class HumanAssistanceRecovery
 * @brief Level-1 last-resort mitigation: asks a human operator for help.
 *
 * See docs/recoveries_easynav.md §5.15 — this is a simplified instance of the design's
 * HumanAssistanceRecovery (no `teleop` mode, no episode-id `ack`: the "ack" here is physical,
 * whatever fixed the problem shows up as the offending diagnostic going back to OK). Generic
 * and domain-agnostic by design: unlike SafeRetreatRecovery/AmclRelocalizeMitigation, it does
 * not know or care which component raised the diagnostic — can_handle() accepts any ERROR,
 * meant to be configured with the lowest priority (or listed last in "mitigation_types") so it
 * is only reached once every more specific mitigator has been tried and excluded (see
 * RecoveryManagerNode's per-diagnostic exclusion, docs/recoveries_easynav_implementation.md).
 *
 * Not to be confused with the now-removed NotifyAndHoldRecovery (Fase 4, retired in the
 * Sesión 11 of the implementation log), which treated its trigger as a mission failure
 * (`GoalManager::set_failed`). This one does not: once every diagnostic is observed back at OK
 * — presumably because a human fixed whatever was wrong — it returns control and the robot
 * resumes its current mission, exactly as §5.15 specifies for a human-certified recovery.
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
};

}  // namespace easynav

#endif  // EASYNAV_HUMAN_ASSISTANCE_RECOVERY__HUMANASSISTANCERECOVERY_HPP_
