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
/// \brief Declaration of the ForceReplanRecovery plugin.

#ifndef EASYNAV_FORCE_REPLAN_RECOVERY__FORCEREPLANRECOVERY_HPP_
#define EASYNAV_FORCE_REPLAN_RECOVERY__FORCEREPLANRECOVERY_HPP_

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class ForceReplanRecovery
 * @brief Level-1 mitigation: forces an immediate replan. See docs/recoveries_easynav.md §5.11.
 *
 * Selected for diagnostics with hardware_id == "planner" (the vocabulary shared with
 * NoPathEvaluator) at level ERROR or above (an empty/missing path is not itself an error while
 * the planner simply hasn't run yet — WARN — only an ERROR, i.e. the planner actually produced
 * an empty path, warrants forcing a retry).
 *
 * Does not requires_control(): it never touches "cmd_vel" or "control_owner", only requests
 * PlannerMethodBase::force_update() (already used for "a new goal arrived") via the one-shot
 * NavState key "force_replan_requested", consumed by SystemNode::system_cycle() right before
 * it cycles PlannerNode — see docs/recoveries_easynav_implementation.md, Fase 4. This is a
 * request, not a guarantee: whether the new attempt actually produces a non-empty path is up
 * to the planner plugin and the environment, not this mitigation, which always reports
 * SUCCEEDED once the request has been handed off (one non-RT cycle later). If the diagnostic
 * persists, RecoveryManagerNode's per-diagnostic attempt count (see RecoveryManagerNode.hpp)
 * lets a later candidate (e.g. ClearMapRecovery) take over instead of retrying this one
 * forever.
 */
class ForceReplanRecovery : public easynav::RecoveryMitigationBase
{
public:
  ForceReplanRecovery() = default;
  ~ForceReplanRecovery() = default;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;
};

}  // namespace easynav

#endif  // EASYNAV_FORCE_REPLAN_RECOVERY__FORCEREPLANRECOVERY_HPP_
