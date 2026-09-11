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
/// \brief Declaration of the ClearMapRecovery plugin.

#ifndef EASYNAV_CLEAR_MAP_RECOVERY__CLEARMAPRECOVERY_HPP_
#define EASYNAV_CLEAR_MAP_RECOVERY__CLEARMAPRECOVERY_HPP_

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class ClearMapRecovery
 * @brief Level-1 mitigation: resets/clears whatever a MapsManagerBase plugin considers
 * clearable. See docs/recoveries_easynav.md §5.11 (Nav2 Clear*Costmap precedent).
 *
 * Selected, like ForceReplanRecovery, for diagnostics with hardware_id == "planner" at level
 * ERROR or above — the same "no_path" symptom NoPathEvaluator publishes. This is a deliberate
 * simplification versus the design's more general ambition of a dedicated map-health
 * evaluator (see docs/recoveries_easynav_implementation.md, Fase 4): no such evaluator exists
 * yet, so for now this mitigation reuses the one generic "the planner can't find a path"
 * diagnostic, relying on RecoveryManagerNode's per-diagnostic attempt count (mitigation_types
 * order: ForceReplanRecovery first, then this one) to be tried only after ForceReplanRecovery
 * has already had its chance and failed to unstick the planner.
 *
 * Does not requires_control(): only requests MapsManagerNode::reset() via the one-shot
 * NavState key "maps_manager_reset_requested", consumed by SystemNode::system_cycle() right
 * after it cycles MapsManagerNode. Whether that reset actually clears anything depends on
 * whether any loaded MapsManagerBase plugin overrides reset() at all (the default is a no-op);
 * either way this mitigation reports SUCCEEDED once the request has been handed off.
 */
class ClearMapRecovery : public easynav::RecoveryMitigationBase
{
public:
  ClearMapRecovery() = default;
  ~ClearMapRecovery() = default;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;

protected:
  void on_start(NavState & nav_state) override;
  RecoveryStatus on_cycle(NavState & nav_state) override;
};

}  // namespace easynav

#endif  // EASYNAV_CLEAR_MAP_RECOVERY__CLEARMAPRECOVERY_HPP_
