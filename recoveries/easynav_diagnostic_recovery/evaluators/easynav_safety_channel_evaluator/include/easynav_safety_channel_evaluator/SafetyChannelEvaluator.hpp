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
/// \brief Declaration of the SafetyChannelEvaluator plugin.

#ifndef EASYNAV_SAFETY_CHANNEL_EVALUATOR__SAFETYCHANNELEVALUATOR_HPP_
#define EASYNAV_SAFETY_CHANNEL_EVALUATOR__SAFETYCHANNELEVALUATOR_HPP_

#include <optional>

#include "rclcpp/time.hpp"

#include "easynav_diagnostic_recovery/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class SafetyChannelEvaluator
 * @brief Level-1 recovery evaluator for protective stops of the safety channel.
 *
 * Reads "safety_status" (SafetyChannelState, see system_node's "safety.status.*"). During a
 * protective stop (or with the safety status lost) it reports WARN with hardware_id
 * "safety_channel"; once the stop lasts "max_stop_time" seconds (0: never), ERROR, so a
 * mitigation can handle it. EasyNav keeps the mission during the stop and resumes it after.
 */
class SafetyChannelEvaluator : public easynav_diagnostic_recovery::RecoveryEvaluatorBase
{
public:
  SafetyChannelEvaluator() = default;
  ~SafetyChannelEvaluator() = default;

  void on_initialize() override;

protected:
  void update(NavState & nav_state) override;

private:
  /// @brief Seconds of protective stop before reporting ERROR (0: only WARN).
  double max_stop_time_ {0.0};

  /// @brief When the current protective stop started.
  std::optional<rclcpp::Time> stop_start_;
};

}  // namespace easynav

#endif  // EASYNAV_SAFETY_CHANNEL_EVALUATOR__SAFETYCHANNELEVALUATOR_HPP_
