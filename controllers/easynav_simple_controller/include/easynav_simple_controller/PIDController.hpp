// Copyright 2021 Intelligent Robotics Lab
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
/// \brief Declaration of a simple PIDController class.

#ifndef EASYNAV_SIMPLE_CONTROLLER__PIDCONTROLLER_HPP_
#define EASYNAV_SIMPLE_CONTROLLER__PIDCONTROLLER_HPP_

#include <cmath>

namespace easynav
{

/// \brief A simple PID (Proportional-Integral-Derivative) controller implementation.
class PIDController
{
public:
  /**
   * @brief Constructor.
   *
   * Initializes the PID controller with output and reference range limits.
   *
   * @param min_ref Minimum allowed input reference value.
   * @param max_ref Maximum allowed input reference value.
   * @param min_output Minimum output control value.
   * @param max_output Maximum output control value.
   */
  PIDController(double min_ref, double max_ref, double min_output, double max_output);

  /**
   * @brief Sets the PID gains.
   *
   * @param n_KP Proportional gain.
   * @param n_KI Integral gain.
   * @param n_KD Derivative gain.
   */
  void set_pid(double n_KP, double n_KI, double n_KD);

  /**
   * @brief Computes the control output for the given reference input.
   *
   * Applies the PID formula and returns the control command within the specified output limits.
   *
   * @param new_reference The current reference input value (e.g., error signal).
   * @return The computed control output.
   */
  double get_output(double new_reference);

private:
  double KP_;               ///< Proportional gain.
  double KI_;               ///< Integral gain.
  double KD_;               ///< Derivative gain.

  double min_ref_;          ///< Minimum allowed reference input.
  double max_ref_;          ///< Maximum allowed reference input.
  double min_output_;       ///< Minimum allowed output.
  double max_output_;       ///< Maximum allowed output.

  double prev_error_;       ///< Previous error value (for derivative term).
  double int_error_;        ///< Accumulated integral error.
};

}  // namespace easynav

#endif  // EASYNAV_SIMPLE_CONTROLLER__PIDCONTROLLER_HPP_
