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

#include <algorithm>

#include "easynav_simple_controller/PIDController.hpp"

namespace easynav
{

PIDController::PIDController(double min_ref, double max_ref, double min_output, double max_output)
{
  min_ref_ = min_ref;
  max_ref_ = max_ref;
  min_output_ = min_output;
  max_output_ = max_output;
  prev_error_ = int_error_ = 0.0;

  KP_ = 0.7;
  KI_ = 0.0; // start with no integral to avoid windup
  KD_ = 0.0;
  integral_limit_ = max_output_ * 2.0; // default integral clamp
}

void
PIDController::set_pid(double n_KP, double n_KI, double n_KD)
{
  KP_ = n_KP;
  KI_ = n_KI;
  KD_ = n_KD;
}

double
PIDController::get_output(double new_reference, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }

  double error = new_reference;

  // ignore very small errors
  if (std::fabs(error) < min_ref_) {
    // decay integrator slightly
    int_error_ *= 0.9;
    prev_error_ = error;
    return 0.0;
  }

  // Proportional term
  double p = KP_ * error;

  // Integral term with anti-windup
  int_error_ += error * dt;
  if (integral_limit_ > 0.0) {
    if (int_error_ * KI_ > integral_limit_) {int_error_ = integral_limit_ / KI_;}
    if (int_error_ * KI_ < -integral_limit_) {int_error_ = -integral_limit_ / KI_;}
  }
  double i = KI_ * int_error_;

  // Derivative term
  double deriv = (error - prev_error_) / dt;
  double d = KD_ * deriv;
  prev_error_ = error;

  double output = p + i + d;

  // Saturate output to configured limits
  output = std::clamp(output, -max_output_, max_output_);

  return output;
}

void
PIDController::reset()
{
  prev_error_ = 0.0;
  int_error_ = 0.0;
}

}  // namespace easynav
