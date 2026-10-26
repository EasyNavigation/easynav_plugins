// Copyright 2026 Intelligent Robotics Lab
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

#include "easynav_costmap_localizer/LocalizationQuality.hpp"

#include <algorithm>
#include <cmath>

namespace easynav
{

LocalizationQuality::LocalizationQuality(double alpha_slow, double alpha_fast)
: alpha_slow_(std::clamp(alpha_slow, 0.0, 1.0)), alpha_fast_(std::clamp(alpha_fast, 0.0, 1.0))
{
}

void LocalizationQuality::add(double mean_score, double best_score)
{
  if (!std::isfinite(mean_score) || !std::isfinite(best_score)) {
    return;
  }
  ++count_;
  has_data_ = true;
  // The plain mean of the scores so far until it weighs less than alpha.
  const double warm = 1.0 / static_cast<double>(count_);
  const double a_fast = std::max(alpha_fast_ > 0.0 ? alpha_fast_ : 1.0, warm);
  best_ += a_fast * (best_score - best_);
  if (averaging()) {
    slow_ += std::max(alpha_slow_, warm) * (mean_score - slow_);
    fast_ += a_fast * (mean_score - fast_);
  }
}

double LocalizationQuality::quality_ratio() const
{
  if (!has_data_ || !averaging() || slow_ <= 1e-9) {
    return 1.0;
  }
  return fast_ / slow_;
}

double LocalizationQuality::injection_probability() const
{
  return std::max(0.0, 1.0 - quality_ratio());
}

void LocalizationQuality::reset()
{
  has_data_ = false;
  count_ = 0;
  best_ = slow_ = fast_ = 0.0;
}

}  // namespace easynav
