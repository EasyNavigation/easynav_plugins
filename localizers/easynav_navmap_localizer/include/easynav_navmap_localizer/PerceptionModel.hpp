// Copyright 2025 Intelligent Robotics Lab
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


#ifndef EASYNAV_NAVMAP_LOCALIZER__PERCEPTIONMODEL_HPP_
#define EASYNAV_NAVMAP_LOCALIZER__PERCEPTIONMODEL_HPP_

#include "easynav_common/types/NavState.hpp"
#include "tf2/LinearMath/Vector3.hpp"

namespace easynav
{
namespace navmap
{

class PerceptionModel
{
public:
  PerceptionModel() = default;

  virtual void update(NavState & nav_state) = 0;
  virtual bool hit(tf2::Vector3 & pt) = 0;
};

}  // namespace navmap

}  // namespace easynav
#endif  // EASYNAV_NAVMAP_LOCALIZER__PERCEPTIONMODEL_HPP_
