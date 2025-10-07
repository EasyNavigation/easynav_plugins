// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


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
