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


#ifndef EASYNAV_NAVMAP_MAPS_MANAGER__OBSTACLEFILTER_HPP_
#define EASYNAV_NAVMAP_MAPS_MANAGER__OBSTACLEFILTER_HPP_

#include <string>

#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

namespace easynav
{
namespace navmap
{

class ObstacleFilter : public NavMapFilter
{
public:
  ObstacleFilter();

  virtual void on_initialize() override;
  virtual void update(::easynav::NavState & nav_state) override;

  virtual bool is_adding_layer() override {return true;}
  virtual std::string get_layer_name() override {return "obstacles";}

private:
  ::navmap::NavMap navmap_;
};

}  // namespace navmap
}  // namespace easynav
#endif  // EASYNAV_NAVMAP_MAPS_MANAGER__OBSTACLEFILTER_HPP_
