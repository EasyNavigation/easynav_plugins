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


#ifndef EASYNAV_OCTOMAP_MAPS_MANAGER__OBSTACLEFILTER_HPP_
#define EASYNAV_OCTOMAP_MAPS_MANAGER__OBSTACLEFILTER_HPP_

#include <string>

#include "pluginlib/class_loader.hpp"

#include "octomap_core/Octomap.hpp"
#include "easynav_common/types/NavState.hpp"

#include "easynav_octomap_maps_manager/filters/OctomapFilter.hpp"

namespace easynav
{
namespace octomap
{

class ObstacleFilter : public OctomapFilter
{
public:
  ObstacleFilter();

  virtual void on_initialize();
  virtual void update(::easynav::NavState & nav_state);

  bool is_adding_layer() override {return true;}
  std::string get_layer_name() override {return "obstacles";}

private:
  ::octomap::Octomap octomap_;
};

}  // namespace octomap
}  // namespace easynav
#endif  // EASYNAV_OCTOMAP_MAPS_MANAGER__OBSTACLEFILTER_HPP_
