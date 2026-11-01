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

#ifndef EASYNAV_BONXAI_PLANNER__BONXAIPLANNER_HPP_
#define EASYNAV_BONXAI_PLANNER__BONXAIPLANNER_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_bonxai_planner/Planner3D.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

/// @brief A* in 3D over a Bonxai map ("map.bonxai"), for aerial robots: the path keeps
/// min_clearance from every occupied voxel, and prefers preferred_clearance.
class BonxaiPlanner : public PlannerMethodBase
{
public:
  BonxaiPlanner() = default;
  ~BonxaiPlanner() override = default;

  void on_initialize() override;
  void update(NavState & nav_state) override;

  /// The grid the last plan used (tests).
  const bonxai_planner::ClearanceGrid & grid() const {return grid_;}

protected:
  void clear_path(NavState & nav_state);
  /// The occupied voxels to the clearance grid, when the map changed.
  bool refresh_grid(const NavState & nav_state);

  bonxai_planner::ClearanceGrid grid_;
  bonxai_planner::PlannerParams params_;
  bonxai_planner::SmootherParams smoother_;
  bonxai_planner::FillBelow fill_below_ {bonxai_planner::FillBelow::Lowest};
  double resolution_ {1.0};
  double margin_ {10.0};
  double min_altitude_ {std::numeric_limits<double>::quiet_NaN()};
  double max_altitude_ {std::numeric_limits<double>::quiet_NaN()};
  double path_step_ {0.5};
  bool continuous_replan_ {true};

  const void * map_ptr_ {nullptr};
  std::size_t map_voxels_ {0};

  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Pose current_goal_;
  bool path_published_ {false};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace easynav

#endif  // EASYNAV_BONXAI_PLANNER__BONXAIPLANNER_HPP_
