// Copyright 2026 José Miguel Guerrero
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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__MARKER_PUBLISHER_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__MARKER_PUBLISHER_HPP_

#include <string>

#include "visualization_msgs/msg/marker_array.hpp"

#include "easynav_bigmap_maps_manager/lod_mesher.hpp"

namespace bigmap_maps_manager
{

/// Builds the RViz view of a LOD mesh:
///  - "lod_triangles": TRIANGLE_LIST, per-vertex colour = leg size (green small -> red large)
///  - "lod_wireframe": LINE_LIST of the triangle edges (makes the resolution change obvious)
///  - "lod_high_res_radius": LINE_STRIP circle of high_res_radius around the robot
///  - "lod_traversed_radius": (if outer_radius > 0) the larger circle, in orange
visualization_msgs::msg::MarkerArray make_markers(
  const LodMesh & mesh, const LodParams & params, double robot_x, double robot_y, double robot_z,
  double max_leg, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  double outer_radius = 0.0);

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__MARKER_PUBLISHER_HPP_
