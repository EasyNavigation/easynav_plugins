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

#include "easynav_bigmap_maps_manager/marker_publisher.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace bigmap_maps_manager
{

namespace
{
std_msgs::msg::ColorRGBA lod_color(float leg, float min_leg, float max_leg)
{
  // log scale: each doubling of the triangle size moves the same amount along green -> red
  const float t = std::clamp(
    std::log2(leg / min_leg) / std::max(1e-3f, std::log2(max_leg / min_leg)), 0.0f, 1.0f);
  std_msgs::msg::ColorRGBA c;
  c.r = t;
  c.g = 1.0f - 0.6f * t;
  c.b = 0.15f;
  c.a = 1.0f;
  return c;
}

geometry_msgs::msg::Point pt(float x, float y, float z)
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}
}  // namespace

visualization_msgs::msg::MarkerArray make_markers(
  const LodMesh & mesh, const LodParams & params, double robot_x, double robot_y, double robot_z,
  double max_leg, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  double outer_radius)
{
  using visualization_msgs::msg::Marker;
  const float min_leg = static_cast<float>(std::max(1e-3, params.min_size));
  const std::size_t nt = mesh.triangle_count();

  Marker tris;
  tris.header.frame_id = frame_id;
  tris.header.stamp = stamp;
  tris.ns = "lod_triangles";
  tris.id = 0;
  tris.type = Marker::TRIANGLE_LIST;
  tris.action = Marker::ADD;
  tris.pose.orientation.w = 1.0;
  tris.scale.x = tris.scale.y = tris.scale.z = 1.0;
  tris.color.a = 1.0;
  tris.points.reserve(nt * 3);
  tris.colors.reserve(nt * 3);

  Marker wire = tris;
  wire.ns = "lod_wireframe";
  wire.type = Marker::LINE_LIST;
  wire.scale.x = 0.05;
  wire.color.r = wire.color.g = wire.color.b = 0.05f;
  wire.colors.clear();
  wire.points.clear();
  wire.points.reserve(nt * 6);

  for (std::size_t t = 0; t < nt; ++t) {
    const auto col = lod_color(mesh.leg[t], min_leg, static_cast<float>(max_leg));
    uint32_t v[3] = {mesh.tri[3 * t], mesh.tri[3 * t + 1], mesh.tri[3 * t + 2]};
    for (int k = 0; k < 3; ++k) {
      tris.points.push_back(pt(mesh.x[v[k]], mesh.y[v[k]], mesh.z[v[k]]));
      tris.colors.push_back(col);
      const uint32_t a = v[k], b = v[(k + 1) % 3];
      wire.points.push_back(pt(mesh.x[a], mesh.y[a], mesh.z[a] + 0.05f));
      wire.points.push_back(pt(mesh.x[b], mesh.y[b], mesh.z[b] + 0.05f));
    }
  }

  Marker ring;
  ring.header = tris.header;
  ring.ns = "lod_high_res_radius";
  ring.id = 0;
  ring.type = Marker::LINE_STRIP;
  ring.action = Marker::ADD;
  ring.pose.orientation.w = 1.0;
  ring.scale.x = 0.25;
  ring.color.r = 0.1f; ring.color.g = 0.4f; ring.color.b = 1.0f; ring.color.a = 1.0f;
  constexpr int kSegments = 96;
  for (int i = 0; i <= kSegments; ++i) {
    const double a = 2.0 * M_PI * i / kSegments;
    ring.points.push_back(pt(
        static_cast<float>(robot_x + params.high_res_radius * std::cos(a)),
        static_cast<float>(robot_y + params.high_res_radius * std::sin(a)),
        static_cast<float>(robot_z + 0.5)));
  }

  visualization_msgs::msg::MarkerArray arr;
  arr.markers = {tris, wire, ring};
  if (outer_radius > 0.0) {
    // the larger circle: inside it, ground the robot has driven over keeps full resolution
    Marker outer = ring;
    outer.ns = "lod_traversed_radius";
    outer.color.r = 1.0f; outer.color.g = 0.6f; outer.color.b = 0.1f;
    outer.points.clear();
    for (int i = 0; i <= kSegments; ++i) {
      const double a = 2.0 * M_PI * i / kSegments;
      outer.points.push_back(pt(
          static_cast<float>(robot_x + outer_radius * std::cos(a)),
          static_cast<float>(robot_y + outer_radius * std::sin(a)),
          static_cast<float>(robot_z + 0.5)));
    }
    arr.markers.push_back(outer);
  }
  return arr;
}

}  // namespace bigmap_maps_manager
