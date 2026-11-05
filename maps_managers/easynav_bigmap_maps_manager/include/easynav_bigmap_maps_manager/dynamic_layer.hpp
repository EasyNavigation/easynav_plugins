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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__DYNAMIC_LAYER_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__DYNAMIC_LAYER_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include <limits>

#include "easynav_bigmap_maps_manager/lod_mesher.hpp"

namespace bigmap_maps_manager
{

/// Finds the triangle of a LodMesh that contains a point (x, y). Built once per mesh.
class TriangleLocator
{
public:
  explicit TriangleLocator(const LodMesh & mesh, double cell = 8.0);
  /// Index of the triangle containing (x, y), or -1.
  int locate(double x, double y) const;
  /// With several surfaces over (x, y) (floors of a building): the highest one that is not above
  /// `z` + 0.25 m, or the lowest if all are. NaN `z`: the first one found.
  int locate_at(double x, double y, double z) const;

private:
  const LodMesh * mesh_;
  double cell_, x0_, y0_;
  int nx_, ny_;
  std::vector<std::vector<std::uint32_t>> bins_;
};

struct DynamicParams
{
  double max_range{5.0};     ///< only sensor points this close to the robot (m)
  /// A point must be this far above the plane of the NavCel it falls in, measured along that
  /// NavCel's normal (m): a ramp, however steep, is the NavCel's own plane and never counts.
  double min_height{0.3};
  /// ... and the points of its cell must span at least this much in height. An object has
  /// vertical extent; ground that the terrain model puts a few decimetres too low (the DEM is never
  /// exact) is a flat surface and spans only what its slope gives within one cell, so it is not
  /// mistaken for an obstacle however far it is above the model.
  double min_vertical_extent{0.2};
  /// A detection counts only once it has been seen in the same cell for this long (s). An object
  /// stays where it is; a fleeting error (the robot's attitude changing faster than the sensors
  /// and the IMU agree on) moves to other cells in the next frame and never gets confirmed.
  double confirm_time{0.3};
  double max_height{3.0};    ///< ... and no higher than this above the robot's ground (m)
  double forget_time{5.0};   ///< a detection not seen again for this long is dropped (s)
  double cell{0.25};         ///< detections are merged on a grid of this size (m)
  double voxel{0.1};         ///< sensor clouds are thinned to one point per voxel this big (m)

  /// Local ground model from the sensor itself. In a window in front of the robot (window_length
  /// ahead, window_width across, in the robot's frame) the lowest point of every cell is taken
  /// and a plane is fitted to them (RANSAC, tilt limited so a wall can't be the plane). A cell
  /// inside the window is an obstacle only if its highest point also rises min_height above THAT
  /// plane (along its normal): a ramp, up or down, is a plane and its points are all on it,
  /// whatever the map says (ground_tolerance decides which lowest points fit the plane). Outside
  /// the window, or with fewer than ground_min_cells cells, only the map is used.
  bool ground_fit{true};
  double window_length{4.0};   ///< m ahead of the robot
  double window_width{3.0};    ///< m across (centred on the robot)
  double window_z_min{-1.0};   ///< body-frame z range of the points that can be ground (m)
  double window_z_max{2.5};
  double ground_tolerance{0.08};  ///< a cell's lowest point within this of the plane is ground (m)
  double max_ground_tilt_deg{40.0};
  int ground_min_cells{12};
};

/// A NavCel raised to what the sensor measured.
struct RaisedTile
{
  std::uint32_t triangle;  ///< index in the LodMesh
  float z_top;             ///< height of the NavCel now (map frame, relative to the DEM reference)
  float above_surface;     ///< z_top minus the mean terrain height of the NavCel
};

/// A detection that has gone on for another `min_time` seconds (see DynamicLayer::sustained)
struct SustainedHit
{
  float x, y;       ///< map frame (m)
  float z;          ///< highest point seen in the cell
  float surface;    ///< terrain height under it
};

/// Memory of what the sensors saw above the terrain, to raise the NavCels where it was.
///
/// Heights are measured relative to the robot and referred to the terrain under it
/// (`z = terrain(robot) + z of the point rotated by the robot's orientation`), not through the
/// map -> robot TF: the localization's z drifts by metres, while the robot's x, y and its
/// roll/pitch/yaw (from the IMU) are what is reliable.
class DynamicLayer
{
public:
  using Surface = std::function<double(double, double)>;  ///< terrain height at (x, y)

  /// `points`: sensor points in the robot's body frame (x forward, z up, z = 0 at the ground
  /// contact), which tilts with the robot. `q_xyzw`: the robot's full orientation in the map
  /// (roll, pitch and yaw): on a slope the sensors tilt with it, and ignoring that would turn the
  /// slope into obstacles. `now`: seconds. Returns true if the detections changed noticeably.
  ///
  /// With `mesh` and `locator` (those of the NavMap in use) the height of a point is measured
  /// against the plane of the NavCel it falls in, along its normal; points outside the mesh are
  /// ignored. Without them `terrain` is used and the height is the vertical one.
  bool ingest(
    const std::vector<std::array<float, 3>> & points, double robot_x, double robot_y,
    const std::array<double, 4> & q_xyzw, double now, const Surface & terrain,
    const DynamicParams & p, const LodMesh * mesh = nullptr,
    const TriangleLocator * locator = nullptr,
    double robot_z = std::numeric_limits<double>::quiet_NaN());

  /// The same for a robot that is level, with this heading.
  bool ingest(
    const std::vector<std::array<float, 3>> & points, double robot_x, double robot_y,
    double robot_yaw, double now, const Surface & terrain, const DynamicParams & p,
    const LodMesh * mesh = nullptr, const TriangleLocator * locator = nullptr);

  /// Drops detections not seen for p.forget_time (unconfirmed ones: for 0.5 s) and confirms those
  /// seen for p.confirm_time. Returns true if the confirmed set changed.
  bool expire(double now, const DynamicParams & p);

  /// The NavCels that contain confirmed detections, raised to the highest one seen in each. With
  /// `min_age` only detections that have been seen without a break (gaps over 1 s start the count
  /// again) for at least that long (s).
  std::vector<RaisedTile> tiles(
    const LodMesh & mesh, const TriangleLocator & locator, double min_age = 0.0) const;

  /// One SustainedHit per detection cell for every `min_time` seconds it has been seen without a
  /// break since the last call: 5 s over the same spot is one hit, 10 s two (reported by two
  /// calls). Detections are cells of `DynamicParams::cell`: the caller merges those of one NavCel.
  std::vector<SustainedHit> sustained(double now, double min_time);

  std::size_t size() const {return entries_.size();}
  void clear() {entries_.clear();}

private:
  struct Entry
  {
    float x, y, z, surface;
    double stamp;        ///< last time it was seen
    double first;        ///< first time it was seen
    bool confirmed;
    double streak;       ///< start of the current run seen without a gap over 1 s
    int periods;         ///< `min_time` periods of that run already reported by sustained()
  };
  std::unordered_map<std::int64_t, Entry> entries_;
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__DYNAMIC_LAYER_HPP_
