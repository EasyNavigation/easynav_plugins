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

/// \file
/// \brief The map's self-similarities: for sampled places, the other poses of the map whose
/// view is the same. Computed once from the map; online it says which hypotheses a pose needs.

#ifndef EASYNAV_ORBIT_LOCALIZER__SYMMETRYGRAPH_HPP_
#define EASYNAV_ORBIT_LOCALIZER__SYMMETRYGRAPH_HPP_

#include <atomic>
#include <functional>
#include <cmath>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "easynav_orbit_localizer/DistanceField.hpp"
#include "easynav_orbit_localizer/Lie.hpp"
#include "easynav_orbit_localizer/Matcher.hpp"

namespace easynav
{
namespace orbit
{

/// @brief What a sensor sees from a place: points (in the place's frame, heading 0), a
/// rotation-invariant descriptor, and a heading profile to align two views in yaw.
struct View
{
  std::vector<Eigen::Vector3d> points;
  std::vector<float> descriptor;
  std::vector<float> profile;
};

/// @brief Makes views, from the map (virtual) and from a real scan with the same recipe.
class ViewMaker
{
public:
  virtual ~ViewMaker() = default;
  /// @brief True when the views have no height: the coverage then compares in the plane.
  virtual bool planar() const {return false;}
  /// @brief Rays a view casts: a view with far fewer returns cannot judge the coverage.
  virtual std::size_t rays() const = 0;
  virtual View fromMap(const Eigen::Vector3d & position) const = 0;
  virtual View fromScan(const std::vector<Eigen::Vector3d> & body_points) const = 0;
  /// @brief The places of the map to sample.
  virtual std::vector<Eigen::Vector3d> samplePositions() const = 0;
};

/// @brief Views of a 2D grid map: ray casts of a 360 degree lidar.
class GridViews2D : public ViewMaker
{
public:
  bool planar() const override {return true;}
  std::size_t rays() const override {return static_cast<std::size_t>(n_rays_);}
  GridViews2D(const GridDistanceField2D & field, double max_range, double sample_step);
  View fromMap(const Eigen::Vector3d & position) const override;
  View fromScan(const std::vector<Eigen::Vector3d> & body_points) const override;
  std::vector<Eigen::Vector3d> samplePositions() const override;

private:
  View fromRanges(const std::vector<double> & ranges, double z) const;
  const GridDistanceField2D & field_;
  double max_range_, sample_step_;
  int n_rays_ {180};
};

/// @brief Views of a 3D point map: a lidar simulated from the place, rays cast through the
/// occupied voxels on a grid of azimuths and elevations, in a height band; a real scan is
/// binned on the same grid (nearest return per cell), so both views have the same recipe and
/// what is behind a wall is in neither. Places sampled on the ground (lowest map point of the
/// column) or at given heights.
struct ViewRays
{
  double sensor_height {0.5};    // m, the sensor over the place (a lidar over the footprint)
  int azimuths {90};
  int elevations {12};
  double elevation_min {-0.44};  // rad
  double elevation_max {0.26};
};

class CloudViews3D : public ViewMaker
{
public:
  std::size_t rays() const override
  {
    return static_cast<std::size_t>(rays_.azimuths) * static_cast<std::size_t>(rays_.elevations);
  }
  using Rays = ViewRays;
  CloudViews3D(
    const CloudDistanceField3D & field, double max_range, double z_min, double z_max,
    double sample_step, const std::vector<double> & altitudes, double ground_height,
    const Rays & rays = Rays());
  View fromMap(const Eigen::Vector3d & position) const override;
  View fromScan(const std::vector<Eigen::Vector3d> & body_points) const override;
  std::vector<Eigen::Vector3d> samplePositions() const override;
  /// @brief Places given from outside (a navigable-surface map knows the floors): used
  /// instead of the ground of the point columns.
  void setPlaces(std::vector<Eigen::Vector3d> places) {places_ = std::move(places);}

private:
  /// @brief The view of a range image (one range per azimuth x elevation cell, max_range or
  /// more for no return), in the place's frame.
  View fromRanges(const std::vector<double> & ranges) const;
  const CloudDistanceField3D & field_;
  double max_range_, z_min_, z_max_, sample_step_;
  std::vector<double> altitudes_;
  double ground_height_;
  Rays rays_;
  std::vector<Eigen::Vector3d> places_;
};

/// @brief How much of what the map shows from a place the scan explains: the fraction of the
/// view's points (place frame, heading 0, as fromMap gives them) with a scan point within
/// inlier_distance once the scan (body frame) is placed at X and the view at its place. A
/// scan that fits the map where the map is dense but shows none of it is not there.
/// @brief What of a map's view the robot's sensor could see: its band of heights, its field
/// of view (azimuth about the heading, elevations), and whether the view is planar (2D).
struct CoverageBand
{
  bool planar {false};
  double z_min {-1e9};
  double z_max {1e9};
  double azimuth_fov {2.0 * M_PI};   // rad, centred on the heading
  double elevation_min {-M_PI_2};
  double elevation_max {M_PI_2};
};

/// @brief Fraction of the view's points (the place's frame, heading 0) that the sensor could see
/// at X and that have a point of the scan within the inlier distance.
double coverage(
  const std::vector<Eigen::Vector3d> & view_points, const Eigen::Vector3d & place,
  const std::vector<Eigen::Vector3d> & body_points, const Pose & X, double inlier_distance,
  const CoverageBand & band = CoverageBand());

/// @brief A place and the other poses of the map that look the same from it.
struct Alternative
{
  Pose T_node_alt;      // the alternative pose in the node's frame
  double score {0.0};   // how well the node's view fits the map there (inlier fraction)
  int node {-1};        // the node the alternative pose is at, if any
};

struct Node
{
  Eigen::Vector3d position;
  std::vector<float> descriptor;
  std::vector<float> profile;
  std::vector<Eigen::Vector3d> points;   // the view from the place (its frame, heading 0)
  std::vector<Alternative> alternatives;
};

struct GraphOptions
{
  double sample_step {0.5};      // m, between the sampled places (the view maker's)
  int candidates {12};           // nodes with the closest descriptors tried per node
  int yaw_seeds {2};             // headings from the profile correlation tried per candidate
  double min_score {0.95};       // an alternative needs this fraction of the node's own score
  double merge_distance {1.0};   // m, closer poses are the same place
  double merge_angle {0.5};      // rad
  int threads {4};
};

/// @brief A pose found by a global match of a scan.
struct Candidate
{
  Pose pose;
  double score {0.0};
};

class SymmetryGraph
{
public:
  /// @brief Builds the graph: views of every sampled place, and their alternatives verified by
  /// the matcher. progress(fraction) is called now and then; it returns false to abort.
  void build(
    const ViewMaker & views, const Matcher & matcher, const GraphOptions & options,
    const std::function<bool(double)> & progress = {});

  /// @brief Nearest sampled place to p (null if none within two sample steps).
  const Node * nearest(const Eigen::Vector3d & p) const;

  /// @brief The poses that look like X: for each alternative of X's place, X carried there.
  std::vector<Candidate> twins(const Pose & X) const;

  /// @brief Where a scan fits the map: the places with the closest descriptors, aligned in
  /// heading by the profiles, refined by the matcher and scored where they land (the scan's
  /// fit and how much of the place's view it explains). Best first.
  std::vector<Candidate> globalMatch(
    const ViewMaker & views, const Matcher & matcher,
    const std::vector<Eigen::Vector3d> & body_points, int candidates, int yaw_seeds,
    double min_score, const std::vector<Eigen::Vector3d> * view_points = nullptr,
    const CoverageBand & band = CoverageBand()) const;

  /// @brief Diagnostics: the rank (0 = closest) of the place nearest to p among all places by
  /// descriptor distance to the given one, and that distance; -1 if no place is near p.
  int descriptorRank(
    const std::vector<float> & descriptor, const Eigen::Vector3d & p, float * distance) const;
  /// @brief Saves the graph with a fingerprint of the map and the options it was built from.
  bool save(const std::string & path, uint64_t fingerprint = 0) const;
  /// @brief Loads a graph; with a fingerprint, only one built from the same map and options.
  bool load(const std::string & path, uint64_t fingerprint = 0);

  const std::vector<Node> & nodes() const {return nodes_;}
  std::size_t alternativesCount() const;
  double sampleStep() const {return step_;}

  /// @brief Headings (rad) that best align profile b onto a, best first.
  static std::vector<double> yawSeeds(
    const std::vector<float> & a, const std::vector<float> & b, int n);

  /// @brief L1 distance of two descriptors.
  static float descriptorDistance(const std::vector<float> & a, const std::vector<float> & b);

private:
  void index();
  std::vector<int> closestNodes(const std::vector<float> & descriptor, int n, int exclude) const;

  std::vector<Node> nodes_;
  double step_ {0.5};
  std::unordered_map<int64_t, std::vector<int>> cells_;
  int64_t cellKey(const Eigen::Vector3d & p) const;
};

}  // namespace orbit
}  // namespace easynav

#endif  // EASYNAV_ORBIT_LOCALIZER__SYMMETRYGRAPH_HPP_
