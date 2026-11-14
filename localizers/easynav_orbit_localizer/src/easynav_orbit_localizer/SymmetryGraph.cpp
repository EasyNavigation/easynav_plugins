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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <thread>
#include <utility>
#include <unordered_map>
#include <vector>

#include "easynav_orbit_localizer/SymmetryGraph.hpp"

namespace easynav
{
namespace orbit
{

namespace
{
constexpr int PROFILE_BINS = 72;
constexpr int RANGE_BINS = 16;
constexpr int HEIGHT_BINS = 8;
constexpr std::size_t MAX_VIEW_POINTS = 400;

// Keeps at most n points, evenly spread
void capPoints(std::vector<Eigen::Vector3d> & pts, std::size_t n)
{
  if (pts.size() <= n) {return;}
  std::vector<Eigen::Vector3d> kept;
  kept.reserve(n);
  const double step = static_cast<double>(pts.size()) / static_cast<double>(n);
  for (std::size_t k = 0; k < n; ++k) {
    kept.push_back(pts[static_cast<std::size_t>(k * step)]);
  }
  pts.swap(kept);
}

int azimuthBin(double x, double y)
{
  const double a = std::atan2(y, x);
  int b = static_cast<int>(std::floor((a + M_PI) / (2.0 * M_PI) * PROFILE_BINS));
  return std::min(std::max(b, 0), PROFILE_BINS - 1);
}

bool samePose(const Pose & a, const Pose & b, double d, double angle)
{
  return (a.translation() - b.translation()).norm() < d &&
         std::abs(wrapAngle(yawOf(a) - yawOf(b))) < angle;
}
}  // namespace

// ---------------------------------------------------------------- GridViews2D

GridViews2D::GridViews2D(
  const GridDistanceField2D & field, double max_range, double sample_step)
: field_(field), max_range_(max_range), sample_step_(sample_step) {}

std::vector<Eigen::Vector3d> GridViews2D::samplePositions() const
{
  std::vector<Eigen::Vector3d> out;
  const double x0 = field_.originX(), y0 = field_.originY();
  const double x1 = x0 + field_.sizeX() * field_.resolution();
  const double y1 = y0 + field_.sizeY() * field_.resolution();
  for (double y = y0 + 0.5 * sample_step_; y < y1; y += sample_step_) {
    for (double x = x0 + 0.5 * sample_step_; x < x1; x += sample_step_) {
      if (!field_.isFree(x, y)) {continue;}
      // Not against a wall: a robot cannot be there and the view would be half occluded
      if (field_.distance(Eigen::Vector3d(x, y, 0.0), nullptr) < 0.25) {continue;}
      out.emplace_back(x, y, 0.0);
    }
  }
  return out;
}

View GridViews2D::fromRanges(const std::vector<double> & ranges, double z) const
{
  View v;
  v.descriptor.assign(RANGE_BINS + 1, 0.0f);
  v.profile.assign(PROFILE_BINS, 1.0f);
  const int n = static_cast<int>(ranges.size());
  for (int k = 0; k < n; ++k) {
    const double a = 2.0 * M_PI * (k + 0.5) / n - M_PI;   // the centre of the ray's bin
    double r = ranges[k];
    // A lone empty bin between two returns is a gap of the binning, not of the scene
    if (r >= max_range_ && n > 2) {
      const double before = ranges[(k + n - 1) % n], after = ranges[(k + 1) % n];
      if (before < max_range_ && after < max_range_) {r = 0.5 * (before + after);}
    }
    if (r >= max_range_) {
      v.descriptor[RANGE_BINS] += 1.0f;
      continue;
    }
    v.points.emplace_back(r * std::cos(a), r * std::sin(a), z);
    const int b = std::min(static_cast<int>(r / max_range_ * RANGE_BINS), RANGE_BINS - 1);
    v.descriptor[b] += 1.0f;
    const int pb = std::min(static_cast<int>(static_cast<double>(k) / n * PROFILE_BINS),
      PROFILE_BINS - 1);
    v.profile[pb] = std::min(v.profile[pb], static_cast<float>(r / max_range_));
  }
  for (auto & d : v.descriptor) {
    d /= static_cast<float>(std::max(n, 1));
  }
  capPoints(v.points, MAX_VIEW_POINTS);
  return v;
}

View GridViews2D::fromMap(const Eigen::Vector3d & position) const
{
  std::vector<double> ranges(n_rays_);
  for (int k = 0; k < n_rays_; ++k) {
    const double a = 2.0 * M_PI * (k + 0.5) / n_rays_ - M_PI;
    ranges[k] = field_.raycast(position.x(), position.y(), a, max_range_);
  }
  return fromRanges(ranges, position.z());
}

View GridViews2D::fromScan(const std::vector<Eigen::Vector3d> & body_points) const
{
  // The nearest return of each ray bin; an empty bin is a ray without a return
  std::vector<double> ranges(n_rays_, max_range_);
  for (const auto & p : body_points) {
    const double r = std::hypot(p.x(), p.y());
    if (r <= 0.0 || r >= max_range_) {continue;}
    const double a = std::atan2(p.y(), p.x());
    int k = static_cast<int>(std::floor((a + M_PI) / (2.0 * M_PI) * n_rays_));
    k = std::min(std::max(k, 0), n_rays_ - 1);
    ranges[k] = std::min(ranges[k], r);
  }
  return fromRanges(ranges, 0.0);
}

// ---------------------------------------------------------------- CloudViews3D

CloudViews3D::CloudViews3D(
  const CloudDistanceField3D & field, double max_range, double z_min, double z_max,
  double sample_step, const std::vector<double> & altitudes, double ground_height,
  const Rays & rays)
: field_(field), max_range_(max_range), z_min_(z_min), z_max_(z_max),
  sample_step_(sample_step), altitudes_(altitudes), ground_height_(ground_height), rays_(rays)
{
  rays_.azimuths = std::max(rays_.azimuths, 8);
  rays_.elevations = std::max(rays_.elevations, 1);
}

std::vector<Eigen::Vector3d> CloudViews3D::samplePositions() const
{
  if (!places_.empty()) {return places_;}
  // The lowest map point of every XY cell: the ground under a place, and where the map is
  const Eigen::Vector3d & lo = field_.min();
  const Eigen::Vector3d & hi = field_.max();
  const int nx = std::max(1, static_cast<int>(std::ceil((hi.x() - lo.x()) / sample_step_)));
  const int ny = std::max(1, static_cast<int>(std::ceil((hi.y() - lo.y()) / sample_step_)));
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> ground(static_cast<std::size_t>(nx) * ny, inf);
  for (const auto & p : field_.cloud()->points) {
    const int i = std::min(nx - 1, static_cast<int>((p.x - lo.x()) / sample_step_));
    const int j = std::min(ny - 1, static_cast<int>((p.y - lo.y()) / sample_step_));
    double & g = ground[static_cast<std::size_t>(j) * nx + i];
    g = std::min(g, static_cast<double>(p.z));
  }
  std::vector<Eigen::Vector3d> out;
  // Places near the map's structure: within half the range of a cell with points
  const int reach = std::max(1, static_cast<int>(0.5 * max_range_ / sample_step_));
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const double x = lo.x() + (i + 0.5) * sample_step_;
      const double y = lo.y() + (j + 0.5) * sample_step_;
      const double g = ground[static_cast<std::size_t>(j) * nx + i];
      if (altitudes_.empty()) {
        if (!std::isfinite(g)) {continue;}
        out.emplace_back(x, y, g + ground_height_);
        continue;
      }
      bool near = false;
      for (int dj = -reach; dj <= reach && !near; ++dj) {
        for (int di = -reach; di <= reach && !near; ++di) {
          const int a = i + di, b = j + dj;
          if (a < 0 || b < 0 || a >= nx || b >= ny) {continue;}
          near = std::isfinite(ground[static_cast<std::size_t>(b) * nx + a]);
        }
      }
      if (!near) {continue;}
      for (double z : altitudes_) {
        out.emplace_back(x, y, z);
      }
    }
  }
  return out;
}

View CloudViews3D::fromRanges(const std::vector<double> & ranges) const
{
  // Cell (a, e): azimuth bin a, elevation bin e; the ray at the cell's centre
  View v;
  v.descriptor.assign(RANGE_BINS * HEIGHT_BINS + 1, 0.0f);
  v.profile.assign(PROFILE_BINS, 1.0f);
  const int na = rays_.azimuths, ne = rays_.elevations;
  const Eigen::Vector3d origin(0.0, 0.0, rays_.sensor_height);
  std::size_t returns = 0;
  for (int e = 0; e < ne; ++e) {
    const double el = ne == 1 ? 0.5 * (rays_.elevation_min + rays_.elevation_max) :
      rays_.elevation_min + (e + 0.5) * (rays_.elevation_max - rays_.elevation_min) / ne;
    for (int a = 0; a < na; ++a) {
      const double az = 2.0 * M_PI * (a + 0.5) / na - M_PI;
      const double r = ranges[static_cast<std::size_t>(e) * na + a];
      if (!(r < max_range_)) {continue;}
      const Eigen::Vector3d p = origin + r * Eigen::Vector3d(
        std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      if (p.z() < z_min_ || p.z() > z_max_) {continue;}
      ++returns;
      v.points.push_back(p);
      const int rb = std::min(static_cast<int>(r / max_range_ * RANGE_BINS), RANGE_BINS - 1);
      const int hb = std::min(
        static_cast<int>((p.z() - z_min_) / (z_max_ - z_min_) * HEIGHT_BINS), HEIGHT_BINS - 1);
      v.descriptor[hb * RANGE_BINS + rb] += 1.0f;
      const int pb = azimuthBin(p.x(), p.y());
      v.profile[pb] = std::min(
        v.profile[pb], static_cast<float>(std::hypot(p.x(), p.y()) / max_range_));
    }
  }
  const float n = static_cast<float>(std::max<std::size_t>(returns, 1));
  for (auto & d : v.descriptor) {
    d /= n;
  }
  // The share of rays without a return in the band: open space, or nothing to see
  v.descriptor[RANGE_BINS * HEIGHT_BINS] =
    1.0f - static_cast<float>(returns) / static_cast<float>(na * ne);
  capPoints(v.points, MAX_VIEW_POINTS);
  return v;
}

View CloudViews3D::fromMap(const Eigen::Vector3d & position) const
{
  const int na = rays_.azimuths, ne = rays_.elevations;
  std::vector<double> ranges(static_cast<std::size_t>(na) * ne, max_range_);
  const Eigen::Vector3d origin = position + Eigen::Vector3d(0.0, 0.0, rays_.sensor_height);
  Eigen::Vector3d hit;
  for (int e = 0; e < ne; ++e) {
    const double el = ne == 1 ? 0.5 * (rays_.elevation_min + rays_.elevation_max) :
      rays_.elevation_min + (e + 0.5) * (rays_.elevation_max - rays_.elevation_min) / ne;
    for (int a = 0; a < na; ++a) {
      const double az = 2.0 * M_PI * (a + 0.5) / na - M_PI;
      const Eigen::Vector3d d(
        std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      if (field_.raycast(origin, d, max_range_, hit)) {
        ranges[static_cast<std::size_t>(e) * na + a] = (hit - origin).norm();
      }
    }
  }
  return fromRanges(ranges);
}

View CloudViews3D::fromScan(const std::vector<Eigen::Vector3d> & body_points) const
{
  // Per azimuth x elevation cell, the return nearest to the cell's centre ray (what the map's
  // ray at that centre would hit), seen from the sensor
  const int na = rays_.azimuths, ne = rays_.elevations;
  std::vector<double> ranges(static_cast<std::size_t>(na) * ne, max_range_);
  std::vector<double> off(ranges.size(), std::numeric_limits<double>::infinity());
  const Eigen::Vector3d origin(0.0, 0.0, rays_.sensor_height);
  const double daz = 2.0 * M_PI / na;
  const double del = (rays_.elevation_max - rays_.elevation_min) / ne;
  for (const auto & q : body_points) {
    const Eigen::Vector3d d = q - origin;
    const double r = d.norm();
    if (r <= 0.0) {continue;}
    const double el = std::asin(std::max(-1.0, std::min(1.0, d.z() / r)));
    if (el < rays_.elevation_min || el >= rays_.elevation_max) {continue;}
    const double az = std::atan2(d.y(), d.x());
    const int e = ne == 1 ? 0 :
      std::min(ne - 1, static_cast<int>((el - rays_.elevation_min) / del));
    const int a = std::min(na - 1, std::max(0, static_cast<int>((az + M_PI) / daz)));
    const double el_c = rays_.elevation_min + (e + 0.5) * del;
    const double az_c = (a + 0.5) * daz - M_PI;
    const double o = std::hypot(el - el_c, std::remainder(az - az_c, 2.0 * M_PI));
    const std::size_t k = static_cast<std::size_t>(e) * na + a;
    if (o < off[k]) {
      off[k] = o;
      ranges[k] = r;
    }
  }
  return fromRanges(ranges);
}

double coverage(
  const std::vector<Eigen::Vector3d> & view_points, const Eigen::Vector3d & place,
  const std::vector<Eigen::Vector3d> & body_points, const Pose & X, double inlier_distance,
  const CoverageBand & band)
{
  if (view_points.empty() || body_points.empty() || inlier_distance <= 0.0) {return 0.0;}
  // The scan's points in the map, hashed on a grid of the inlier distance
  const double cell = inlier_distance;
  auto key = [cell](const Eigen::Vector3d & p) {
      const int64_t i = static_cast<int64_t>(std::floor(p.x() / cell));
      const int64_t j = static_cast<int64_t>(std::floor(p.y() / cell));
      const int64_t k = static_cast<int64_t>(std::floor(p.z() / cell));
      return (i & 0x1FFFFF) | ((j & 0x1FFFFF) << 21) | ((k & 0x1FFFFF) << 42);
    };
  std::unordered_map<int64_t, std::vector<Eigen::Vector3d>> grid;
  for (const auto & q : body_points) {
    Eigen::Vector3d w = X * q;
    if (band.planar) {w.z() = 0.0;}   // a 2D view has no height, the scan has the sensor's
    grid[key(w)].push_back(w);
  }
  const double d2 = inlier_distance * inlier_distance;
  const double yaw = yawOf(X);
  const bool all_around = band.azimuth_fov >= 2.0 * M_PI - 1e-6;
  std::size_t covered = 0, judged = 0;
  for (const auto & v : view_points) {
    // Only what the sensor could see from X: its band of heights (a 2D view has none) and its
    // field of view about the heading
    if (!band.planar && (v.z() < band.z_min || v.z() > band.z_max)) {continue;}
    if (!all_around) {
      const double az = std::remainder(std::atan2(v.y(), v.x()) - yaw, 2.0 * M_PI);
      if (std::abs(az) > 0.5 * band.azimuth_fov) {continue;}
    }
    if (!band.planar) {
      const double el = std::atan2(v.z(), std::hypot(v.x(), v.y()));
      if (el < band.elevation_min || el > band.elevation_max) {continue;}
    }
    ++judged;
    Eigen::Vector3d w = place + v;
    if (band.planar) {w.z() = 0.0;}
    bool near = false;
    for (int di = -1; di <= 1 && !near; ++di) {
      for (int dj = -1; dj <= 1 && !near; ++dj) {
        for (int dk = -1; dk <= 1 && !near; ++dk) {
          const auto it = grid.find(key(w + cell * Eigen::Vector3d(di, dj, dk)));
          if (it == grid.end()) {continue;}
          for (const auto & s : it->second) {
            if ((s - w).squaredNorm() <= d2) {near = true; break;}
          }
        }
      }
    }
    if (near) {++covered;}
  }
  if (judged == 0) {return 0.0;}
  return static_cast<double>(covered) / static_cast<double>(judged);
}

// ---------------------------------------------------------------- SymmetryGraph

float SymmetryGraph::descriptorDistance(const std::vector<float> & a, const std::vector<float> & b)
{
  float d = 0.0f;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t k = 0; k < n; ++k) {
    d += std::abs(a[k] - b[k]);
  }
  return d;
}

std::vector<double> SymmetryGraph::yawSeeds(
  const std::vector<float> & a, const std::vector<float> & b, int n)
{
  // Shift s of a that matches b: b[k] ~ a[k + s], i.e. the body turned by 2 pi s / m
  const int m = static_cast<int>(std::min(a.size(), b.size()));
  std::vector<double> cost(m, 0.0);
  for (int s = 0; s < m; ++s) {
    double c = 0.0;
    for (int k = 0; k < m; ++k) {
      c += std::abs(a[(k + s) % m] - b[k]);
    }
    cost[s] = c;
  }
  // Local minima, best first, at least two bins apart
  std::vector<std::pair<double, int>> minima;
  for (int s = 0; s < m; ++s) {
    if (cost[s] <= cost[(s + 1) % m] && cost[s] <= cost[(s + m - 1) % m]) {
      minima.emplace_back(cost[s], s);
    }
  }
  std::sort(minima.begin(), minima.end());
  std::vector<double> out;
  for (const auto & [c, s] : minima) {
    const double yaw = 2.0 * M_PI * s / m;
    bool far = true;
    for (double y : out) {
      if (std::abs(wrapAngle(y - yaw)) < 2.0 * 2.0 * M_PI / m) {far = false; break;}
    }
    if (far) {out.push_back(wrapAngle(yaw));}
    if (static_cast<int>(out.size()) >= n) {break;}
  }
  return out;
}

int64_t SymmetryGraph::cellKey(const Eigen::Vector3d & p) const
{
  const int64_t i = static_cast<int64_t>(std::floor(p.x() / step_));
  const int64_t j = static_cast<int64_t>(std::floor(p.y() / step_));
  const int64_t k = static_cast<int64_t>(std::floor(p.z() / std::max(step_, 2.0)));
  return (i & 0x1FFFFF) | ((j & 0x1FFFFF) << 21) | ((k & 0x1FFFFF) << 42);
}

void SymmetryGraph::index()
{
  cells_.clear();
  for (std::size_t n = 0; n < nodes_.size(); ++n) {
    cells_[cellKey(nodes_[n].position)].push_back(static_cast<int>(n));
  }
}

const Node * SymmetryGraph::nearest(const Eigen::Vector3d & p) const
{
  const Node * best = nullptr;
  double best_d = 2.0 * step_;
  const double zs = std::max(step_, 2.0);
  for (int dk = -1; dk <= 1; ++dk) {
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        const Eigen::Vector3d q = p + Eigen::Vector3d(di * step_, dj * step_, dk * zs);
        const auto it = cells_.find(cellKey(q));
        if (it == cells_.end()) {continue;}
        for (int n : it->second) {
          const double d = (nodes_[n].position - p).norm();
          if (d < best_d) {best_d = d; best = &nodes_[n];}
        }
      }
    }
  }
  return best;
}

std::vector<int> SymmetryGraph::closestNodes(
  const std::vector<float> & descriptor, int n, int exclude) const
{
  std::vector<std::pair<float, int>> all;
  all.reserve(nodes_.size());
  for (std::size_t k = 0; k < nodes_.size(); ++k) {
    if (static_cast<int>(k) == exclude) {continue;}
    all.emplace_back(descriptorDistance(nodes_[k].descriptor, descriptor), static_cast<int>(k));
  }
  const std::size_t keep = std::min<std::size_t>(static_cast<std::size_t>(n), all.size());
  std::partial_sort(all.begin(), all.begin() + keep, all.end());
  std::vector<int> out;
  for (std::size_t k = 0; k < keep; ++k) {
    out.push_back(all[k].second);
  }
  return out;
}

void SymmetryGraph::build(
  const ViewMaker & views, const Matcher & matcher, const GraphOptions & options,
  const std::function<bool(double)> & progress)
{
  nodes_.clear();
  step_ = options.sample_step > 0.0 ? options.sample_step : 0.5;
  const auto positions = views.samplePositions();
  nodes_.resize(positions.size());
  std::vector<View> all_views(positions.size());
  std::atomic<std::size_t> next {0};
  std::atomic<bool> abort {false};
  const int threads = std::max(1, options.threads);

  // Views of every place
  auto view_worker = [&]() {
      for (std::size_t n = next++; n < positions.size() && !abort; n = next++) {
        all_views[n] = views.fromMap(positions[n]);
        nodes_[n].position = positions[n];
        nodes_[n].descriptor = all_views[n].descriptor;
        nodes_[n].profile = all_views[n].profile;
        nodes_[n].points = all_views[n].points;
      }
    };
  auto run = [&](const std::function<void()> & fn, double from, double to) {
      std::vector<std::thread> pool;
      for (int t = 0; t < threads; ++t) {
        pool.emplace_back(fn);
      }
      while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const std::size_t done = std::min(next.load(), positions.size());
        const double f = from + (to - from) * static_cast<double>(done) /
          static_cast<double>(std::max<std::size_t>(positions.size(), 1));
        if (progress && !progress(f)) {abort = true;}
        if (done >= positions.size() || abort) {break;}
      }
      for (auto & t : pool) {
        t.join();
      }
    };
  run(view_worker, 0.0, 0.2);
  if (abort) {nodes_.clear(); return;}
  index();

  // Alternatives: places whose view fits elsewhere
  next = 0;
  auto alt_worker = [&]() {
      for (std::size_t n = next++; n < nodes_.size() && !abort; n = next++) {
        const View & view = all_views[n];
        if (view.points.size() < 10) {continue;}
        const Pose X_n = fromXYZYaw(positions[n].x(), positions[n].y(), positions[n].z(), 0.0);
        const double self_score = matcher.score(view.points, X_n);
        if (self_score <= 0.0) {continue;}
        auto cands = closestNodes(view.descriptor, options.candidates, static_cast<int>(n));
        cands.push_back(static_cast<int>(n));  // the place itself, turned
        std::vector<Alternative> found;
        for (int j : cands) {
          const auto seeds = yawSeeds(nodes_[j].profile, view.profile, options.yaw_seeds + 1);
          for (double yaw : seeds) {
            const Pose X_start = fromXYZYaw(
              positions[j].x(), positions[j].y(), positions[j].z(), yaw);
            if (samePose(X_start, X_n, options.merge_distance, options.merge_angle)) {continue;}
            const auto r = matcher.match(view.points, X_start);
            if (r.score < options.min_score * self_score) {continue;}
            if (samePose(r.pose, X_n, options.merge_distance, options.merge_angle)) {continue;}
            bool dup = false;
            for (auto & a : found) {
              if (samePose(X_n * a.T_node_alt, r.pose, options.merge_distance,
                options.merge_angle))
              {
                dup = true;
                if (r.score > a.score) {a.score = r.score; a.T_node_alt = X_n.inverse() * r.pose;}
                break;
              }
            }
            if (dup) {continue;}
            Alternative a;
            a.T_node_alt = X_n.inverse() * r.pose;
            a.score = r.score;
            a.node = j;
            found.push_back(a);
          }
        }
        nodes_[n].alternatives = std::move(found);
      }
    };
  run(alt_worker, 0.2, 1.0);
  if (abort) {nodes_.clear(); cells_.clear(); return;}
  if (progress) {progress(1.0);}
}

std::vector<Candidate> SymmetryGraph::twins(const Pose & X) const
{
  std::vector<Candidate> out;
  const Node * n = nearest(X.translation());
  if (!n) {return out;}
  const Pose X_n = fromXYZYaw(n->position.x(), n->position.y(), n->position.z(), 0.0);
  for (const auto & a : n->alternatives) {
    Candidate c;
    c.pose = X_n * a.T_node_alt * X_n.inverse() * X;
    c.score = a.score;
    out.push_back(c);
  }
  return out;
}

int SymmetryGraph::descriptorRank(
  const std::vector<float> & descriptor, const Eigen::Vector3d & p, float * distance) const
{
  const Node * n = nearest(p);
  if (!n) {return -1;}
  const float d = descriptorDistance(descriptor, n->descriptor);
  if (distance) {*distance = d;}
  int rank = 0;
  for (const auto & m : nodes_) {
    if (descriptorDistance(descriptor, m.descriptor) < d) {++rank;}
  }
  return rank;
}

std::vector<Candidate> SymmetryGraph::globalMatch(
  const ViewMaker & views, const Matcher & matcher,
  const std::vector<Eigen::Vector3d> & body_points, int candidates, int yaw_seeds,
  double min_score, const std::vector<Eigen::Vector3d> * view_points,
  const CoverageBand & band) const
{
  std::vector<Candidate> out;
  if (nodes_.empty() || body_points.empty()) {return out;}
  const View view = views.fromScan(
    view_points && !view_points->empty() ? *view_points : body_points);
  const double inlier = matcher.options().inlier_distance;
  // The places whose view looks like the scan's, at the headings their profiles suggest,
  // refined by the matcher and scored where they land: the scan's fit and how much of the
  // place's view it explains
  for (int j : closestNodes(view.descriptor, candidates, -1)) {
    for (double yaw : yawSeeds(nodes_[j].profile, view.profile, yaw_seeds)) {
      const Pose X0 = fromXYZYaw(
        nodes_[j].position.x(), nodes_[j].position.y(), nodes_[j].position.z(), yaw);
      const auto r = matcher.match(body_points, X0);
      if (r.score < min_score) {continue;}
      const Node * n = nearest(r.pose.translation());
      const double cov = (n && n->points.size() >= 0.25 * views.rays()) ?   // few returns: no judge
        coverage(n->points, n->position, body_points, r.pose, inlier, band) : r.score;
      const double score = std::sqrt(r.score * cov);
      bool dup = false;
      for (auto & c : out) {
        if (samePose(c.pose, r.pose, std::max(step_, 0.5), 0.3)) {
          dup = true;
          if (score > c.score) {c = Candidate{r.pose, score};}
          break;
        }
      }
      if (!dup) {out.push_back(Candidate{r.pose, score});}
    }
  }
  std::sort(
    out.begin(), out.end(), [](const Candidate & a, const Candidate & b) {
      return a.score > b.score;
    });
  return out;
}

std::size_t SymmetryGraph::alternativesCount() const
{
  std::size_t n = 0;
  for (const auto & node : nodes_) {
    n += node.alternatives.size();
  }
  return n;
}

bool SymmetryGraph::save(const std::string & path, uint64_t fingerprint) const
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {return false;}
  auto put = [&f](const auto & v) {f.write(reinterpret_cast<const char *>(&v), sizeof(v));};
  const char magic[8] = {'S', 'Y', 'M', 'G', '0', '0', '0', '3'};
  f.write(magic, 8);
  put(fingerprint);
  put(step_);
  const uint64_t n = nodes_.size();
  put(n);
  for (const auto & node : nodes_) {
    for (int k = 0; k < 3; ++k) {
      put(node.position[k]);
    }
    const uint32_t nd = node.descriptor.size(), np = node.profile.size(),
      na = node.alternatives.size();
    put(nd);
    f.write(reinterpret_cast<const char *>(node.descriptor.data()), nd * sizeof(float));
    put(np);
    f.write(reinterpret_cast<const char *>(node.profile.data()), np * sizeof(float));
    const uint32_t npts = node.points.size();
    put(npts);
    for (const auto & p : node.points) {
      for (int k = 0; k < 3; ++k) {
        const float v = static_cast<float>(p[k]);
        put(v);
      }
    }
    put(na);
    for (const auto & a : node.alternatives) {
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
          put(a.T_node_alt.matrix()(r, c));
        }
      }
      put(a.score);
      const int32_t nn = a.node;
      put(nn);
    }
  }
  return static_cast<bool>(f);
}

bool SymmetryGraph::load(const std::string & path, uint64_t fingerprint)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return false;}
  auto get = [&f](auto & v) {f.read(reinterpret_cast<char *>(&v), sizeof(v));};
  char magic[8];
  f.read(magic, 8);
  if (!f || std::memcmp(magic, "SYMG0003", 8) != 0) {return false;}
  uint64_t stored = 0;
  get(stored);
  if (fingerprint != 0 && stored != fingerprint) {return false;}   // another map or options
  get(step_);
  uint64_t n = 0;
  get(n);
  if (!f || n > 50000000) {return false;}
  nodes_.assign(n, Node{});
  for (auto & node : nodes_) {
    for (int k = 0; k < 3; ++k) {
      get(node.position[k]);
    }
    uint32_t nd = 0, np = 0, na = 0;
    get(nd);
    node.descriptor.resize(nd);
    f.read(reinterpret_cast<char *>(node.descriptor.data()), nd * sizeof(float));
    get(np);
    node.profile.resize(np);
    f.read(reinterpret_cast<char *>(node.profile.data()), np * sizeof(float));
    uint32_t npts = 0;
    get(npts);
    if (npts > 100000) {nodes_.clear(); return false;}
    node.points.resize(npts);
    for (auto & p : node.points) {
      for (int k = 0; k < 3; ++k) {
        float v = 0.0f;
        get(v);
        p[k] = v;
      }
    }
    get(na);
    node.alternatives.resize(na);
    for (auto & a : node.alternatives) {
      Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
          get(m(r, c));
        }
      }
      a.T_node_alt.matrix() = m;
      get(a.score);
      int32_t nn = 0;
      get(nn);
      a.node = nn;
    }
    if (!f) {nodes_.clear(); return false;}
  }
  index();
  return true;
}

}  // namespace orbit
}  // namespace easynav
