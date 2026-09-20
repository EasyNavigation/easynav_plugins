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
/// \brief Implementation of ParticlesDistribution.

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "easynav_costmap_common/cost_values.hpp"

#include "easynav_mhamcl_localizer/ParticlesDistribution.hpp"

namespace easynav
{
namespace mhamcl
{

double
yaw_of(const tf2::Transform & tf)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

double
normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

namespace
{

tf2::Transform
planar_transform(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
}

// std::normal_distribution is undefined for a non-positive deviation.
double
safe_std(double std_dev)
{
  return std::max(std_dev, 1e-12);
}

}  // namespace

ParticlesDistribution::ParticlesDistribution(const ParticlesParams & params, unsigned int seed)
: params_(params), rng_(seed)
{
}

void
ParticlesDistribution::init(
  const tf2::Transform & pose, double l00, double l10, double l11, double std_yaw,
  double quality)
{
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  std::normal_distribution<double> yaw_noise(0.0, safe_std(std_yaw));

  const double x = pose.getOrigin().x();
  const double y = pose.getOrigin().y();
  const double yaw = yaw_of(pose);

  const std::size_t n = static_cast<std::size_t>(
    std::max(1, (params_.max_particles + params_.min_particles) / 2));

  particles_.assign(n, Particle{});
  for (auto & p : particles_) {
    const double z0 = standard_normal(rng_);
    const double z1 = standard_normal(rng_);
    p.pose = planar_transform(x + l00 * z0, y + l10 * z0 + l11 * z1, yaw + yaw_noise(rng_));
    p.weight = 1.0 / static_cast<double>(n);
    p.hits = 0.0f;
  }

  quality_ = quality;
}

void
ParticlesDistribution::init(
  const tf2::Transform & pose, double std_xy, double std_yaw, double quality)
{
  init(pose, std_xy, 0.0, std_xy, std_yaw, quality);
}

void
ParticlesDistribution::predict(const tf2::Transform & delta)
{
  const tf2::Vector3 & t = delta.getOrigin();
  const double dx = t.x();
  const double dy = t.y();
  const double trans_len = std::sqrt(dx * dx + dy * dy);
  const double dyaw = yaw_of(delta);
  const double rot_len = std::abs(dyaw);

  std::normal_distribution<double> noise_dx(0.0,
    safe_std(std::abs(dx) * params_.noise_translation));
  std::normal_distribution<double> noise_dy(0.0,
    safe_std(std::abs(dy) * params_.noise_translation));
  std::normal_distribution<double> noise_yaw(
    0.0,
    safe_std(rot_len * params_.noise_rotation + trans_len * params_.noise_translation_to_rotation));

  for (auto & p : particles_) {
    const tf2::Transform noisy_delta = planar_transform(
      dx + noise_dx(rng_), dy + noise_dy(rng_), dyaw + noise_yaw(rng_));
    p.pose = p.pose * noisy_delta;
  }
}

double
ParticlesDistribution::error_to_obstacle(
  const easynav::Costmap2D & costmap, double x, double y, double dx, double dy) const
{
  auto is_obstacle = [&costmap](double wx, double wy) {
      unsigned int mx, my;
      return costmap.worldToMap(wx, wy, mx, my) &&
             costmap.getCost(mx, my) == easynav::LETHAL_OBSTACLE;
    };

  if (is_obstacle(x, y)) {return 0.0;}

  // Obstacles farther than 3 sigma from the expected position have a negligible likelihood,
  // so only the cells along the beam within that distance are queried.
  const double resolution = costmap.getResolution();
  const double max_dist = 3.0 * params_.distance_perception_error;

  for (double dist = resolution; dist < max_dist; dist += resolution) {
    if (is_obstacle(x + dx * dist, y + dy * dist) || is_obstacle(x - dx * dist, y - dy * dist)) {
      return dist;
    }
  }

  return std::numeric_limits<double>::infinity();
}

void
ParticlesDistribution::correct(
  const std::vector<ScanPoint> & points, const easynav::Costmap2D & costmap)
{
  if (points.empty() || particles_.empty()) {return;}

  static constexpr double inv_sqrt_2pi = 0.3989422804014327;
  const double sigma = params_.distance_perception_error;
  const double normal_comp_1 = inv_sqrt_2pi / sigma;

  for (auto & p : particles_) {
    p.hits = 0.0f;
  }

  for (auto & p : particles_) {
    const tf2::Matrix3x3 & rot = p.pose.getBasis();

    for (const auto & sp : points) {
      const tf2::Vector3 world = p.pose * sp.point;
      const tf2::Vector3 dir = rot * sp.direction;

      const double error = error_to_obstacle(costmap, world.x(), world.y(), dir.x(), dir.y());
      if (std::isinf(error)) {continue;}

      const double a = error / sigma;
      const double prob = std::clamp(normal_comp_1 * std::exp(-0.5 * a * a), 0.0, 1.0);

      p.weight = std::max(p.weight + prob, 1e-6);
      p.hits += static_cast<float>(prob);
    }
  }

  normalize();

  quality_ = 0.0;
  for (auto & p : particles_) {
    p.hits /= static_cast<float>(points.size());
    quality_ = std::max(quality_, static_cast<double>(p.hits));
  }
}

void
ParticlesDistribution::reseed()
{
  if (particles_.empty()) {return;}

  std::sort(
    particles_.begin(), particles_.end(),
    [](const Particle & a, const Particle & b) {return a.weight > b.weight;});

  // Adapt the number of particles: more when the hypothesis explains the perception badly,
  // fewer when it converged.
  std::size_t n = particles_.size();
  if (quality_ < params_.low_quality_threshold) {
    n = static_cast<std::size_t>(std::clamp(
        static_cast<int>(n) + params_.particles_step, params_.min_particles,
        params_.max_particles));
  } else if (quality_ > params_.good_quality_threshold) {
    n = static_cast<std::size_t>(std::clamp(
        static_cast<int>(n) - params_.particles_step, params_.min_particles,
        params_.max_particles));
  }
  n = std::max<std::size_t>(n, 1);

  if (n > particles_.size()) {
    particles_.resize(n, particles_.front());
  } else {
    particles_.resize(n);
  }

  const std::size_t num_losers = std::min(
    n - 1, static_cast<std::size_t>(static_cast<double>(n) * params_.reseed_percentage_losers));
  const std::size_t num_survivors = n - num_losers;
  const std::size_t num_winners = std::clamp<std::size_t>(
    static_cast<std::size_t>(static_cast<double>(n) * params_.reseed_percentage_winners),
    1, num_survivors);

  // Parents are selected among the winners following N(0, winners / 2)
  std::normal_distribution<double> selector(0.0, safe_std(static_cast<double>(num_winners) / 2.0));
  std::normal_distribution<double> noise_xy(0.0, safe_std(params_.reseed_noise_xy));
  std::normal_distribution<double> noise_yaw(0.0, safe_std(params_.reseed_noise_yaw));

  const double survivor_weight = particles_[num_survivors - 1].weight;

  for (std::size_t i = num_survivors; i < n; ++i) {
    const std::size_t parent_idx = std::min(
      static_cast<std::size_t>(std::abs(selector(rng_))), num_winners - 1);
    const tf2::Transform & parent = particles_[parent_idx].pose;

    Particle & child = particles_[i];
    child.pose = planar_transform(
      parent.getOrigin().x() + noise_xy(rng_),
      parent.getOrigin().y() + noise_xy(rng_),
      normalize_angle(yaw_of(parent) + noise_yaw(rng_)));
    child.weight = survivor_weight;
    child.hits = 0.0f;
  }

  normalize();
}

void
ParticlesDistribution::merge(const ParticlesDistribution & other)
{
  const std::size_t size = particles_.size();
  particles_.insert(particles_.end(), other.particles_.begin(), other.particles_.end());

  std::sort(
    particles_.begin(), particles_.end(),
    [](const Particle & a, const Particle & b) {return a.weight > b.weight;});
  particles_.resize(size);

  quality_ = std::max(quality_, other.quality_);
  normalize();
}

void
ParticlesDistribution::normalize()
{
  const double sum = std::accumulate(
    particles_.begin(), particles_.end(), 0.0,
    [](double acc, const Particle & p) {return acc + p.weight;});

  if (sum > 0.0) {
    for (auto & p : particles_) {
      p.weight /= sum;
    }
  } else if (!particles_.empty()) {
    for (auto & p : particles_) {
      p.weight = 1.0 / static_cast<double>(particles_.size());
    }
  }
}

tf2::Transform
ParticlesDistribution::get_pose() const
{
  if (particles_.empty()) {return tf2::Transform::getIdentity();}

  double sum_w = 0.0, x = 0.0, y = 0.0, cos_yaw = 0.0, sin_yaw = 0.0;
  for (const auto & p : particles_) {
    const double yaw = yaw_of(p.pose);
    sum_w += p.weight;
    x += p.weight * p.pose.getOrigin().x();
    y += p.weight * p.pose.getOrigin().y();
    cos_yaw += p.weight * std::cos(yaw);
    sin_yaw += p.weight * std::sin(yaw);
  }

  if (sum_w <= 0.0) {return particles_.front().pose;}

  return planar_transform(x / sum_w, y / sum_w, std::atan2(sin_yaw, cos_yaw));
}

std::array<double, 36>
ParticlesDistribution::get_covariance() const
{
  std::array<double, 36> cov;
  cov.fill(0.0);
  if (particles_.empty()) {return cov;}

  const tf2::Transform mean = get_pose();
  const double mx = mean.getOrigin().x();
  const double my = mean.getOrigin().y();
  const double myaw = yaw_of(mean);

  double sum_w = 0.0, cxx = 0.0, cxy = 0.0, cyy = 0.0, cxt = 0.0, cyt = 0.0, ctt = 0.0;
  for (const auto & p : particles_) {
    const double dx = p.pose.getOrigin().x() - mx;
    const double dy = p.pose.getOrigin().y() - my;
    const double dt = normalize_angle(yaw_of(p.pose) - myaw);

    sum_w += p.weight;
    cxx += p.weight * dx * dx;
    cxy += p.weight * dx * dy;
    cyy += p.weight * dy * dy;
    cxt += p.weight * dx * dt;
    cyt += p.weight * dy * dt;
    ctt += p.weight * dt * dt;
  }

  if (sum_w <= 0.0) {return cov;}

  cov[0] = cxx / sum_w;
  cov[1] = cov[6] = cxy / sum_w;
  cov[7] = cyy / sum_w;
  cov[5] = cov[30] = cxt / sum_w;
  cov[11] = cov[31] = cyt / sum_w;
  cov[35] = ctt / sum_w;

  return cov;
}

}  // namespace mhamcl
}  // namespace easynav
