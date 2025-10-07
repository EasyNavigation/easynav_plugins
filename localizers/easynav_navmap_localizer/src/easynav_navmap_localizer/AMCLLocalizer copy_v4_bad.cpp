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

/// \file
/// \brief Implementation of the AMCLLocalizer with probabilistic inflation + raycast check.

#include <expected>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iomanip>

#include "bonxai/bonxai.hpp"
#include "bonxai/probabilistic_map.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/types/IMUPerception.hpp"

#include "navmap_core/NavMap.hpp"

#include "easynav_navmap_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

namespace easynav
{
namespace navmap
{

static inline double sqr(double x) {return x * x;}

tf2::Vector3 computeMean(
  const std::vector<Particle> & particles,
  std::size_t start, std::size_t count)
{
  tf2::Vector3 weighted_sum(0.0, 0.0, 0.0);
  double total_weight = 0.0;

  if (start >= particles.size() || count == 0) {return weighted_sum;}
  std::size_t end = std::min(start + count, particles.size());

  for (std::size_t i = start; i < end; ++i) {
    const Particle & p = particles[i];
    const tf2::Vector3 & origin = p.pose.getOrigin();
    if (!std::isfinite(origin.x()) || !std::isfinite(origin.y()) || !std::isfinite(origin.z())) {
      continue;
    }
    weighted_sum += origin * p.weight;
    total_weight += p.weight;
  }
  return (total_weight > 0.0) ? (weighted_sum / total_weight) : tf2::Vector3(0.0, 0.0, 0.0);
}

tf2::Matrix3x3
computeCovariance(
  const std::vector<Particle> & particles,
  std::size_t start, std::size_t count,
  const tf2::Vector3 & mean)
{
  double total_weight = 0.0;
  double cov[3][3] = {{0.0}};

  if (start >= particles.size() || count == 0) {return tf2::Matrix3x3();}
  std::size_t end = std::min(start + count, particles.size());

  for (std::size_t i = start; i < end; ++i) {
    const Particle & p = particles[i];
    tf2::Vector3 d = p.pose.getOrigin() - mean;

    double w = p.weight;
    total_weight += w;

    cov[0][0] += w * d.x() * d.x();
    cov[0][1] += w * d.x() * d.y();
    cov[0][2] += w * d.x() * d.z();
    cov[1][1] += w * d.y() * d.y();
    cov[1][2] += w * d.y() * d.z();
    cov[2][2] += w * d.z() * d.z();
  }

  if (total_weight > 0.0) {
    cov[0][0] /= total_weight;
    cov[0][1] /= total_weight;
    cov[0][2] /= total_weight;
    cov[1][1] /= total_weight;
    cov[1][2] /= total_weight;
    cov[2][2] /= total_weight;
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];
  }

  return tf2::Matrix3x3(
    cov[0][0], cov[0][1], cov[0][2],
    cov[1][0], cov[1][1], cov[1][2],
    cov[2][0], cov[2][1], cov[2][2]);
}

double extractYaw(const tf2::Transform & transform)
{
  double r, p, y;
  tf2::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
  return y;
}

double computeYawVariance(
  const std::vector<Particle> & particles,
  std::size_t start, std::size_t count)
{
  double sum_cos = 0.0, sum_sin = 0.0, total_weight = 0.0;
  if (start >= particles.size() || count == 0) {return 0.0;}

  std::size_t end = std::min(start + count, particles.size());
  for (std::size_t i = start; i < end; ++i) {
    const Particle & p = particles[i];
    double y = extractYaw(p.pose);
    double w = p.weight;
    sum_cos += w * std::cos(y);
    sum_sin += w * std::sin(y);
    total_weight += w;
  }

  if (total_weight == 0.0) {return 0.0;}
  double mc = sum_cos / total_weight;
  double ms = sum_sin / total_weight;
  double R = std::sqrt(mc * mc + ms * ms);
  R = std::clamp((std::isfinite(R) ? R : 0.0), 1e-12, 1.0);
  double var = -2.0 * std::log(R);
  return (std::isfinite(var) && var >= 0.0) ? var : 0.0;
}

using std::placeholders::_1;
using namespace std::chrono_literals;

// ------------------- helpers -------------------
static inline tf2::Vector3 project_onto_plane(const tf2::Vector3 & v, const tf2::Vector3 & n_unit)
{
  return v - n_unit * v.dot(n_unit);
}

static inline tf2::Quaternion frame_from_normal_and_yaw(
  const tf2::Vector3 & n_world_unit, double yaw_world)
{
  tf2::Vector3 x_tgt(std::cos(yaw_world), std::sin(yaw_world), 0.0);
  tf2::Vector3 x_tan = project_onto_plane(x_tgt, n_world_unit);
  double xn = x_tan.length();
  if (xn < 1e-9) {
    tf2::Quaternion q; q.setRPY(0, 0, yaw_world);
    return q;
  }
  x_tan /= xn;

  tf2::Vector3 z_axis = n_world_unit;
  tf2::Vector3 y_axis = z_axis.cross(x_tan);
  double yn = y_axis.length();
  if (yn < 1e-9) {
    tf2::Quaternion q; q.setRPY(0, 0, yaw_world);
    return q;
  }
  y_axis /= yn;

  tf2::Matrix3x3 R(
    x_tan.x(), y_axis.x(), z_axis.x(),
    x_tan.y(), y_axis.y(), z_axis.y(),
    x_tan.z(), y_axis.z(), z_axis.z());
  tf2::Quaternion q; R.getRotation(q);
  return q;
}

static inline tf2::Vector3 to_tf(const Eigen::Vector3f & v)
{
  return {static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z())};
}

static inline bool is_finite_tf(const tf2::Transform & T)
{
  const auto & o = T.getOrigin();
  const auto & q = T.getRotation();
  return std::isfinite(o.x()) && std::isfinite(o.y()) && std::isfinite(o.z()) &&
         std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
         std::isfinite(q.w());
}

// ------------------- ctor/dtor -------------------
AMCLLocalizer::AMCLLocalizer()
{
  NavState::register_printer<nav_msgs::msg::Odometry>(
    [](const nav_msgs::msg::Odometry & odom) {
      std::ostringstream ret;
      double x = odom.pose.pose.position.x;
      double y = odom.pose.pose.position.y;

      tf2::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      ret << "Odometry with pose: (x: " << x << ", y: " << y << ", yaw: " << yaw << ")";
      return ret.str();
    });
}

AMCLLocalizer::~AMCLLocalizer() = default;

// ------------------- on_initialize -------------------
std::expected<void, std::string>
AMCLLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  int num_particles;
  double x_init, y_init, yaw_init, std_dev_xy, std_dev_yaw;

  node->declare_parameter<int>(plugin_name + ".num_particles", 100);
  node->declare_parameter<double>(plugin_name + ".initial_pose.x", 0.0);
  node->declare_parameter<double>(plugin_name + ".initial_pose.y", 0.0);
  node->declare_parameter<double>(plugin_name + ".initial_pose.yaw", 0.0);
  node->declare_parameter<double>(plugin_name + ".initial_pose.std_dev_xy", 0.5);
  node->declare_parameter<double>(plugin_name + ".initial_pose.std_dev_yaw", 0.5);
  node->declare_parameter<double>(plugin_name + ".reseed_freq", 1.0);
  node->declare_parameter<double>(plugin_name + ".noise_translation", 0.01);
  node->declare_parameter<double>(plugin_name + ".noise_rotation", 0.01);
  node->declare_parameter<double>(plugin_name + ".noise_translation_to_rotation", 0.01);
  node->declare_parameter<double>(plugin_name + ".min_noise_xy", 0.05);
  node->declare_parameter<double>(plugin_name + ".min_noise_yaw", 0.05);
  node->declare_parameter<bool>(plugin_name + ".compute_odom_from_tf", false);
  node->declare_parameter<double>(plugin_name + ".inflation_stddev", 0.05);
  node->declare_parameter<double>(plugin_name + ".inflation_prob_min", 0.01);
  node->declare_parameter<int>(plugin_name + ".correct_max_points", 1500);
  node->declare_parameter<double>(plugin_name + ".weights_tau", 0.7);
  node->declare_parameter<double>(plugin_name + ".top_keep_fraction", 0.2);

  node->get_parameter<int>(plugin_name + ".num_particles", num_particles);
  node->get_parameter<double>(plugin_name + ".initial_pose.x", x_init);
  node->get_parameter<double>(plugin_name + ".initial_pose.y", y_init);
  node->get_parameter<double>(plugin_name + ".initial_pose.yaw", yaw_init);
  node->get_parameter<double>(plugin_name + ".initial_pose.std_dev_xy", std_dev_xy);
  node->get_parameter<double>(plugin_name + ".initial_pose.std_dev_yaw", std_dev_yaw);
  node->get_parameter<double>(plugin_name + ".noise_translation", noise_translation_);
  node->get_parameter<double>(plugin_name + ".noise_rotation", noise_rotation_);
  node->get_parameter<double>(plugin_name + ".noise_translation_to_rotation",
    noise_translation_to_rotation_);
  node->get_parameter<double>(plugin_name + ".min_noise_xy", min_noise_xy_);
  node->get_parameter<double>(plugin_name + ".min_noise_yaw", min_noise_yaw_);
  node->get_parameter<bool>(plugin_name + ".compute_odom_from_tf", compute_odom_from_tf_);
  node->get_parameter<double>(plugin_name + ".inflation_stddev", inflation_stddev_);
  node->get_parameter<double>(plugin_name + ".inflation_prob_min", inflation_prob_min_);
  int tmp_max_pts = 1500;
  node->get_parameter<int>(plugin_name + ".correct_max_points", tmp_max_pts);
  correct_max_points_ = (tmp_max_pts >
    0) ? static_cast<std::size_t>(tmp_max_pts) : std::size_t{1500};
  node->get_parameter<double>(plugin_name + ".weights_tau", weights_tau_);
  node->get_parameter<double>(plugin_name + ".top_keep_fraction", top_keep_fraction_);

  if (!std::isfinite(inflation_stddev_) || inflation_stddev_ <= 0.0) {inflation_stddev_ = 1.5;}
  if (!std::isfinite(inflation_prob_min_) || inflation_prob_min_ <= 0.0) {
    inflation_prob_min_ = 0.01;
  }
  if (!std::isfinite(weights_tau_) || weights_tau_ <= 0.0) {weights_tau_ = 0.7;}
  if (!std::isfinite(top_keep_fraction_) || top_keep_fraction_ <= 0.0) {top_keep_fraction_ = 0.2;}
  if (top_keep_fraction_ > 1.0) {top_keep_fraction_ = 1.0;}

  double reseed_freq;
  node->get_parameter<double>(plugin_name + ".reseed_freq", reseed_freq);
  reseed_time_ = 1.0 / reseed_freq;

  RCLCPP_INFO(node->get_logger(), "Initialized AMCL with %d particles", num_particles);
  RCLCPP_INFO(node->get_logger(), "init pose (%.3f, %.3f, %.3f) std_dev [%.3f, %.3f]",
    x_init, y_init, yaw_init, std_dev_xy, std_dev_yaw);
  RCLCPP_INFO(node->get_logger(), "inflation_stddev=%.3f, inflation_prob_min=%.3f, "
                                   "correct_max_points=%zu, weights_tau=%.3f, top_keep_fraction=%.3f",
    inflation_stddev_, inflation_prob_min_, correct_max_points_, weights_tau_, top_keep_fraction_);

  std::normal_distribution<double> noise_x(x_init, std_dev_xy);
  std::normal_distribution<double> noise_y(y_init, std_dev_xy);
  std::normal_distribution<double> noise_yaw(yaw_init, std_dev_yaw);

  particles_.resize(num_particles);
  for (auto & p : particles_) {
    tf2::Quaternion q; q.setRPY(0.0, 0.0, noise_yaw(rng_));
    p.pose.setRotation(q);
    p.pose.setOrigin(tf2::Vector3(noise_x(rng_), noise_y(rng_), 0.0));
    p.hits = 0;
    p.possible_hits = 0;
    p.weight = 1.0 / num_particles;
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  auto node_typed = std::dynamic_pointer_cast<LocalizerNode>(get_node());
  auto rt_cbg = node_typed->get_real_time_cbg();
  rclcpp::SubscriptionOptions options; options.callback_group = rt_cbg;

  if (!compute_odom_from_tf_) {
    odom_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS().reliable(),
      std::bind(&AMCLLocalizer::odom_callback, this, _1), options);
  }

  const auto fq = node->get_fully_qualified_name() + std::string("/") + plugin_name;
  particles_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseArray>(fq + "/particles",
        10);
  estimate_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(fq +
        "/pose", 10);

  last_reseed_ = get_node()->now();
  get_node()->get_logger().set_level(rclcpp::Logger::Level::Debug);
  return {};
}

// ------------------- odom/tf -------------------
void
AMCLLocalizer::odom_callback(nav_msgs::msg::Odometry::UniquePtr msg)
{
  if (compute_odom_from_tf_) {return;}
  tf2::fromMsg(msg->pose.pose, odom_);
  if (!initialized_odom_) {last_odom_ = odom_; initialized_odom_ = true;}
}

void
AMCLLocalizer::update_odom_from_tf()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "AMCLLocalizer::update: TF failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_odom; tf2::fromMsg(tf_msg.transform, tf_odom);
  last_odom_ = odom_;
  odom_ = tf_odom;
  initialized_odom_ = true;
}

// ------------------- imu -------------------
std::optional<tf2::Quaternion>
get_latest_imu_quat(const NavState & nav_state)
{
  if (!nav_state.has("imu")) {return std::nullopt;}
  const auto imus = nav_state.get<IMUPerceptions>("imu");
  if (imus.empty() || !imus.back()) {return std::nullopt;}

  const auto & imu_msg = imus.back()->data;
  tf2::Quaternion q;
  q.setX(imu_msg.orientation.x);
  q.setY(imu_msg.orientation.y);
  q.setZ(imu_msg.orientation.z);
  q.setW(imu_msg.orientation.w);
  if (q.length2() < 1e-12) {return std::nullopt;}
  q.normalize();
  return q;
}

// ------------------- predict -------------------
void
AMCLLocalizer::predict(NavState & nav_state)
{
  if (!initialized_odom_) {
    if (compute_odom_from_tf_) {update_odom_from_tf();}
    return;
  }
  if (compute_odom_from_tf_) {update_odom_from_tf();}

  tf2::Transform delta = last_odom_.inverseTimes(odom_);

  const bool have_navmap = nav_state.has("map.navmap");
  if (have_navmap) {
    navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");
  }

  const auto imu_q_opt = get_latest_imu_quat(nav_state);

  tf2::Vector3 t = delta.getOrigin();
  double dx = t.x(), dy = t.y(), dz = t.z();
  double trans_len = std::sqrt(dx * dx + dy * dy + dz * dz);

  double r, p, y;
  tf2::Matrix3x3(delta.getRotation()).getRPY(r, p, y);
  double rot_len = std::abs(y);

  std::random_device rd; std::mt19937 gen(rd());

  for (auto & ptl : particles_) {
    std::normal_distribution<double> noise_dx(0.0, std::abs(dx) * noise_translation_);
    std::normal_distribution<double> noise_dy(0.0, std::abs(dy) * noise_translation_);
    std::normal_distribution<double> noise_dz(0.0, std::abs(dz) * noise_translation_);
    std::normal_distribution<double> noise_yaw(
      0.0, rot_len * noise_rotation_ + trans_len * noise_translation_to_rotation_);

    tf2::Vector3 noisy_translation(dx + noise_dx(gen), dy + noise_dy(gen), dz + noise_dz(gen));
    double noisy_yaw = y + noise_yaw(gen);
    tf2::Quaternion noisy_q; noisy_q.setRPY(0.0, 0.0, noisy_yaw);
    tf2::Transform noisy_delta(noisy_q, noisy_translation);

    ptl.pose = ptl.pose * noisy_delta;

    if (have_navmap) {
      ::navmap::NavMap::LocateOpts opts;
      if (ptl.last_cid != std::numeric_limits<uint32_t>::max()) {
        opts.hint_cid = ptl.last_cid;
        opts.hint_surface = ptl.last_surface;
      }
      opts.planar_eps = 1e-4f;
      opts.height_eps = 0.50f;
      opts.use_downward_ray = true;

      tf2::Vector3 Pw = ptl.pose.getOrigin();

      std::size_t sidx = 0;
      ::navmap::NavCelId cid = std::numeric_limits<uint32_t>::max();
      Eigen::Vector3f bary, hit_eig;

      const bool ok = navmap_.locate_navcel(
        Eigen::Vector3f(static_cast<float>(Pw.x()),
                        static_cast<float>(Pw.y()),
                        static_cast<float>(Pw.z() + 0.5f)),
        sidx, cid, bary, &hit_eig, opts);

      if (ok) {
        tf2::Vector3 hit = to_tf(hit_eig);
        ptl.pose.setOrigin(tf2::Vector3(Pw.x(), Pw.y(), hit.z()));

        if (imu_q_opt.has_value()) {
          ptl.pose.setRotation(*imu_q_opt);
        } else {
          const auto & cel = navmap_.navcels[cid];
          tf2::Vector3 n_world = to_tf(cel.normal);
          double nlen = n_world.length();
          if (nlen > 1e-9) {
            n_world /= nlen;
            double rr, pp, yy;
            tf2::Matrix3x3(ptl.pose.getRotation()).getRPY(rr, pp, yy);
            tf2::Quaternion q_align = frame_from_normal_and_yaw(n_world, yy);
            ptl.pose.setRotation(q_align);
          }
        }
        ptl.last_cid = cid;
        ptl.last_surface = sidx;
      } else {
        ptl.pose.setOrigin(tf2::Vector3(Pw.x(), Pw.y(), Pw.z()));
        if (imu_q_opt.has_value()) {
          ptl.pose.setRotation(*imu_q_opt);
        }
      }
    }
  }

  last_odom_ = odom_;
  pose_ = getEstimatedPose();
  tf2::Transform map2bf = pose_;
  tf2::Transform map2odom = map2bf * odom_.inverse();
  publishTF(map2odom);
  publishEstimatedPose(map2bf);
}

// ------------------- inflate map (brushfire) -------------------
using ProgressCallback = std::function<void(float)>;

std::shared_ptr<Bonxai::ProbabilisticMap>
inflate_map(
  const std::shared_ptr<Bonxai::ProbabilisticMap> & src,
  double sigma,
  double p_min,
  ProgressCallback progress = nullptr)
{
  using namespace Bonxai;

  if (!src || sigma <= 0.0) {return nullptr;}

  const auto & src_grid = src->grid();
  const double res = src_grid.voxelSize();

  auto dst = std::make_shared<ProbabilisticMap>(res);
  dst->setOptions(src->options());

  std::vector<CoordT> seeds;
  src->getOccupiedVoxels(seeds);
  if (seeds.empty()) {return dst;}

  const int R = static_cast<int>(std::ceil(3.0 * sigma / res));
  if (R <= 0) {
    auto acc = dst->grid().createAccessor();
    for (const auto & c : seeds) {
      auto * cell = acc.value(c, /*create=*/true);
      cell->probability_log = dst->options().clamp_max_log;
      cell->update_id = 0;
    }
    return dst;
  }

  const double inv_2sig2 = 1.0 / (2.0 * sigma * sigma);
  std::vector<int32_t> lut_logod(R + 1);
  for (int d = 0; d <= R; ++d) {
    const double dist_m = d * res;
    const double p = std::exp(-(dist_m * dist_m) * inv_2sig2);
    lut_logod[d] = (p >= p_min) ?
      ProbabilisticMap::logods(static_cast<float>(p)) :
      std::numeric_limits<int32_t>::min();
  }

  std::vector<std::vector<CoordT>> buckets(R + 1);
  buckets[0] = seeds;

  struct KeyHash
  {
    size_t operator()(const CoordT & c) const noexcept
    {
      return (size_t)c.x * 73856093u ^ (size_t)c.y * 19349663u ^ (size_t)c.z * 83492791u;
    }
  };
  struct KeyEq
  {
    bool operator()(const CoordT & a, const CoordT & b) const noexcept
    {
      return a.x == b.x && a.y == b.y && a.z == b.z;
    }
  };
  std::unordered_set<CoordT, KeyHash, KeyEq> visited;
  visited.reserve(seeds.size() * 4);
  for (const auto & c : seeds) {
    visited.insert(c);
  }

  auto acc = dst->grid().createAccessor();
  const auto clamp_min = dst->options().clamp_min_log;
  const auto clamp_max = dst->options().clamp_max_log;

  auto write_max = [&](const CoordT & c, int32_t v) {
      if (v == std::numeric_limits<int32_t>::min()) {return;}
      auto * cell = acc.value(c, /*create=*/true);
      int32_t proposed = std::max(cell->probability_log, v);
      if (proposed < clamp_min) {proposed = clamp_min;}
      if (proposed > clamp_max) {proposed = clamp_max;}
      cell->probability_log = proposed;
      cell->update_id = 0;
    };

  for (const auto & c : seeds) {
    write_max(c, lut_logod[0]);
  }

  static const CoordT N6[6] = {
    {1, 0, 0}, {-1, 0, 0},
    {0, 1, 0}, {0, -1, 0},
    {0, 0, 1}, {0, 0, -1}
  };

  std::cout << "Inflating probabilistic map: seeds=" << seeds.size()
            << " sigma=" << sigma << "m, R=" << R << ", res=" << res << "m\n";

  size_t expanded = 0;
  for (int d = 0; d < R; ++d) {
    auto & bucket = buckets[d];

    if (progress) {progress(static_cast<float>(d) / static_cast<float>(R));}

    if (R >= 10 && (d % std::max(1, R / 10) == 0)) {
      double pct = 100.0 * static_cast<double>(d) / static_cast<double>(R);
      std::cout << "  d=" << d << "/" << R << " (" << pct << "%)\n";
    }

    const int nd = d + 1;
    for (const CoordT & c0 : bucket) {
      for (const CoordT & dn : N6) {
        CoordT c{c0.x + dn.x, c0.y + dn.y, c0.z + dn.z};
        if (visited.find(c) != visited.end()) {continue;}
        visited.insert(c);
        buckets[nd].push_back(c);
        write_max(c, lut_logod[nd]);
      }
      ++expanded;
    }
  }

  if (progress) {progress(1.0f);}
  std::cout << "Inflation complete. Expanded " << expanded << " nodes within R=" << R << ".\n";
  return dst;
}

// ------------------- raycast DDA -------------------
namespace
{
struct CoordHash
{
  size_t operator()(const Bonxai::CoordT & c) const noexcept
  {
    return (size_t)c.x * 73856093u ^ (size_t)c.y * 19349663u ^ (size_t)c.z * 83492791u;
  }
};
struct CoordEq
{
  bool operator()(const Bonxai::CoordT & a, const Bonxai::CoordT & b) const noexcept
  {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
};

inline bool ray_occluded_dda(
  const Bonxai::ProbabilisticMap & occ_map,
  const Bonxai::VoxelGrid<Bonxai::ProbabilisticMap::CellT>::ConstAccessor & acc,
  const Bonxai::CoordT & a, const Bonxai::CoordT & b,
  int tail_skip_voxels = 1)
{
  using Bonxai::CoordT;
  using CellT = Bonxai::ProbabilisticMap::CellT;

  int x = a.x, y = a.y, z = a.z;
  const int xe = b.x, ye = b.y, ze = b.z;

  const int dx = std::abs(xe - x);
  const int dy = std::abs(ye - y);
  const int dz = std::abs(ze - z);

  const int sx = (xe >= x) ? 1 : -1;
  const int sy = (ye >= y) ? 1 : -1;
  const int sz = (ze >= z) ? 1 : -1;

  int nsteps = std::max({dx, dy, dz});
  if (nsteps <= 0) {return false;}

  int ex = (dx >= dy && dx >= dz) ? dx / 2 : (dy >= dz ? dy / 2 : dz / 2);
  int ey = ex, ez = ex;

  for (int step = 0; step < nsteps; ++step) {
    if (step < nsteps - std::max(0, tail_skip_voxels) - 1) {
      const CellT * cell = acc.value(CoordT{x, y, z});
      if (cell && cell->probability_log > occ_map.options().occupancy_threshold_log) {
        return true;
      }
    }

    if (dx >= dy && dx >= dz) {
      x += sx;
      ey += dy; ez += dz;
      if (ey >= dx) {y += sy; ey -= dx;}
      if (ez >= dx) {z += sz; ez -= dx;}
    } else if (dy >= dx && dy >= dz) {
      y += sy;
      ex += dx; ez += dz;
      if (ex >= dy) {x += sx; ex -= dy;}
      if (ez >= dy) {z += sz; ez -= dy;}
    } else {
      z += sz;
      ex += dx; ey += dy;
      if (ex >= dz) {x += sx; ex -= dz;}
      if (ey >= dz) {y += sy; ey -= dz;}
    }
  }
  return false;
}
} // unnamed namespace

// ------------------- update/publish -------------------
void
AMCLLocalizer::update_rt(NavState & nav_state)
{
  predict(nav_state);
  nav_state.set("robot_pose", get_pose());
}

void
AMCLLocalizer::update(NavState & nav_state)
{
  correct(nav_state);

  if ((get_node()->now() - last_reseed_).seconds() > reseed_time_) {
    reseed();
    last_reseed_ = get_node()->now();
  }

  nav_state.set("robot_pose", get_pose());
  publishParticles();
}

// ------------------- correct -------------------
void
AMCLLocalizer::correct(NavState & nav_state)
{
  auto t0 = get_node()->now();
  if (!nav_state.has("points")) {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no points perceptions");
    return;
  }
  const auto perceptions = nav_state.get<PointPerceptions>("points");

  if (!nav_state.has("map.bonxai")) {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no a bonxai map");
    return;
  }

  if (!nav_state.has("map.bonxai.inflated")) {
    auto start = get_node()->now();
    RCLCPP_INFO(get_node()->get_logger(), "Creating inflation map");
    const auto original_map = nav_state.get_ptr<Bonxai::ProbabilisticMap>("map.bonxai");

    auto logger = get_node()->get_logger();
    auto clock = get_node()->get_clock();

    auto progress_cb = [logger, clock](float f) {
        RCLCPP_INFO_THROTTLE(logger, *clock, 2000, "Inflating... %.1f%%", double(f) * 100.0);
      };

    std::shared_ptr<Bonxai::ProbabilisticMap> inflated_map =
      inflate_map(original_map, inflation_stddev_, inflation_prob_min_, progress_cb);

    if (!inflated_map) {
      RCLCPP_ERROR(get_node()->get_logger(), "Error creating bonxai inflated map");
      return;
    }
    nav_state.set("map.bonxai.inflated", inflated_map);

    auto end = get_node()->now();
    RCLCPP_INFO(get_node()->get_logger(), "Created in %.3f seconds", (end - start).seconds());
  }

  const auto original_map = nav_state.get_ptr<Bonxai::ProbabilisticMap>("map.bonxai");
  const auto map = nav_state.get_ptr<Bonxai::ProbabilisticMap>("map.bonxai.inflated");
  auto t1 = get_node()->now();

  const double vox = map->grid().voxelSize();
  const double ds_vox = std::max(0.5, 2.0 * vox);
  auto view = PointPerceptionsOpsView(perceptions)
    .downsample(ds_vox)
    .fuse(get_tf_prefix() + "base_footprint");
  auto filtered = view->as_points();

  if (filtered.size() > correct_max_points_) {
    pcl::PointCloud<pcl::PointXYZ> slim;
    slim.header = filtered.header;
    slim.is_dense = filtered.is_dense;
    slim.points.reserve(correct_max_points_);

    const size_t stride = std::max<std::size_t>(1, filtered.size() / correct_max_points_);
    for (size_t i = 0; i < filtered.size() && slim.points.size() < correct_max_points_;
      i += stride)
    {
      slim.points.push_back(filtered.points[i]);
    }
    slim.width = static_cast<uint32_t>(slim.points.size());
    slim.height = 1;
    filtered.swap(slim);
  }

  auto t2 = get_node()->now();
  if (filtered.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No points to correct");
    return;
  }
  auto t3 = get_node()->now();

  // Tail-skip derived from Gaussian cutoff: distance where prob == inflation_prob_min_
  const double res_vox = original_map->grid().voxelSize();
  const double d_tail = inflation_stddev_ * std::sqrt(-2.0 * std::log(std::max(1e-6,
        inflation_prob_min_)));
  const int    TAIL_SKIP = std::max(1, static_cast<int>(std::ceil(d_tail / res_vox)));

  const float P_RAY_MIN = static_cast<float>(inflation_prob_min_);

  auto acc_occ = original_map->grid().createConstAccessor();
  auto acc_inf = map->grid().createConstAccessor();

  for (auto & particle : particles_) {

    bool debug = particle == particles_.front();

    float hits = 0.0f, possible_hits = 0.0f;

    const tf2::Vector3 origin_w = particle.pose.getOrigin();
    const Bonxai::CoordT key_origin =
      original_map->grid().posToCoord({origin_w.x(), origin_w.y(), origin_w.z()});

    std::unordered_map<Bonxai::CoordT, int, CoordHash, CoordEq> end_counts;
    end_counts.reserve(filtered.size());

    for (const auto & pt : filtered) {
      const tf2::Vector3 pw = particle.pose * tf2::Vector3(pt.x, pt.y, pt.z);
      if (!std::isfinite(pw.x())) {continue;}
      const Bonxai::CoordT key_end = original_map->grid().posToCoord({pw.x(), pw.y(), pw.z()});
      ++end_counts[key_end];
    }

    for (const auto & kv : end_counts) {
      const Bonxai::CoordT & key_end = kv.first;
      const int mult = kv.second;
      possible_hits += static_cast<float>(mult);

      float p_end = 0.0f;
      if (const auto * cell_end = acc_inf.value(key_end)) {
        p_end = Bonxai::ProbabilisticMap::prob(cell_end->probability_log);
      }

      if (p_end < P_RAY_MIN) {
        hits += p_end * mult;
        continue;
      }

      const bool occluded = ray_occluded_dda(*original_map, acc_occ, key_origin, key_end,
            TAIL_SKIP);
      hits += (occluded ? 0.0f : p_end) * mult;
    }

    particle.hits += hits;
    particle.possible_hits += possible_hits;
  }

  auto t4 = get_node()->now();

  for (auto & particle : particles_) {
    if (particle.possible_hits > 0) {
      double mean_p = static_cast<double>(particle.hits) /
        static_cast<double>(particle.possible_hits);
      mean_p = std::clamp(mean_p, 1e-6, 1.0);
      particle.weight = std::pow(mean_p, weights_tau_);
    } else {
      particle.weight = 1e-6;
    }
  }

  double total_weight = 0.0;
  for (const auto & p : particles_) {
    total_weight += p.weight;
  }
  if (total_weight > 0.0) {
    for (auto & p : particles_) {
      p.weight /= total_weight;
    }
  }

  auto t5 = get_node()->now();

  std::cerr << "Prepare maps: " << std::fixed << std::setprecision(6) << (t1 - t0).seconds() <<
    "\n";
  std::cerr << "Prepare perception: " << std::fixed << std::setprecision(6) <<
    (t2 - t1).seconds() << "\n";
  std::cerr << "Core: " << std::fixed << std::setprecision(6) << (t4 - t3).seconds() << "\n";
  std::cerr << "Final: " << std::fixed << std::setprecision(6) << (t5 - t4).seconds() << "\n";
}

// ------------------- reseed -------------------
void
AMCLLocalizer::reseed()
{
  if (particles_.empty()) {return;}
  if (particles_[0].possible_hits == 0) {return;}

  const std::size_t N = particles_.size();
  auto top_count = [&](std::size_t n) -> std::size_t {
      const double f = std::clamp(top_keep_fraction_, 0.0, 1.0);
      std::size_t k = static_cast<std::size_t>(std::floor(static_cast<double>(n) * f));
      return std::max<std::size_t>(1, std::min<std::size_t>(k, n));
    };
  const std::size_t N_top = top_count(N);

  std::sort(particles_.begin(), particles_.end(),
    [](const Particle & a, const Particle & b) {return a.weight > b.weight;});

  std::cerr <<
    "=================================================================================\n";
  for (std::size_t i = 0; i < particles_.size(); i++) {
    std::cerr << "[" << i << "] " << particles_[i].hits << "/" << particles_[i].possible_hits
              << "     " << static_cast<int>(particles_[i].last_cid) << std::endl;
  }


  tf2::Vector3 mean = computeMean(particles_, 0, N_top);
  tf2::Matrix3x3 cov = computeCovariance(particles_, 0, N_top, mean);

  double a = cov[0][0], b = cov[0][1], c = cov[1][1];

  auto safe_cov_2x2 = [](double a_in, double b_in, double c_in) {
      const double eps = 1e-9;
      double a2 = std::isfinite(a_in) ? a_in : 0.0;
      double b2 = std::isfinite(b_in) ? b_in : 0.0;
      double c2 = std::isfinite(c_in) ? c_in : 0.0;
      if (a2 < eps) {a2 = eps;}
      if (c2 < eps) {c2 = eps;}
      double det = a2 * c2 - b2 * b2;
      if (det < 0.0) {
        double max_b = std::sqrt(a2 * c2) - eps;
        max_b = std::max(0.0, max_b);
        if (b2 > max_b) {b2 = max_b;}
        if (b2 < -max_b) {b2 = -max_b;}
      }
      return std::tuple<double, double, double>(a2, b2, c2);
    };

  auto [a2, b2, c2] = safe_cov_2x2(a, b, c);
  double l00 = std::sqrt(std::max(a2, 0.0));
  double l10 = (l00 > 0.0) ? (b2 / l00) : 0.0;
  double t = c2 - l10 * l10; if (t < 0.0) {t = 0.0;}
  double l11 = std::sqrt(t);

  double yaw_variance = computeYawVariance(particles_, 0, N_top);
  if (!std::isfinite(yaw_variance) || yaw_variance < 0.0) {yaw_variance = 0.0;}
  double yaw_stddev = std::sqrt(yaw_variance);
  if (!std::isfinite(yaw_stddev) || yaw_stddev < min_noise_yaw_) {yaw_stddev = min_noise_yaw_;}
  std::normal_distribution<double> yaw_noise(0.0, yaw_stddev);

  std::normal_distribution<double> n01(0.0, 1.0);
  std::normal_distribution<double> index_dist(0.0, std::max(1.0, 0.05 * static_cast<double>(N)));

  auto sample_xy_offset = [&]() -> std::pair<double, double> {
      double z0 = n01(rng_), z1 = n01(rng_);
      double dx = l00 * z0;
      double dy = l10 * z0 + l11 * z1;

      if (!std::isfinite(dx) || !std::isfinite(dy)) {
        std::normal_distribution<double> nIso(0.0, min_noise_xy_);
        return {nIso(rng_), nIso(rng_)};
      }

      double r = std::hypot(dx, dy);
      if (!(r >= min_noise_xy_)) {
        if (r < 1e-12) {
          std::normal_distribution<double> nIso(0.0, min_noise_xy_);
          dx = nIso(rng_); dy = nIso(rng_);
        } else {
          double s = (min_noise_xy_ / r);
          dx *= s; dy *= s;
        }
      }
      return {dx, dy};
    };

  for (std::size_t i = N_top; i < N; ++i) {
    int selected_idx;
    int guard = 0;
    do {
      double idx_sample = std::round(index_dist(rng_));
      selected_idx = static_cast<int>(idx_sample);
      if (selected_idx < 0) {selected_idx = -selected_idx;}
      if (selected_idx >= static_cast<int>(N_top)) {selected_idx = static_cast<int>(N_top) - 1;}
      if (selected_idx < 0) {selected_idx = 0;}
      if (++guard > 16) {selected_idx = 0; break;}
    } while (selected_idx < 0 || static_cast<std::size_t>(selected_idx) >= N_top);

    const auto & ref = particles_[static_cast<std::size_t>(selected_idx)];
    tf2::Vector3 origin = ref.pose.getOrigin();
    auto [dx_off, dy_off] = sample_xy_offset();

    double rr, pp, yy;
    tf2::Matrix3x3(ref.pose.getRotation()).getRPY(rr, pp, yy);
    double noisy_yaw = yy + yaw_noise(rng_);
    if (!std::isfinite(noisy_yaw)) {noisy_yaw = yy;}

    tf2::Quaternion q; q.setRPY(0.0, 0.0, noisy_yaw);
    if (q.length2() < 1e-20 || !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
      !std::isfinite(q.z()) || !std::isfinite(q.w()))
    {
      q.setRPY(0.0, 0.0, yy);
    }

    tf2::Vector3 new_origin(origin.x() + dx_off, origin.y() + dy_off, origin.z());
    if (!std::isfinite(new_origin.x()) || !std::isfinite(new_origin.y()) ||
      !std::isfinite(new_origin.z()))
    {
      new_origin = tf2::Vector3(origin.x(), origin.y(), origin.z());
    }

    particles_[i].pose.setOrigin(new_origin);
    particles_[i].pose.setRotation(q);
    particles_[i].weight = ref.weight;
  }

  for (auto & p : particles_) {
    p.hits = 0; p.possible_hits = 0;
  }

  double total_weight = 0.0;
  for (const auto & p : particles_) {
    if (std::isfinite(p.weight)) {
      total_weight += p.weight;
    }
  }

  if (total_weight > 0.0 && std::isfinite(total_weight)) {
    for (auto & p : particles_) {
      p.weight /= total_weight;
      if (!std::isfinite(p.weight) || p.weight < 0.0) {p.weight = 0.0;}
    }
  } else {
    double w = 1.0 / static_cast<double>(particles_.size());
    for (auto & p : particles_) {
      p.weight = w;
    }
  }
}

// ------------------- publish & pose -------------------
void
AMCLLocalizer::publishTF(const tf2::Transform & map2bf)
{
  if (!is_finite_tf(map2bf)) {
    RCLCPP_ERROR(get_node()->get_logger(), "publishTF: transform has NaN/inf; skipping");
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = get_node()->now();
  tf_msg.header.frame_id = get_tf_prefix() + "map";
  tf_msg.child_frame_id = get_tf_prefix() + "odom";
  tf_msg.transform = tf2::toMsg(map2bf);

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  tf_broadcaster_->sendTransform(tf_msg);
}

void
AMCLLocalizer::publishParticles()
{
  geometry_msgs::msg::PoseArray array_msg;
  array_msg.header.stamp = get_node()->now();
  array_msg.header.frame_id = get_tf_prefix() + "map";

  array_msg.poses.reserve(particles_.size());
  for (const auto & p : particles_) {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = p.pose.getOrigin().x();
    pose_msg.position.y = p.pose.getOrigin().y();
    pose_msg.position.z = p.pose.getOrigin().z();
    pose_msg.orientation = tf2::toMsg(p.pose.getRotation());
    array_msg.poses.push_back(pose_msg);
  }
  particles_pub_->publish(array_msg);
}

tf2::Transform
AMCLLocalizer::getEstimatedPose() const
{
  if (particles_.empty()) {return tf2::Transform::getIdentity();}

  const std::size_t N = particles_.size();
  auto top_count = [&](std::size_t n) -> std::size_t {
      const double f = std::clamp(top_keep_fraction_, 0.0, 1.0);
      std::size_t k = static_cast<std::size_t>(std::floor(static_cast<double>(n) * f));
      return std::max<std::size_t>(1, std::min<std::size_t>(k, n));
    };
  const std::size_t N_top = top_count(N);

  std::vector<Particle> sorted = particles_;
  std::sort(sorted.begin(), sorted.end(),
    [](const Particle & a, const Particle & b) {return a.weight > b.weight;});

  tf2::Vector3 mean = computeMean(sorted, 0, N_top);
  if (!std::isfinite(mean.x()) || !std::isfinite(mean.y()) || !std::isfinite(mean.z())) {
    mean = tf2::Vector3(0.0, 0.0, 0.0);
  }
  double r, p, y;
  tf2::Matrix3x3(sorted[0].pose.getRotation()).getRPY(r, p, y);
  tf2::Quaternion q; q.setRPY(0.0, 0.0, y);

  tf2::Transform est; est.setOrigin(mean); est.setRotation(q);
  return est;
}

void
AMCLLocalizer::publishEstimatedPose(const tf2::Transform & est_pose)
{
  if (particles_.empty()) {return;}

  const std::size_t N = particles_.size();
  auto top_count = [&](std::size_t n) -> std::size_t {
      const double f = std::clamp(top_keep_fraction_, 0.0, 1.0);
      std::size_t k = static_cast<std::size_t>(std::floor(static_cast<double>(n) * f));
      return std::max<std::size_t>(1, std::min<std::size_t>(k, n));
    };
  const std::size_t N_top = top_count(N);

  tf2::Vector3 mean = est_pose.getOrigin();
  tf2::Matrix3x3 cov = computeCovariance(particles_, 0, N_top, mean);
  double yaw_variance = computeYawVariance(particles_, 0, N_top);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = get_node()->now();
  msg.header.frame_id = get_tf_prefix() + "map";

  msg.pose.pose.position.x = mean.x();
  msg.pose.pose.position.y = mean.y();
  msg.pose.pose.position.z = mean.z();
  msg.pose.pose.orientation = tf2::toMsg(est_pose.getRotation());

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      msg.pose.covariance[6 * r + c] = cov[r][c];
    }
  }
  msg.pose.covariance[6 * 5 + 5] = yaw_variance;

  estimate_pub_->publish(msg);
}

nav_msgs::msg::Odometry
AMCLLocalizer::get_pose()
{
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = get_node()->now();
  odom_msg.header.frame_id = get_tf_prefix() + "map";
  odom_msg.child_frame_id = get_tf_prefix() + "base_footprint";

  pose_ = getEstimatedPose();
  tf2::Transform est_pose = pose_;

  odom_msg.pose.pose.position.x = est_pose.getOrigin().x();
  odom_msg.pose.pose.position.y = est_pose.getOrigin().y();
  odom_msg.pose.pose.position.z = est_pose.getOrigin().z();
  odom_msg.pose.pose.orientation = tf2::toMsg(est_pose.getRotation());

  if (!particles_.empty()) {
    const std::size_t N = particles_.size();
    auto top_count = [&](std::size_t n) -> std::size_t {
        const double f = std::clamp(top_keep_fraction_, 0.0, 1.0);
        std::size_t k = static_cast<std::size_t>(std::floor(static_cast<double>(n) * f));
        return std::max<std::size_t>(1, std::min<std::size_t>(k, n));
      };
    const std::size_t N_top = top_count(N);

    std::vector<Particle> sorted = particles_;
    std::sort(sorted.begin(), sorted.end(),
      [](const Particle & a, const Particle & b) {return a.weight > b.weight;});

    tf2::Vector3 mean = computeMean(sorted, 0, N_top);
    tf2::Matrix3x3 cov = computeCovariance(sorted, 0, N_top, mean);
    double yaw_var = computeYawVariance(sorted, 0, N_top);

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom_msg.pose.covariance[6 * r + c] = cov[r][c];
      }
    }
    odom_msg.pose.covariance[6 * 5 + 5] = yaw_var;
  }

  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  return odom_msg;
}

}  // namespace navmap
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::AMCLLocalizer, easynav::LocalizerMethodBase)
