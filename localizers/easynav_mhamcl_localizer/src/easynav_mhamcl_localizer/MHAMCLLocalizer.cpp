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
/// \brief Implementation of the MHAMCLLocalizer class using Costmap2D.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"

#include "easynav_mhamcl_localizer/MHAMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

namespace easynav
{
namespace mhamcl
{

using std::placeholders::_1;
using namespace std::chrono_literals;

MHAMCLLocalizer::MHAMCLLocalizer()
: rng_(std::random_device{}())
{
  NavState::register_printer<nav_msgs::msg::Odometry>(
    [](const nav_msgs::msg::Odometry & odom) {
      std::ostringstream ret;
      const double x = odom.pose.pose.position.x;
      const double y = odom.pose.pose.position.y;

      tf2::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      ret << "{" << rclcpp::Time(odom.header.stamp).seconds() << " } Odometry with pose: (x: " <<
        x << ", y: " << y << ", yaw: " << yaw << ")";
      return ret.str();
    });
}

MHAMCLLocalizer::~MHAMCLLocalizer()
{
  // The background matching only reads immutable data, but it must not outlive the plugin.
  if (matching_.valid()) {
    matching_.wait();
  }
}

void
MHAMCLLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  double x_init = 0.0;
  double y_init = 0.0;
  double yaw_init = 0.0;
  double std_dev_xy = 0.5;
  double std_dev_yaw = 0.5;
  double reseed_freq = 1.0 / 3.0;
  double hypotheses_freq = 1.0 / 3.0;
  int correct_max_points = static_cast<int>(correct_max_points_);

  auto & pp = particles_params_;
  auto & hp = hypotheses_params_;

  auto param = [&](const std::string & name, auto & value) {
      using T = std::decay_t<decltype(value)>;
      node->declare_parameter<T>(plugin_name + "." + name, value);
      node->get_parameter<T>(plugin_name + "." + name, value);
    };

  // Initial pose
  param("initial_pose.x", x_init);
  param("initial_pose.y", y_init);
  param("initial_pose.yaw", yaw_init);
  param("initial_pose.std_dev_xy", std_dev_xy);
  param("initial_pose.std_dev_yaw", std_dev_yaw);

  // Particle filter of every hypothesis
  param("max_particles", pp.max_particles);
  param("min_particles", pp.min_particles);
  param("particles_step", pp.particles_step);
  param("reseed_freq", reseed_freq);
  param("reseed_percentage_losers", pp.reseed_percentage_losers);
  param("reseed_percentage_winners", pp.reseed_percentage_winners);
  param("reseed_noise_xy", pp.reseed_noise_xy);
  param("reseed_noise_yaw", pp.reseed_noise_yaw);
  param("distance_perception_error", pp.distance_perception_error);
  param("correct_max_points", correct_max_points);

  // Motion model
  param("noise_translation", pp.noise_translation);
  param("noise_rotation", pp.noise_rotation);
  param("noise_translation_to_rotation", pp.noise_translation_to_rotation);
  param("min_noise_xy", min_noise_xy_);
  param("min_noise_yaw", min_noise_yaw_);
  param("odom_key", odom_key_);

  // Hypotheses
  param("multihypothesis", hp.multihypothesis);
  param("max_hypotheses", hp.max_hypotheses);
  param("hypotheses_freq", hypotheses_freq);
  param("min_candidate_weight", hp.min_candidate_weight);
  param("min_candidate_distance", hp.min_candidate_distance);
  param("min_candidate_angle", hp.min_candidate_angle);
  param("low_q_hypo_threshold", hp.low_q_hypo_threshold);
  param("very_low_q_hypo_threshold", hp.very_low_q_hypo_threshold);
  param("hypo_merge_distance", hp.hypo_merge_distance);
  param("hypo_merge_angle", hp.hypo_merge_angle);
  param("good_hypo_threshold", hp.good_hypo_threshold);
  param("min_hypo_diff_winner", hp.min_hypo_diff_winner);
  param("new_hypothesis_std_xy", hp.new_hypothesis_std_xy);
  param("new_hypothesis_std_yaw", hp.new_hypothesis_std_yaw);
  param("matcher_levels", hp.matcher_levels);
  param("matcher_angle_step", hp.matcher_angle_step);

  // Non-positive values are undefined behavior in std::normal_distribution or make no sense
  if (
    std_dev_xy <= 0.0 || std_dev_yaw <= 0.0 ||
    pp.noise_translation <= 0.0 || pp.noise_rotation <= 0.0 ||
    pp.noise_translation_to_rotation <= 0.0 ||
    min_noise_xy_ <= 0.0 || min_noise_yaw_ <= 0.0 ||
    pp.distance_perception_error <= 0.0 ||
    pp.reseed_noise_xy <= 0.0 || pp.reseed_noise_yaw <= 0.0 ||
    hp.new_hypothesis_std_xy <= 0.0 || hp.new_hypothesis_std_yaw <= 0.0)
  {
    throw std::runtime_error("MHAMCLLocalizer: standard deviations must be positive");
  }

  if (pp.min_particles < 1 || pp.max_particles < pp.min_particles) {
    throw std::runtime_error("MHAMCLLocalizer: invalid range of particles");
  }

  if (reseed_freq <= 0.0 || hypotheses_freq <= 0.0) {
    throw std::runtime_error("MHAMCLLocalizer: frequencies must be positive");
  }

  if (hp.max_hypotheses < 1 || hp.matcher_levels < 1 || hp.matcher_angle_step <= 0.0 ||
    correct_max_points < 1)
  {
    throw std::runtime_error("MHAMCLLocalizer: invalid hypotheses parameters");
  }

  pp.good_quality_threshold = hp.good_hypo_threshold;
  pp.low_quality_threshold = hp.low_q_hypo_threshold;
  correct_max_points_ = static_cast<std::size_t>(correct_max_points);
  reseed_time_ = 1.0 / reseed_freq;
  hypotheses_time_ = 1.0 / hypotheses_freq;

  RCLCPP_INFO(
    node->get_logger(),
    "Initialized MH-AMCL at position (%lf, %lf, %lf) std_dev [%lf, %lf], up to %d hypotheses",
    x_init, y_init, yaw_init, std_dev_xy, std_dev_yaw, hp.max_hypotheses);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_init);
  current_ = std::make_shared<ParticlesDistribution>(pp, rng_());
  current_->init(
    tf2::Transform(q, tf2::Vector3(x_init, y_init, 0.0)), std_dev_xy, std_dev_yaw, 0.5);
  hypotheses_.push_back(current_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  const std::string prefix = node->get_fully_qualified_name() + std::string("/") + plugin_name;
  particles_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseArray>(
    prefix + "/particles", 10);
  hypotheses_pub_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
    prefix + "/hypotheses", 10);
  estimate_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    prefix + "/pose", 10);

  init_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10, std::bind(&MHAMCLLocalizer::init_pose_callback, this, _1));

  last_reseed_ = get_node()->now();
  last_hypotheses_ = get_node()->now();
  last_input_time_ = get_node()->now();
}

void
MHAMCLLocalizer::update_rt(NavState & nav_state)
{
  predict(nav_state);

  std::lock_guard<std::mutex> lock(mutex_);
  nav_state.set("robot_pose", pose_msg_locked());
}

void
MHAMCLLocalizer::update(NavState & nav_state)
{
  correct(nav_state);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if ((get_node()->now() - last_reseed_).seconds() > reseed_time_) {
      for (auto & hypothesis : hypotheses_) {
        hypothesis->reseed();
      }
      last_reseed_ = get_node()->now();
    }
  }

  if (hypotheses_params_.multihypothesis && nav_state.has("map.base")) {
    run_matching(nav_state.get<Costmap2D>("map.base"));
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishParticles();
  }
}

void
MHAMCLLocalizer::init_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg)
{
  auto logger = get_node()->get_logger();
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  const std::string expected_frame = tf_info.map_frame;
  if (!msg->header.frame_id.empty() && msg->header.frame_id != expected_frame) {
    RCLCPP_WARN(
      logger,
      "MHAMCLLocalizer::init_pose_callback: received initial pose in frame '%s' but expected "
      "'%s'. Ignoring message.",
      msg->header.frame_id.c_str(), expected_frame.c_str());
    return;
  }

  const auto & pose = msg->pose.pose;
  tf2::Transform mean_pose;
  tf2::fromMsg(pose, mean_pose);
  const double mean_yaw = yaw_of(mean_pose);

  // Covariance in ROS order: x, y, z, roll, pitch, yaw
  const auto & cov = msg->pose.covariance;
  const double var_x = std::max(cov[0], 0.0);
  const double cov_xy = cov[1];
  const double var_y = std::max(cov[7], 0.0);
  const double var_yaw = std::max(cov[35], 0.0);

  // 2D Cholesky decomposition of the xy covariance
  double l00 = std::sqrt(var_x);
  double l10 = 0.0;
  double l11 = std::sqrt(var_y);
  if (l00 > 0.0) {
    l10 = cov_xy / l00;
    l11 = std::sqrt(std::max(var_y - l10 * l10, 0.0));
  }

  // Enforce a minimum noise if the covariance is too small
  const double std_xy = std::sqrt(0.5 * (var_x + var_y));
  if (std_xy < min_noise_xy_) {
    const double scale = min_noise_xy_ / ((std_xy > 1e-6) ? std_xy : 1.0);
    l00 *= scale;
    l10 *= scale;
    l11 *= scale;
    if (std_xy <= 1e-6) {
      // Degenerate covariance: isotropic noise
      l00 = l11 = min_noise_xy_;
      l10 = 0.0;
    }
  }
  const double std_yaw = std::max(std::sqrt(var_yaw), min_noise_yaw_);

  std::lock_guard<std::mutex> lock(mutex_);

  // A new pose given by the user discards everything the localizer believed
  auto hypothesis = std::make_shared<ParticlesDistribution>(particles_params_, rng_());
  hypothesis->init(mean_pose, l00, l10, l11, std_yaw, 1.0);
  hypotheses_.clear();
  hypotheses_.push_back(hypothesis);
  current_ = hypothesis;

  // Results of a matching launched before are not valid anymore
  matching_ = std::future<std::vector<TransformWeighted>>();

  last_reseed_ = get_node()->now();
  last_hypotheses_ = get_node()->now();

  if (initialized_odom_) {
    publishTF(estimated_pose_locked() * odom_.inverse());
  }
  publishEstimatedPose();
  publishParticles();

  RCLCPP_INFO(
    logger, "MHAMCLLocalizer::init_pose_callback: reinitialized around (%.3f, %.3f, %.3f)",
    mean_pose.getOrigin().x(), mean_pose.getOrigin().y(), mean_yaw);
}

bool
MHAMCLLocalizer::update_odom(NavState & nav_state)
{
  if (nav_state.has(odom_key_)) {
    const auto & msg = nav_state.get<nav_msgs::msg::Odometry>(odom_key_);

    // The handler stores an empty message until the first one arrives
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
      last_odom_ = odom_;
      tf2::fromMsg(msg.pose.pose, odom_);
      last_input_time_ = msg.header.stamp;

      if (!initialized_odom_) {
        last_odom_ = odom_;
        initialized_odom_ = true;
      }
      return true;
    }
  }

  return update_odom_from_tf();
}

bool
MHAMCLLocalizer::update_odom_from_tf()
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      tf_info.odom_frame, tf_info.robot_footprint_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "MHAMCLLocalizer::update: TF failed: %s", ex.what());
    return false;
  }

  last_odom_ = odom_;
  tf2::fromMsg(tf_msg.transform, odom_);
  last_input_time_ = tf_msg.header.stamp;

  if (!initialized_odom_) {
    last_odom_ = odom_;
    initialized_odom_ = true;
  }
  return true;
}

void
MHAMCLLocalizer::predict(NavState & nav_state)
{
  if (!update_odom(nav_state)) {return;}

  std::lock_guard<std::mutex> lock(mutex_);

  // Displacement of the robot in its own frame since the last prediction
  const tf2::Transform delta = last_odom_.inverseTimes(odom_);
  for (auto & hypothesis : hypotheses_) {
    hypothesis->predict(delta);
  }
  last_odom_ = odom_;

  const tf2::Transform map2bf = estimated_pose_locked();
  publishTF(map2bf * odom_.inverse());
  publishEstimatedPose();
}

void
MHAMCLLocalizer::correct(NavState & nav_state)
{
  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  if (perceptions.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "There are no points perceptions");
    return;
  }

  if (!nav_state.has("map.base")) {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no a map.base map");
    return;
  }

  const auto & map = nav_state.get<Costmap2D>("map.base");
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  auto view = PointPerceptionsOpsView(perceptions);
  view.downsample(map.getResolution())
  .fuse(tf_info.robot_footprint_frame, last_input_time_)
  .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
  .collapse({NAN, NAN, 0.1})
  .downsample(map.getResolution());
  const auto & filtered = view.as_points();

  if (filtered.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No points to correct");
    return;
  }

  // Limit the cost of the correction. Every point is seen along its ray from the robot.
  const std::size_t stride = std::max<std::size_t>(1, filtered.size() / correct_max_points_);
  std::vector<ScanPoint> scan_points;
  std::vector<tf2::Vector3> points;
  scan_points.reserve(filtered.size() / stride + 1);
  points.reserve(filtered.size() / stride + 1);
  for (std::size_t i = 0; i < filtered.size(); i += stride) {
    const tf2::Vector3 p(filtered[i].x, filtered[i].y, filtered[i].z);
    const double norm = std::hypot(p.x(), p.y());
    if (norm < 1e-6) {continue;}

    scan_points.push_back({p, tf2::Vector3(p.x() / norm, p.y() / norm, 0.0)});
    points.push_back(p);
  }

  if (scan_points.empty()) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & hypothesis : hypotheses_) {
    hypothesis->correct(scan_points, map);
  }
  last_points_ = std::move(points);
}

void
MHAMCLLocalizer::run_matching(const easynav::Costmap2D & map)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto & hp = hypotheses_params_;
  std::vector<TransformWeighted> candidates;
  bool harvested = false;

  if (matching_.valid() && matching_.wait_for(0s) == std::future_status::ready) {
    candidates = matching_.get();
    harvested = true;

    // The robot moved while the matching was running: bring the candidates to the present
    const tf2::Transform moved = matching_odom_.inverse() * odom_;
    for (auto & candidate : candidates) {
      candidate.transform = candidate.transform * moved;
    }
  }

  const bool time_to_manage = (get_node()->now() - last_hypotheses_).seconds() > hypotheses_time_;

  if (harvested || time_to_manage) {
    manage_hypotheses(candidates, map);
  }

  if (time_to_manage) {
    last_hypotheses_ = get_node()->now();

    if (!matching_.valid() && !last_points_.empty()) {
      // The pyramid is rebuilt only when the map changes
      const int64_t map_stamp_ns = map.getLastModifiedStamp().nanoseconds();
      if (!matcher_ || map_stamp_ns != matcher_map_stamp_ns_) {
        matcher_ = std::make_shared<MapMatcher>(map, hp.matcher_levels, hp.matcher_angle_step);
        matcher_map_stamp_ns_ = map_stamp_ns;
      }

      matching_odom_ = odom_;
      matching_ = std::async(
        std::launch::async,
        [matcher = matcher_, points = last_points_, min_weight = hp.min_candidate_weight]() {
          return matcher->get_matches(points, min_weight);
        });
    }
  }
}

void
MHAMCLLocalizer::get_distances(
  const tf2::Transform & a, const tf2::Transform & b, double & dist_xy, double & dist_yaw)
{
  dist_xy = (a.getOrigin() - b.getOrigin()).length();
  dist_yaw = std::abs(normalize_angle(yaw_of(a) - yaw_of(b)));
}

bool
MHAMCLLocalizer::is_free(const easynav::Costmap2D & map, const tf2::Transform & pose)
{
  unsigned int mx, my;
  return map.worldToMap(pose.getOrigin().x(), pose.getOrigin().y(), mx, my) &&
         map.getCost(mx, my) == easynav::FREE_SPACE;
}

void
MHAMCLLocalizer::manage_hypotheses(
  const std::vector<TransformWeighted> & candidates, const easynav::Costmap2D & map)
{
  const auto & hp = hypotheses_params_;

  // Creation: new hypotheses where the perception fits and no other hypothesis is nearby
  for (const auto & candidate : candidates) {
    if (static_cast<int>(hypotheses_.size()) >= hp.max_hypotheses) {break;}
    if (candidate.weight <= hp.min_candidate_weight) {break;}  // Sorted from best to worst

    const bool covered = std::any_of(
      hypotheses_.begin(), hypotheses_.end(),
      [&](const Hypothesis & h) {
        double dist, diff_yaw;
        get_distances(h->get_pose(), candidate.transform, dist, diff_yaw);
        return dist < hp.min_candidate_distance && diff_yaw < hp.min_candidate_angle;
      });

    if (!covered) {
      auto hypothesis = std::make_shared<ParticlesDistribution>(particles_params_, rng_());
      hypothesis->init(
        candidate.transform, hp.new_hypothesis_std_xy, hp.new_hypothesis_std_yaw,
        candidate.weight);
      hypotheses_.push_back(hypothesis);
    }
  }

  // Destruction: hypotheses out of the free space or that do not explain the perception
  for (auto it = hypotheses_.begin(); it != hypotheses_.end(); ) {
    const double quality = (*it)->get_quality();
    const bool very_low = quality < hp.very_low_q_hypo_threshold;
    const bool low = quality < hp.low_q_hypo_threshold;
    const bool max_reached = static_cast<int>(hypotheses_.size()) == hp.max_hypotheses;
    const bool in_free = is_free(map, (*it)->get_pose());

    if (hypotheses_.size() > 1 && (!in_free || very_low || (low && max_reached))) {
      it = hypotheses_.erase(it);
    } else {
      ++it;
    }
  }

  if (std::find(hypotheses_.begin(), hypotheses_.end(), current_) == hypotheses_.end()) {
    current_ = hypotheses_.front();
  }

  // Merge: hypotheses that converged to the same pose
  for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
    for (std::size_t j = i + 1; j < hypotheses_.size(); ) {
      double dist, diff_yaw;
      get_distances(hypotheses_[i]->get_pose(), hypotheses_[j]->get_pose(), dist, diff_yaw);

      if (dist < hp.hypo_merge_distance && diff_yaw < hp.hypo_merge_angle) {
        hypotheses_[i]->merge(*hypotheses_[j]);
        if (current_ == hypotheses_[j]) {current_ = hypotheses_[i];}
        hypotheses_.erase(hypotheses_.begin() + j);
      } else {
        ++j;
      }
    }
  }

  // Selection: another hypothesis replaces the current one only if it is clearly better
  double current_quality = current_->get_quality();
  for (const auto & hypothesis : hypotheses_) {
    const double quality = hypothesis->get_quality();
    if (quality > hp.good_hypo_threshold && quality > current_quality + hp.min_hypo_diff_winner) {
      current_quality = quality;
      current_ = hypothesis;
    }
  }

  RCLCPP_DEBUG(
    get_node()->get_logger(), "%zu hypotheses, selected quality %lf", hypotheses_.size(),
    current_->get_quality());
}

tf2::Transform
MHAMCLLocalizer::estimated_pose_locked() const
{
  return current_ ? current_->get_pose() : tf2::Transform::getIdentity();
}

tf2::Transform
MHAMCLLocalizer::getEstimatedPose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return estimated_pose_locked();
}

std::size_t
MHAMCLLocalizer::get_num_hypotheses() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return hypotheses_.size();
}

void
MHAMCLLocalizer::publishTF(const tf2::Transform & map2odom)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = last_input_time_;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  tf_msg.header.frame_id = tf_info.map_frame;
  tf_msg.child_frame_id = tf_info.odom_frame;
  tf_msg.transform = tf2::toMsg(map2odom);

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  tf_broadcaster_->sendTransform(tf_msg);
}

void
MHAMCLLocalizer::publishEstimatedPose()
{
  if (!current_) {return;}

  const tf2::Transform est = estimated_pose_locked();
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = last_input_time_;
  msg.header.frame_id = tf_info.map_frame;
  msg.pose.pose.position.x = est.getOrigin().x();
  msg.pose.pose.position.y = est.getOrigin().y();
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = tf2::toMsg(est.getRotation());
  msg.pose.covariance = current_->get_covariance();

  estimate_pub_->publish(msg);
}

nav_msgs::msg::Odometry
MHAMCLLocalizer::pose_msg_locked() const
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = last_input_time_;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  odom_msg.header.frame_id = tf_info.map_frame;
  odom_msg.child_frame_id = tf_info.robot_footprint_frame;

  const tf2::Transform est = estimated_pose_locked();
  odom_msg.pose.pose.position.x = est.getOrigin().x();
  odom_msg.pose.pose.position.y = est.getOrigin().y();
  odom_msg.pose.pose.position.z = est.getOrigin().z();
  odom_msg.pose.pose.orientation = tf2::toMsg(est.getRotation());
  if (current_) {
    odom_msg.pose.covariance = current_->get_covariance();
  }

  return odom_msg;
}

nav_msgs::msg::Odometry
MHAMCLLocalizer::get_pose()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pose_msg_locked();
}

void
MHAMCLLocalizer::publishParticles()
{
  if (!current_) {return;}

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::PoseArray array_msg;
  array_msg.header.stamp = last_input_time_;
  array_msg.header.frame_id = tf_info.map_frame;
  array_msg.poses.reserve(current_->get_particles().size());
  for (const auto & p : current_->get_particles()) {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = p.pose.getOrigin().x();
    pose_msg.position.y = p.pose.getOrigin().y();
    pose_msg.position.z = p.pose.getOrigin().z();
    pose_msg.orientation = tf2::toMsg(p.pose.getRotation());
    array_msg.poses.push_back(pose_msg);
  }
  particles_pub_->publish(array_msg);

  if (hypotheses_pub_->get_subscription_count() == 0) {return;}

  static const float colors[][3] = {
    {0.8f, 0.1f, 0.1f}, {0.1f, 0.8f, 0.1f}, {0.1f, 0.1f, 0.8f}, {1.0f, 0.5f, 0.0f},
    {0.6f, 0.0f, 0.6f}, {0.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.4f, 1.0f}};

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int id = 0;
  for (const auto & hypothesis : hypotheses_) {
    const auto * color = colors[id % (sizeof(colors) / sizeof(colors[0]))];

    // The particles of the hypothesis...
    visualization_msgs::msg::Marker points;
    points.header.stamp = last_input_time_;
    points.header.frame_id = tf_info.map_frame;
    points.ns = "particles";
    points.id = id;
    points.type = visualization_msgs::msg::Marker::POINTS;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.r = color[0];
    points.color.g = color[1];
    points.color.b = color[2];
    points.color.a = 1.0;
    for (const auto & p : hypothesis->get_particles()) {
      geometry_msgs::msg::Point pt;
      pt.x = p.pose.getOrigin().x();
      pt.y = p.pose.getOrigin().y();
      points.points.push_back(pt);
    }
    markers.markers.push_back(points);

    // ...and its pose, thicker if it is the selected one
    visualization_msgs::msg::Marker arrow = points;
    arrow.ns = "poses";
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.points.clear();
    const tf2::Transform pose = hypothesis->get_pose();
    arrow.pose.position.x = pose.getOrigin().x();
    arrow.pose.position.y = pose.getOrigin().y();
    arrow.pose.orientation = tf2::toMsg(pose.getRotation());
    arrow.scale.x = 0.6;
    arrow.scale.y = hypothesis == current_ ? 0.15 : 0.05;
    arrow.scale.z = arrow.scale.y;
    markers.markers.push_back(arrow);

    ++id;
  }

  hypotheses_pub_->publish(markers);
}

}  // namespace mhamcl
}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::mhamcl::MHAMCLLocalizer, easynav::LocalizerMethodBase)
