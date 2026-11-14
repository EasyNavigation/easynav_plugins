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
#include <unordered_set>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "GeographicLib/UTMUPS.hpp"
#include "bonxai/pcl_utils.hpp"
#include "bonxai/probabilistic_map.hpp"
#include "navmap_core/NavMap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_sensors/types/PosePerception.hpp"
#include "easynav_sensors/types/TwistPerception.hpp"
#include "easynav_orbit_localizer/OrbitLocalizer.hpp"

namespace easynav
{
namespace orbit
{

namespace
{

Pose fromMsg(const geometry_msgs::msg::Pose & p)
{
  Pose X = Pose::Identity();
  X.linear() = Eigen::Quaterniond(
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).normalized()
    .toRotationMatrix();
  X.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return X;
}

Pose fromTransform(const geometry_msgs::msg::Transform & t)
{
  Pose X = Pose::Identity();
  X.linear() = Eigen::Quaterniond(
    t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).normalized().toRotationMatrix();
  X.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return X;
}

geometry_msgs::msg::Pose toMsg(const Pose & X)
{
  geometry_msgs::msg::Pose p;
  const Eigen::Quaterniond q(X.linear());
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.position.x = X.translation().x();
  p.position.y = X.translation().y();
  p.position.z = X.translation().z();
  return p;
}

geometry_msgs::msg::Transform toTransformMsg(const Pose & X)
{
  geometry_msgs::msg::Transform t;
  const Eigen::Quaterniond q(X.linear());
  t.rotation.w = q.w();
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.translation.x = X.translation().x();
  t.translation.y = X.translation().y();
  t.translation.z = X.translation().z();
  return t;
}

double seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
         .count();
}

// Covariance (right perturbation of X_mb) in the map frame, as nav_msgs wants it
Mat6 toMapFrame(const Pose & X, const Mat6 & P)
{
  Mat6 ad = Mat6::Zero();
  ad.block<3, 3>(0, 0) = X.linear();
  ad.block<3, 3>(3, 3) = X.linear();
  return ad * P * ad.transpose();
}

template<typename T, typename NodeT>
void param(const NodeT & node, const std::string & name, T & value)
{
  easynav::declare_parameter_if_absent<T>(*node, name, value);
  node->get_parameter(name, value);
}

}  // namespace

OrbitLocalizer::~OrbitLocalizer()
{
  graph_abort_ = true;
  if (graph_thread_.joinable()) {graph_thread_.join();}
}

void OrbitLocalizer::readParameters()
{
  auto node = get_node();
  const std::string p = get_plugin_name() + ".";
  std::string mode = "planar";
  param(node, p + "mode", mode);
  mode_ = mode == "free" ? Mode::Free : mode == "planar_terrain" ? Mode::PlanarTerrain :
    Mode::Planar;
  param(node, p + "map_type", map_type_);
  param(node, p + "odometry_source", odometry_source_);
  param(node, p + "odom_key", odom_key_);
  param(node, p + "dvl_key", dvl_key_);
  param(node, p + "depth_key", depth_key_);
  param(node, p + "imu_key", imu_key_);
  param(node, p + "gps_key", gps_key_);
  param(node, p + "navmap_key", navmap_key_);
  param(node, p + "noise_translation", noise_translation_);
  param(node, p + "noise_rotation", noise_rotation_);
  param(node, p + "noise_translation_to_rotation", noise_translation_to_rotation_);
  param(node, p + "min_noise_xy", min_noise_xy_);
  param(node, p + "min_noise_z", min_noise_z_);
  param(node, p + "min_noise_yaw", min_noise_yaw_);
  param(node, p + "min_noise_roll_pitch", min_noise_roll_pitch_);
  param(node, p + "initial_pose.use_last_known", use_last_known_pose_);
  param(node, p + "global_init", global_init_);
  param(node, p + "use_points", use_points_);
  param(node, p + "max_points", max_points_);
  param(node, p + "min_evidence_points", min_evidence_points_);
  param(node, p + "min_height", min_height_);
  param(node, p + "max_height", max_height_);
  param(node, p + "max_range", max_range_);
  param(node, p + "downsample", downsample_);
  param(node, p + "sonar_mode", sonar_mode_);
  param(node, p + "sonar_vertical_fov", sonar_vertical_fov_);
  match_options_.mode = mode_;
  param(node, p + "sigma_point", match_options_.sigma_point);
  param(node, p + "cauchy_c", match_options_.cauchy_c);
  param(node, p + "inlier_distance", match_options_.inlier_distance);
  param(node, p + "max_iterations", match_options_.max_iterations);
  param(node, p + "max_hypotheses", max_hypotheses_);
  param(node, p + "weight_sharpness", weight_sharpness_);
  param(node, p + "score_deadband", score_deadband_);
  param(node, p + "credit_max", credit_max_);
  param(node, p + "min_weight", min_weight_);
  param(node, p + "merge_distance", merge_distance_);
  param(node, p + "merge_angle", merge_angle_);
  param(node, p + "min_winner_time", min_winner_time_);
  param(node, p + "lost_score", lost_score_);
  param(node, p + "lost_cycles", lost_cycles_);
  param(node, p + "relocalize.candidates", reloc_candidates_);
  param(node, p + "relocalize.yaw_seeds", reloc_yaw_seeds_);
  param(node, p + "relocalize.min_score", reloc_min_score_);
  param(node, p + "relocalize.min_travel", reloc_min_travel_);
  param(node, p + "slip_sigmas", slip_sigmas_);
  param(node, p + "update_min_distance", update_min_distance_);
  param(node, p + "update_min_angle", update_min_angle_);
  param(node, p + "symmetry.enabled", symmetry_enabled_);
  double sample_step = 0.5;
  param(node, p + "symmetry.sample_step", sample_step);
  graph_options_.sample_step = sample_step;
  param(node, p + "symmetry.candidates", graph_options_.candidates);
  param(node, p + "symmetry.yaw_seeds", graph_options_.yaw_seeds);
  param(node, p + "symmetry.min_score", graph_options_.min_score);
  param(node, p + "symmetry.merge_distance", graph_options_.merge_distance);
  param(node, p + "symmetry.merge_angle", graph_options_.merge_angle);
  param(node, p + "symmetry.threads", graph_options_.threads);
  param(node, p + "symmetry.range", graph_range_);
  param(node, p + "symmetry.z_min", graph_z_min_);
  param(node, p + "symmetry.z_max", graph_z_max_);
  param(node, p + "symmetry.ground_height", graph_ground_height_);
  param(node, p + "symmetry.altitudes", graph_altitudes_);
  param(node, p + "symmetry.sensor_height", graph_rays_.sensor_height);
  param(node, p + "symmetry.azimuths", graph_rays_.azimuths);
  param(node, p + "symmetry.elevations", graph_rays_.elevations);
  param(node, p + "symmetry.elevation_min", graph_rays_.elevation_min);
  param(node, p + "symmetry.elevation_max", graph_rays_.elevation_max);
  param(node, p + "symmetry.cache_file", graph_cache_);
  param(node, p + "symmetry.spawn_weight", spawn_weight_);
  param(node, p + "gps.enabled", gps_enabled_);
  param(node, p + "gps.latitude_origin", latitude_origin_);
  param(node, p + "gps.longitude_origin", longitude_origin_);
  param(node, p + "gps.altitude_origin", altitude_origin_);
  param(node, p + "gps.projection", projection_);
  param(node, p + "gps.sigma_scale", gps_sigma_scale_);
  param(node, p + "gps.gate", gps_gate_);
  param(node, p + "gps.use_z", gps_use_z_);
  param(node, p + "gps.map_yaw", gps_map_yaw_);
  param(node, p + "terrain_max_step", terrain_max_step_);
  param(node, p + "imu.attitude", imu_attitude_);
  param(node, p + "imu.yaw", imu_yaw_);
  param(node, p + "imu.sigma_roll_pitch", imu_sigma_roll_pitch_);
  param(node, p + "imu.sigma_yaw", imu_sigma_yaw_);
  param(node, p + "depth.sigma", depth_sigma_);
  param(node, p + "depth.gate", depth_gate_);
  param(node, p + "sensor_fov.azimuth", fov_azimuth_);
  param(node, p + "sensor_fov.elevation_min", fov_elevation_min_);
  param(node, p + "sensor_fov.elevation_max", fov_elevation_max_);
  param(node, p + "debug.markers", debug_markers_);
}

void OrbitLocalizer::on_initialize()
{
  auto node = get_node();
  const std::string p = get_plugin_name() + ".";
  readParameters();

  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, sxy = 0.5, syaw = 0.3, sz = 0.5;
  param(node, p + "initial_pose.x", x);
  param(node, p + "initial_pose.y", y);
  param(node, p + "initial_pose.z", z);
  param(node, p + "initial_pose.yaw", yaw);
  param(node, p + "initial_pose.std_dev_xy", sxy);
  param(node, p + "initial_pose.std_dev_z", sz);
  param(node, p + "initial_pose.std_dev_yaw", syaw);
  Mat6 P = Mat6::Zero();
  P.diagonal() << sxy * sxy, sxy * sxy, sz * sz, 0.01, 0.01, syaw * syaw;
  resetAt(fromXYZYaw(x, y, z, yaw), P);

  if (gps_enabled_) {
    if (projection_ == "enu") {
      enu_ = std::make_unique<GeographicLib::LocalCartesian>(
        latitude_origin_, longitude_origin_, altitude_origin_);
    } else {
      bool northp;
      GeographicLib::UTMUPS::Forward(
        latitude_origin_, longitude_origin_, utm_zone_, northp, utm_origin_x_, utm_origin_y_);
    }
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  init_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&OrbitLocalizer::initPoseCallback, this, std::placeholders::_1));
  const std::string prefix = node->get_node_base_interface()->get_fully_qualified_name() +
    std::string("/") + get_plugin_name();
  hyps_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>(prefix + "/hypotheses", 10);
  pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    prefix + "/pose", 10);
  if (debug_markers_) {
    markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      prefix + "/markers", 10);
    graph_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      prefix + "/graph", rclcpp::QoS(1).transient_local());
  }
  RCLCPP_INFO(
    node->get_logger(), "OrbitLocalizer: mode %s, map %s, odometry %s, at (%.2f, %.2f, %.2f)",
    mode_ == Mode::Free ? "free" : mode_ == Mode::PlanarTerrain ? "planar_terrain" : "planar",
    map_type_.c_str(), odometry_source_.c_str(), x, y, yaw);
}

void OrbitLocalizer::resetAt(const Pose & X_mb, const Mat6 & P)
{
  std::lock_guard<std::mutex> lock(hyp_mutex_);
  pending_reset_ = std::make_pair(X_mb, P);
}

void OrbitLocalizer::on_last_known_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  if (!use_last_known_pose_) {return;}
  Mat6 P = Mat6::Zero();
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      P(r, c) = pose.pose.covariance[6 * r + c];
    }
  }
  for (int k = 0; k < 6; ++k) {
    P(k, k) = std::max(P(k, k), 1e-4);
  }
  resetAt(fromMsg(pose.pose.pose), P);
  RCLCPP_INFO(
    get_node()->get_logger(), "OrbitLocalizer: starting from the last known pose (%.2f, %.2f)",
    pose.pose.pose.position.x, pose.pose.pose.position.y);
}

void OrbitLocalizer::initPoseCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg)
{
  Mat6 P = Mat6::Zero();
  const auto & c = msg->pose.covariance;
  P.diagonal() << std::max(c[0], 1e-4), std::max(c[7], 1e-4), std::max(c[14], 1e-4),
    0.01, 0.01, std::max(c[35], 1e-4);
  resetAt(fromMsg(msg->pose.pose), P);
  RCLCPP_INFO(
    get_node()->get_logger(), "OrbitLocalizer: initial pose (%.2f, %.2f, yaw %.2f)",
    msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
}

std::optional<Pose> OrbitLocalizer::mounting(const std::string & frame)
{
  const auto it = mountings_.find(frame);
  if (it != mountings_.end()) {return it->second;}
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  try {
    const auto msg = RTTFBuffer::getInstance()->lookupTransform(
      tf_info.robot_footprint_frame, frame, tf2::TimePointZero);
    const Pose X = fromTransform(msg.transform);
    mountings_[frame] = X;
    return X;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "OrbitLocalizer: no mounting of %s: %s", frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<Eigen::Matrix3d> OrbitLocalizer::imuAttitude(NavState & nav_state)
{
  if (!nav_state.has(imu_key_)) {return std::nullopt;}
  const auto & imu = nav_state.get<IMUPerception>(imu_key_);
  const auto & o = imu.data.orientation;
  if (imu.data.orientation_covariance[0] == -1.0) {return std::nullopt;}
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  if (!std::isfinite(q.squaredNorm()) || q.squaredNorm() < 1e-6) {return std::nullopt;}
  q.normalize();
  Eigen::Matrix3d R_body_imu = Eigen::Matrix3d::Identity();
  if (!imu.data.header.frame_id.empty()) {
    if (const auto m = mounting(imu.data.header.frame_id)) {R_body_imu = m->linear();}
  }
  // The IMU's orientation is its frame's: body = imu * mounting^-1
  return (q.toRotationMatrix() * R_body_imu.transpose()).eval();
}

bool OrbitLocalizer::updateOdometry(NavState & nav_state)
{
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  if (odometry_source_ == "tf") {
    geometry_msgs::msg::TransformStamped msg;
    try {
      msg = RTTFBuffer::getInstance()->lookupTransform(
        tf_info.odom_frame, tf_info.robot_footprint_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "OrbitLocalizer: odometry TF failed: %s", ex.what());
      return false;
    }
    std::lock_guard<std::mutex> lock(odom_mutex_);
    const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
    const Pose X = fromTransform(msg.transform);
    if (odom_ok_ && stamp > odom_stamp_) {
      // TF has no velocity: the motion since the previous sample
      const double dt = (stamp - odom_stamp_).seconds();
      const Pose delta = X_ob_.inverse() * X;
      odom_twist_.linear.x = delta.translation().x() / dt;
      odom_twist_.linear.y = delta.translation().y() / dt;
      odom_twist_.linear.z = delta.translation().z() / dt;
      odom_twist_.angular.z = yawOf(delta) / dt;
    }
    X_ob_ = X;
    odom_stamp_ = stamp;
    odom_ok_ = true;
    return true;
  }
  if (odometry_source_ == "dvl_imu") {
    // Dead reckoning: the DVL's velocity (body) turned by the IMU's attitude, integrated
    const auto R = imuAttitude(nav_state);
    std::optional<Eigen::Vector3d> dvl;
    std::string dvl_frame;
    rclcpp::Time dvl_stamp(0, 0, RCL_ROS_TIME);
    if (nav_state.has(dvl_key_)) {
      const auto & t = nav_state.get<TwistPerception>(dvl_key_);
      const auto & v = t.data.twist.twist.linear;
      if (t.valid && std::isfinite(v.x + v.y + v.z)) {
        dvl = Eigen::Vector3d(v.x, v.y, v.z);
        dvl_frame = t.data.header.frame_id;
        dvl_stamp = t.stamp;
      }
    }
    std::lock_guard<std::mutex> lock(odom_mutex_);
    const rclcpp::Time now = get_node()->now();
    if (!R) {return odom_ok_;}
    if (dead_reckoning_stamp_.nanoseconds() == 0) {
      dead_reckoning_stamp_ = now;
      X_ob_.linear() = *R;
      odom_stamp_ = now;
      odom_ok_ = true;
      return true;
    }
    const double dt = (now - dead_reckoning_stamp_).seconds();
    if (dt <= 0.0) {return odom_ok_;}
    dead_reckoning_stamp_ = now;
    // Without a DVL reading of the last second (no bottom lock) the vehicle is taken as still
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (dvl && std::abs((now - dvl_stamp).seconds()) < 1.0) {
      v = *dvl;
      if (!dvl_frame.empty()) {
        if (const auto m = mounting(dvl_frame)) {v = m->linear() * v;}
      }
    }
    X_ob_.translation() += *R * v * dt;
    X_ob_.linear() = *R;
    odom_stamp_ = now;
    odom_twist_.linear.x = v.x();
    odom_twist_.linear.y = v.y();
    odom_twist_.linear.z = v.z();
    odom_ok_ = true;
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = tf_info.odom_frame;
    tf_msg.child_frame_id = tf_info.robot_footprint_frame;
    tf_msg.transform = toTransformMsg(X_ob_);
    RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
    tf_broadcaster_->sendTransform(tf_msg);
    return true;
  }
  // "navstate": the odometry message a sensor of sensors_node puts in the NavState
  if (!nav_state.has(odom_key_)) {return false;}
  const auto & odom = nav_state.get<nav_msgs::msg::Odometry>(odom_key_);
  std::lock_guard<std::mutex> lock(odom_mutex_);
  X_ob_ = fromMsg(odom.pose.pose);
  odom_stamp_ = rclcpp::Time(odom.header.stamp, RCL_ROS_TIME);
  odom_twist_ = odom.twist.twist;
  odom_ok_ = true;
  return true;
}

void OrbitLocalizer::update_rt(NavState & nav_state)
{
  if (!updateOdometry(nav_state)) {return;}
  Pose X_ob;
  rclcpp::Time stamp;
  geometry_msgs::msg::Twist twist;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    X_ob = X_ob_;
    stamp = odom_stamp_;
    twist = odom_twist_;
  }
  Pose X_mo;
  Mat6 P;
  bool have;
  {
    std::lock_guard<std::mutex> lock(hyp_mutex_);
    X_mo = winner_X_mo_;
    P = winner_P_;
    have = have_estimate_;
  }
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_history_.empty() || stamp > odom_history_.back().first) {
      odom_history_.emplace_back(stamp, X_ob);
      while (odom_history_.size() > 1 &&
        (stamp - odom_history_.front().first).seconds() > 3.0) {odom_history_.pop_front();}
    }
  }
  if (!have) {return;}
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = tf_info.map_frame;
  tf_msg.child_frame_id = tf_info.odom_frame;
  tf_msg.transform = toTransformMsg(X_mo);
  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  tf_broadcaster_->sendTransform(tf_msg);

  auto odom = poseMessage(X_mo * X_ob, P);
  odom.header.stamp = stamp;
  odom.twist.twist = twist;
  nav_state.set("robot_pose", odom);
}

nav_msgs::msg::Odometry OrbitLocalizer::poseMessage(const Pose & X_mb, const Mat6 & P) const
{
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = tf_info.map_frame;
  odom.child_frame_id = tf_info.robot_footprint_frame;
  odom.pose.pose = toMsg(X_mb);
  const Mat6 C = toMapFrame(X_mb, P);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      odom.pose.covariance[6 * r + c] = C(r, c);
    }
  }
  return odom;
}

constexpr int kViewsVersion = 2;   // bump when the views' recipe changes: caches rebuild

// FNV-1a over raw bytes: the fingerprint of a map and the options a graph was built from
static uint64_t fnv(uint64_t h, const void * data, std::size_t n)
{
  const auto * p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h = (h ^ p[i]) * 1099511628211ULL;
  }
  return h;
}

template<typename T>
static uint64_t fnvOf(uint64_t h, const T & v)
{
  return fnv(h, &v, sizeof(v));
}

bool OrbitLocalizer::ensureMap(NavState & nav_state)
{
  if (field_) {return true;}
  uint64_t fp = 14695981039346656037ULL;
  if (map_type_ == "costmap") {
    if (!nav_state.has("map.base")) {return false;}
    const auto & map = nav_state.get<Costmap2D>("map.base");
    if (map.getSizeInCellsX() == 0) {return false;}
    map_path_ = nav_state.has("map.base.path") ? nav_state.get<std::string>("map.base.path") : "";
    fp = fnvOf(fp, map.getResolution());
    fp = fnvOf(fp, map.getOriginX());
    fp = fnvOf(fp, map.getOriginY());
    for (unsigned int j = 0; j < map.getSizeInCellsY(); ++j) {
      for (unsigned int i = 0; i < map.getSizeInCellsX(); ++i) {
        fp = fnvOf(fp, map.getCost(i, j));
      }
    }
    field2d_ = std::make_unique<GridDistanceField2D>(map);
    field_ = field2d_.get();
    views_ = std::make_unique<GridViews2D>(*field2d_, graph_range_, graph_options_.sample_step);
  } else {
    if (!nav_state.has("map.bonxai")) {return false;}
    const auto map = nav_state.get_ptr<Bonxai::ProbabilisticMap>("map.bonxai");
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<Bonxai::CoordT> coords;
    map->getOccupiedVoxels(coords);
    if (coords.empty()) {return false;}
    map_path_ = nav_state.has("map.bonxai.path") ?
      nav_state.get<std::string>("map.bonxai.path") : "";
    fp = fnvOf(fp, map->grid().voxelSize());
    fp = fnv(fp, coords.data(), coords.size() * sizeof(coords[0]));
    // The map's own points when its file is at hand: a voxel's centre is up to half a voxel
    // off the surface it holds, and the views cast on the voxels would sit a voxel short
    if (!map_path_.empty() && std::filesystem::path(map_path_).extension() == ".pcd") {
      std::vector<Eigen::Vector3d> file_points;
      if (Bonxai::ReadPointsFromPCD(map_path_, file_points) && !file_points.empty()) {
        cloud->points.reserve(file_points.size());
        for (const auto & p : file_points) {
          if (!std::isfinite(p.x() + p.y() + p.z())) {continue;}
          cloud->points.emplace_back(
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        RCLCPP_INFO(
          get_node()->get_logger(), "OrbitLocalizer: %zu map points from %s",
          cloud->points.size(), map_path_.c_str());
      }
    }
    if (cloud->points.empty()) {   // no file: the voxel centres
      cloud->points.reserve(coords.size());
      for (const auto & c : coords) {
        const auto p = map->grid().coordToPos(c);
        cloud->points.emplace_back(
          static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
      }
    }
    if (cloud->width == 0) {   // the voxel centres
      cloud->width = static_cast<uint32_t>(cloud->points.size());
      cloud->height = 1;
    }
    field3d_ = std::make_unique<CloudDistanceField3D>(
      cloud, map->grid().voxelSize(), std::max(3.0, 10.0 * map->grid().voxelSize()));
    field_ = field3d_.get();
    auto views3d = std::make_unique<CloudViews3D>(
      *field3d_, graph_range_, graph_z_min_, graph_z_max_, graph_options_.sample_step,
      graph_altitudes_, graph_ground_height_, graph_rays_);
    if (mode_ == Mode::PlanarTerrain && nav_state.has(navmap_key_)) {
      // The places are the navigable surfaces, sampled every step: stacked floors each get
      // theirs (the lowest point of a column would put the upper floor's places downstairs)
      const auto & navmap = nav_state.get<::navmap::NavMap>(navmap_key_);
      const double step = graph_options_.sample_step;
      std::unordered_set<int64_t> taken;
      std::vector<Eigen::Vector3d> places;
      for (std::size_t cid = 0; cid < navmap.navcels.size(); ++cid) {
        const Eigen::Vector3f c = navmap.navcel_centroid(static_cast<::navmap::NavCelId>(cid));
        const int64_t i = static_cast<int64_t>(std::floor(c.x() / step));
        const int64_t j = static_cast<int64_t>(std::floor(c.y() / step));
        const int64_t k = static_cast<int64_t>(std::floor(c.z() / 1.0));
        const int64_t key = (i & 0x1FFFFF) | ((j & 0x1FFFFF) << 21) | ((k & 0x1FFFFF) << 42);
        if (!taken.insert(key).second) {continue;}
        places.emplace_back(
          (i + 0.5) * step, (j + 0.5) * step, static_cast<double>(c.z()) + graph_ground_height_);
      }
      fp = fnvOf(fp, places.size());
      if (!places.empty()) {views3d->setPlaces(std::move(places));}
    }
    views_ = std::move(views3d);
  }
  // The graph depends on how the places and their views are made
  fp = fnvOf(fp, graph_options_.sample_step);
  fp = fnvOf(fp, graph_options_.candidates);
  fp = fnvOf(fp, graph_options_.yaw_seeds);
  fp = fnvOf(fp, graph_options_.min_score);
  fp = fnvOf(fp, graph_options_.merge_distance);
  fp = fnvOf(fp, graph_options_.merge_angle);
  fp = fnvOf(fp, graph_range_);
  fp = fnvOf(fp, graph_z_min_);
  fp = fnvOf(fp, graph_z_max_);
  fp = fnvOf(fp, graph_ground_height_);
  fp = fnv(fp, graph_altitudes_.data(), graph_altitudes_.size() * sizeof(double));
  fp = fnvOf(fp, graph_rays_);
  fp = fnvOf(fp, kViewsVersion);
  graph_fingerprint_ = fp == 0 ? 1 : fp;
  if (match_options_.inlier_distance <= 0.0) {
    match_options_.inlier_distance = std::max(0.15, 1.5 * field_->resolution());
  }
  matcher_ = std::make_unique<Matcher>(*field_, match_options_);
  RCLCPP_INFO(
    get_node()->get_logger(), "OrbitLocalizer: map ready (%s, resolution %.2f m)",
    map_type_.c_str(), field_->resolution());
  if (symmetry_enabled_) {startGraphBuild();}
  return true;
}

void OrbitLocalizer::startGraphBuild()
{
  // The cache: the file given, or one named after the map next to it (<map>.orbit), or that
  // name under ~/.cache/easynav/orbit when the map's folder cannot be written. A cache of
  // another map or other options (the fingerprint) is rebuilt and overwritten
  std::vector<std::string> paths;
  if (!graph_cache_.empty()) {
    paths.push_back(graph_cache_);
  } else if (!map_path_.empty()) {
    const std::filesystem::path map(map_path_);
    const std::string name = map.stem().string() + ".orbit";
    paths.push_back((map.parent_path() / name).string());
    if (const char * home = std::getenv("HOME")) {
      paths.push_back(
        (std::filesystem::path(home) / ".cache" / "easynav" / "orbit" / name).string());
    }
  }
  for (const auto & p : paths) {
    if (std::filesystem::exists(p) && graph_.load(p, graph_fingerprint_)) {
      graph_ready_ = true;
      RCLCPP_INFO(
        get_node()->get_logger(), "OrbitLocalizer: symmetry graph loaded from %s (%zu places, "
        "%zu alternatives)", p.c_str(), graph_.nodes().size(), graph_.alternativesCount());
      return;
    }
  }
  graph_building_ = true;
  graph_thread_ = std::thread(
    [this, paths]() {
      const double t0 = seconds();
      auto logger = get_node()->get_logger();
      double last_report = 0.0;
      // Its own matcher: planar, without a prior, less strict on convergence
      MatchOptions opts = match_options_;
      opts.mode = Mode::Planar;
      opts.max_iterations = 8;
      Matcher matcher(*field_, opts);
      graph_.build(
        *views_, matcher, graph_options_, [&](double f) {
          if (f - last_report >= 0.1) {
            last_report = f;
            RCLCPP_INFO(logger, "OrbitLocalizer: symmetry graph %.0f%%", 100.0 * f);
          }
          return !graph_abort_.load();
        });
      if (graph_abort_) {graph_building_ = false; return;}
      RCLCPP_INFO(
        logger, "OrbitLocalizer: symmetry graph built in %.1f s: %zu places, %zu alternatives",
        seconds() - t0, graph_.nodes().size(), graph_.alternativesCount());
      bool saved = false;
      for (const auto & p : paths) {
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(p).parent_path(), ec);
        if (graph_.save(p, graph_fingerprint_)) {
          RCLCPP_INFO(logger, "OrbitLocalizer: symmetry graph saved to %s", p.c_str());
          saved = true;
          break;
        }
      }
      if (!saved && !paths.empty()) {
        RCLCPP_WARN(logger, "OrbitLocalizer: could not save the symmetry graph to %s",
            paths.front().c_str());
      }
      graph_ready_ = true;
      graph_building_ = false;
    });
}

Pose OrbitLocalizer::odomAt(const rclcpp::Time & stamp) const
{
  if (odometry_source_ == "tf" && stamp.nanoseconds() > 0) {
    const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
    try {
      const auto msg = RTTFBuffer::getInstance()->lookupTransform(
        tf_info.odom_frame, tf_info.robot_footprint_frame, tf2_ros::fromRclcpp(stamp));
      return fromTransform(msg.transform);
    } catch (const tf2::TransformException &) {
    }
  }
  std::lock_guard<std::mutex> lock(odom_mutex_);
  Pose best = X_ob_;
  double best_dt = std::numeric_limits<double>::infinity();
  for (const auto & [t, X] : odom_history_) {
    const double dt = std::abs((t - stamp).seconds());
    if (dt < best_dt) {best_dt = dt; best = X;}
  }
  return stamp.nanoseconds() > 0 && best_dt < 0.5 ? best : X_ob_;
}

std::vector<Eigen::Vector3d> OrbitLocalizer::collectPoints(
  NavState & nav_state, rclcpp::Time & stamp, std::vector<Eigen::Vector3d> * all_points,
  std::vector<Eigen::Vector3d> * raw_points)
{
  std::vector<Eigen::Vector3d> out;
  stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (!use_points_) {return out;}
  const auto all = nav_state.get_no_group<PointPerception>();
  PointPerceptions valid;
  for (const auto & h : all) {
    if (h && h->valid) {valid.push_back(h);}
  }
  if (valid.empty()) {return out;}
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const double res = downsample_ > 0.0 ? downsample_ : field_->resolution();
  // Each cloud where the sensor was when it took it (exact time): the odometry of that time
  // pairs with them (odomAt), so a robot turning does not score its scans rotated
  if (raw_points) {  // every return, for the views (a range image needs them all)
    PointPerceptionsOpsView raw(valid);
    rclcpp::Time raw_stamp;
    raw.fuse(tf_info.robot_footprint_frame, raw_stamp, true)
    .filter({NAN, NAN, min_height_}, {NAN, NAN, max_height_});
    const auto cloud = raw.as_points();
    raw_points->clear();
    raw_points->reserve(cloud.points.size());
    const double r2 = max_range_ * max_range_;
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x + p.y + p.z)) {continue;}
      const Eigen::Vector3d v(p.x, p.y, p.z);
      if (v.squaredNorm() > r2 || v.squaredNorm() < 1e-4) {continue;}
      raw_points->push_back(v);
    }
  }
  PointPerceptionsOpsView view(valid);
  view.downsample(res).fuse(tf_info.robot_footprint_frame, stamp, true)
  .filter({NAN, NAN, min_height_}, {NAN, NAN, max_height_});
  const auto cloud = view.as_points();
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(cloud.points.size());
  const double r2 = max_range_ * max_range_;
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x + p.y + p.z)) {continue;}
    const Eigen::Vector3d v(p.x, p.y, p.z);
    if (v.squaredNorm() > r2 || v.squaredNorm() < 1e-4) {continue;}
    pts.push_back(v);
  }
  if (all_points) {*all_points = pts;}
  if (static_cast<int>(pts.size()) > max_points_ && max_points_ > 0) {
    const double step = static_cast<double>(pts.size()) / max_points_;
    out.reserve(max_points_);
    for (int k = 0; k < max_points_; ++k) {
      out.push_back(pts[static_cast<std::size_t>(k * step)]);
    }
    return out;
  }
  return pts;
}

void OrbitLocalizer::sonarElevations(std::vector<Eigen::Vector3d> & points, const Pose & X) const
{
  // An imaging sonar gives range and bearing but not elevation: each echo is placed at the
  // elevation, within the vertical aperture, where it fits the map best
  const auto it = mountings_.find("sonar_rays_link");
  Pose X_bs = Pose::Identity();
  if (it != mountings_.end()) {X_bs = it->second;}
  const Eigen::Vector3d o = X_bs.translation();
  const Eigen::Vector3d up = X_bs.linear().col(2);
  const int steps = 8;
  for (auto & p : points) {
    const Eigen::Vector3d d = p - o;
    const double r = d.norm();
    if (r < 1e-6) {continue;}
    const Eigen::Vector3d dir = d / r;
    Eigen::Vector3d best = p;
    double best_d = std::numeric_limits<double>::infinity();
    for (int k = -steps; k <= steps; ++k) {
      const double e = 0.5 * sonar_vertical_fov_ * k / steps;
      const Eigen::Vector3d q = o + r * (std::cos(e) * dir + std::sin(e) * up);
      const double dist = field_->distance(X * q, nullptr);
      if (dist < best_d) {best_d = dist; best = q;}
    }
    p = best;
  }
}

Mat6 OrbitLocalizer::processNoise(const Pose & U) const
{
  const Vec6 u = logSE3(U);
  const double t = u.head<3>().norm(), a = u.tail<3>().norm();
  const double s_xy = noise_translation_ * t + min_noise_xy_;
  const double s_z = noise_translation_ * t + min_noise_z_;
  const double s_yaw = noise_rotation_ * a + noise_translation_to_rotation_ * t + min_noise_yaw_;
  const double s_rp = noise_rotation_ * a + min_noise_roll_pitch_;
  Mat6 Q = Mat6::Zero();
  Q.diagonal() << s_xy * s_xy, s_xy * s_xy, s_z * s_z, s_rp * s_rp, s_rp * s_rp, s_yaw * s_yaw;
  return Q;
}

void OrbitLocalizer::collectAbsolute(
  NavState & nav_state, const Pose & X_pred, const Mat6 & P,
  std::vector<PositionMeasurement> & positions, std::vector<DepthMeasurement> & depths,
  std::vector<AttitudeMeasurement> & attitudes)
{
  if (gps_enabled_) {
    std::vector<std::shared_ptr<GNSSPerception>> fixes;
    if (nav_state.has_group("gnss")) {
      fixes = nav_state.get_group<GNSSPerception>("gnss");
    } else if (nav_state.has(gps_key_)) {
      fixes.push_back(nav_state.get_ptr<GNSSPerception>(gps_key_));
    }
    if (gps_last_stamp_.size() < fixes.size()) {
      gps_last_stamp_.resize(fixes.size(), rclcpp::Time(0, 0, RCL_ROS_TIME));
    }
    for (std::size_t i = 0; i < fixes.size(); ++i) {
      const auto & fix = fixes[i]->data;
      if (!fixes[i]->valid || fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        continue;
      }
      const rclcpp::Time stamp(fix.header.stamp, RCL_ROS_TIME);
      if (stamp <= gps_last_stamp_[i]) {continue;}
      gps_last_stamp_[i] = stamp;
      PositionMeasurement m;
      if (enu_) {
        enu_->Forward(fix.latitude, fix.longitude, fix.altitude, m.p_map.x(), m.p_map.y(),
          m.p_map.z());
      } else {
        int zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(
          fix.latitude, fix.longitude, zone, northp, m.p_map.x(), m.p_map.y(), utm_zone_);
        m.p_map.x() -= utm_origin_x_;
        m.p_map.y() -= utm_origin_y_;
        m.p_map.z() = fix.altitude - altitude_origin_;
      }
      if (gps_map_yaw_ != 0.0) {  // the map's axes turned from east-north
        const double c = std::cos(gps_map_yaw_), sn = std::sin(gps_map_yaw_);
        const double ex = m.p_map.x(), ny = m.p_map.y();
        m.p_map.x() = c * ex - sn * ny;
        m.p_map.y() = sn * ex + c * ny;
      }
      if (const auto mt = mounting(fix.header.frame_id)) {m.lever = mt->translation();}
      const bool known = fix.position_covariance_type !=
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      m.sigma = Eigen::Vector3d(
        known ? std::sqrt(std::max(fix.position_covariance[0], 1e-4)) : 1.0,
        known ? std::sqrt(std::max(fix.position_covariance[4], 1e-4)) : 1.0,
        known ? std::sqrt(std::max(fix.position_covariance[8], 1e-4)) : 1.0) * gps_sigma_scale_;
      m.use_z = gps_use_z_ && mode_ == Mode::Free;
      // Gate: a fix the prediction cannot explain (a jump, multipath) is left out; many in a row
      // mean the prediction is the wrong one
      const Eigen::Vector3d r = X_pred * m.lever - m.p_map;
      Eigen::Matrix<double, 3, 6> J;
      J.block<3, 3>(0, 0) = X_pred.linear();
      J.block<3, 3>(0, 3) = -X_pred.linear() * hat(m.lever);
      const int n = m.use_z ? 3 : 2;
      const Eigen::MatrixXd S = (J * P * J.transpose()).topLeftCorner(n, n) +
        m.sigma.head(n).cwiseAbs2().asDiagonal().toDenseMatrix();
      const double d2 = r.head(n).transpose() * S.inverse() * r.head(n);
      if (d2 > gps_gate_ && gps_rejected_ < 10) {
        ++gps_rejected_;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "OrbitLocalizer: GPS fix %.1f m off the prediction rejected (d2 %.1f)", r.norm(),
          d2);
        continue;
      }
      gps_rejected_ = 0;
      positions.push_back(m);
    }
  }
  if (!depth_key_.empty() && nav_state.has(depth_key_)) {
    const auto & pose = nav_state.get<PosePerception>(depth_key_);
    if (pose.valid && pose.stamp > depth_used_) {
      depth_used_ = pose.stamp;
      DepthMeasurement d;
      d.z = pose.data.pose.pose.position.z;
      const double var = pose.data.pose.covariance[14];
      d.sigma = var > 0.0 ? std::sqrt(var) : depth_sigma_;
      const double r = X_pred.translation().z() - d.z;
      const double s2 = P(2, 2) + d.sigma * d.sigma;
      // Ten readings in a row off the prediction mean the prediction is the wrong one
      if (r * r / s2 > depth_gate_ && depth_rejected_ < 10) {
        ++depth_rejected_;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "OrbitLocalizer: depth %.1f m off the prediction rejected", r);
      } else {
        depth_rejected_ = 0;
        depths.push_back(d);
      }
    }
  }
  if (imu_attitude_) {
    if (const auto R = imuAttitude(nav_state)) {
      AttitudeMeasurement a;
      a.R_map_body = *R;
      a.sigma_roll_pitch = imu_sigma_roll_pitch_;
      a.use_yaw = imu_yaw_;
      a.sigma_yaw = imu_sigma_yaw_;
      attitudes.push_back(a);
    }
  }
}

void OrbitLocalizer::placeOnTerrain(
  NavState & nav_state, Pose & X, const std::optional<Eigen::Matrix3d> & R_imu)
{
  if (mode_ != Mode::PlanarTerrain || !nav_state.has(navmap_key_)) {return;}
  const auto & navmap = nav_state.get<::navmap::NavMap>(navmap_key_);
  ::navmap::NavMap::LocateOpts opts;
  opts.planar_eps = 1e-4f;
  opts.height_eps = 0.5f;
  opts.use_downward_ray = true;
  std::size_t sidx = 0;
  ::navmap::NavCelId cid = std::numeric_limits<uint32_t>::max();
  Eigen::Vector3f bary, hit;
  const Eigen::Vector3f p = X.translation().cast<float>();
  if (!navmap.locate_navcel(p, sidx, cid, bary, &hit, opts)) {return;}
  // Another level through a hole of the surface (stacked floors) is not a step the robot took
  if (std::abs(hit.z() - p.z()) > terrain_max_step_) {return;}
  X.translation().z() = hit.z();
  const double yaw = yawOf(X);
  if (R_imu) {
    const Eigen::Vector3d rpy = rpyOf(*R_imu);
    X.linear() = fromRPY(rpy.x(), rpy.y(), yaw);
    return;
  }
  Eigen::Vector3d n = navmap.navcels[cid].normal.template cast<double>();
  if (n.norm() < 1e-9) {return;}
  n.normalize();
  if (n.z() < 0.0) {n = -n;}
  // The frame whose z is the normal and whose x heads yaw
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d x = heading - heading.dot(n) * n;
  if (x.norm() < 1e-9) {return;}
  x.normalize();
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = n.cross(x);
  R.col(2) = n;
  X.linear() = R;
}

constexpr double kMinViewFraction = 0.25;   // of the rays with a return, to judge coverage

CoverageBand OrbitLocalizer::coverageBand() const
{
  CoverageBand band;
  band.planar = views_ && views_->planar();
  band.z_min = min_height_;
  band.z_max = max_height_;
  band.azimuth_fov = fov_azimuth_;
  band.elevation_min = fov_elevation_min_;
  band.elevation_max = fov_elevation_max_;
  return band;
}

double OrbitLocalizer::fitScore(
  double forward, const Pose & X, const std::vector<Eigen::Vector3d> & scan) const
{
  if (!views_ || scan.empty() || forward <= 0.0) {return forward;}
  const auto view = views_->fromMap(X.translation());
  // A view of a few rays (open ground, a sparse corner of the map) cannot judge the coverage
  if (view.points.size() < kMinViewFraction * views_->rays()) {return forward;}
  const double cov = coverage(
    view.points, X.translation(), scan, X, matcher_->options().inlier_distance, coverageBand());
  if (std::getenv("ORBIT_TRACE")) {
    // Which view points the scan does not reach: their height and range say what they are
    std::size_t judged = 0, far = 0;
    double zsum = 0.0, rsum = 0.0, zcov = 0.0, rcov = 0.0;
    std::size_t ncov = 0;
    const double d2 = std::pow(matcher_->options().inlier_distance, 2);
    for (const auto & v : view.points) {
      if (!views_->planar() && (v.z() < min_height_ || v.z() > max_height_)) {continue;}
      ++judged;
      const Eigen::Vector3d w = X.translation() + v;
      bool near = false;
      for (const auto & q : scan) {
        if ((X * q - w).squaredNorm() <= d2) {near = true; break;}
      }
      if (near) {++ncov; zcov += v.z(); rcov += v.norm();} else {
        ++far; zsum += v.z(); rsum += v.norm();
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "OrbitLocalizer: fit at (%.1f, %.1f, z %.1f): forward %.2f, coverage %.2f (%zu view "
      "points, %zu in band, %zu scan points); uncovered %zu: mean z %.2f range %.1f; covered "
      "%zu: mean z %.2f range %.1f", X.translation().x(), X.translation().y(),
      X.translation().z(), forward, cov, view.points.size(), judged, scan.size(), far,
      far ? zsum / far : 0.0, far ? rsum / far : 0.0, ncov, ncov ? zcov / ncov : 0.0,
      ncov ? rcov / ncov : 0.0);
  }
  return std::sqrt(forward * cov);
}

void OrbitLocalizer::correctAll(
  NavState & nav_state, const std::vector<Eigen::Vector3d> & points, const Pose & X_ob)
{
  const auto R_imu = (mode_ == Mode::PlanarTerrain) ? imuAttitude(nav_state) : std::nullopt;
  std::vector<Hypothesis> left_behind;
  for (auto & h : hyps_) {
    const Pose U = h.X_ob_at.inverse() * X_ob;
    Pose X_pred = h.X_mo * X_ob;
    const Mat6 AdU = adjoint(U.inverse());
    Mat6 P = AdU * h.P * AdU.transpose() + processNoise(U);
    // A planar odometry knows nothing of height: the IMU's pitch over the distance driven
    // predicts it, so that on a ramp the surface found is the ramp, not the floor beneath it
    if (mode_ == Mode::PlanarTerrain && R_imu) {
      const double pitch = rpyOf(*R_imu).y();
      X_pred.translation().z() += std::sin(-pitch) * U.translation().x();
    }
    placeOnTerrain(nav_state, X_pred, R_imu);
    std::vector<PositionMeasurement> positions;
    std::vector<DepthMeasurement> depths;
    std::vector<AttitudeMeasurement> attitudes;
    collectAbsolute(nav_state, X_pred, P, positions, depths, attitudes);
    std::vector<Eigen::Vector3d> pts = points;
    if (sonar_mode_) {sonarElevations(pts, X_pred);}
    const Mat6 prior = (P + 1e-9 * Mat6::Identity()).inverse();
    auto r = matcher_->match(pts, X_pred, &prior, positions, depths, attitudes);
    r.score = fitScore(r.score, r.pose, scan_);
    // A good fit far from what the odometry predicted is slipping or a jump: trust the scan
    const double d2 = r.correction.transpose() * prior * r.correction;
    const bool moving = U.translation().norm() > 0.02 || logSO3(U.linear()).norm() > 0.02;
    if (moving && judged(pts) && r.score < doubtScore() && d2 < slip_sigmas_ * slip_sigmas_) {
      Mat6 wide = P * 25.0;
      const Mat6 wide_prior = (wide + 1e-9 * Mat6::Identity()).inverse();
      auto r2 = matcher_->match(pts, X_pred, &wide_prior, positions, depths, attitudes);
      r2.score = fitScore(r2.score, r2.pose, scan_);
      if (r2.score > r.score + 0.1) {
        RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "OrbitLocalizer: the scan fits %.2f at %.1f m from the odometry's prediction "
          "(%.2f there): slip or jump", r2.score,
          (r2.pose.translation() - r.pose.translation()).norm(), r.score);
        // The place the odometry predicted stays as a lighter hypothesis, in case the scan
        // fitted somewhere else that merely looks alike
        Hypothesis stay = h;
        stay.X_mo = r.pose * X_ob.inverse();
        stay.P = r.cov;
        stay.X_ob_at = X_ob;
        stay.score = r.score;
        stay.weight = 0.3 * h.weight;
        stay.id = next_id_++;
        stay.best_since = -1.0;
        left_behind.push_back(stay);
        r = r2;
      }
    }
    Pose X = r.pose;
    placeOnTerrain(nav_state, X, R_imu);
    h.X_mo = X * X_ob.inverse();
    h.P = r.cov;
    for (int k = 0; k < 6; ++k) {
      h.P(k, k) = std::max(h.P(k, k), 1e-8);
    }
    h.X_ob_at = X_ob;
    h.score = pts.empty() ? h.score : r.score;
    h.bad_cycles = (judged(pts) && r.score < doubtScore()) ? h.bad_cycles + 1 : 0;
  }
  for (auto & h : left_behind) {
    if (static_cast<int>(hyps_.size()) < max_hypotheses_) {hyps_.push_back(h);}
  }
}

void OrbitLocalizer::updateWeights(double travelled)
{
  if (hyps_.empty()) {return;}
  // The estimate earns credit with every update it fits and spends it with every one it does
  // not: it holds until it has stopped fitting for as long as it fitted (capped). A stretch of
  // the map that lies, a crowd, a glass wall, cannot hand the estimate to a place that happens
  // to fit the scan: the odometry's continuity speaks for the estimate while its credit lasts
  (void)travelled;
  // It spends faster (up to three more units an update, by the gap) when the hypothesis that
  // leads by weight, and has led for min_winner_time, fits clearly better: twins tied through a
  // corridor earn credit alike, and the first wall that tells them apart must decide in a
  // metre, not in the whole length of the corridor. A single update that scores higher
  // elsewhere (the noise of open ground) does not spend it
  const double now = get_node()->now().seconds();
  const Hypothesis * leader = nullptr;
  for (const auto & o : hyps_) {
    if (o.id != winner_id_ && (!leader || o.weight > leader->weight)) {leader = &o;}
  }
  for (auto & h : hyps_) {
    if (h.id != winner_id_) {continue;}
    double beaten = 0.0;
    if (leader && leader->weight > h.weight && leader->best_since >= 0.0 &&
      now - leader->best_since >= min_winner_time_)
    {
      beaten = std::max(0.0, leader->score - h.score - score_deadband_);
    }
    const double spent = (h.score < doubtScore() ? 1.0 : 0.0) +
      std::min(3.0, beaten / score_deadband_);
    h.credit = spent > 0.0 ? std::max(0.0, h.credit - spent) :
      std::min(credit_max_, h.credit + 1.0);
  }
  double best = 0.0;
  for (const auto & h : hyps_) {
    best = std::max(best, h.score);
  }
  // Only a difference the noise cannot explain counts (twins that see the same walls stay
  // even), and one update penalises by a bounded factor (a transient bad fit, a big step or a
  // passer-by, is not to decide between two places for good)
  double sum = 0.0;
  for (auto & h : hyps_) {
    const double gap = std::min(0.0, h.score - best + score_deadband_);
    h.weight *= std::exp(weight_sharpness_ * std::max(gap, -0.15));
    sum += h.weight;
  }
  if (sum <= 0.0) {
    for (auto & h : hyps_) {
      h.weight = 1.0 / hyps_.size();
    }
  } else {
    for (auto & h : hyps_) {
      h.weight /= sum;
    }
  }
  // Merge the ones at the same pose (the heavier survives), drop the light ones
  std::sort(
    hyps_.begin(), hyps_.end(), [](const Hypothesis & a, const Hypothesis & b) {
      return a.weight > b.weight;
    });
  const Pose X_ob = hyps_.front().X_ob_at;
  std::vector<Hypothesis> kept;
  for (const auto & h : hyps_) {
    bool dup = false;
    const Pose X = h.X_mo * X_ob;
    for (auto & k : kept) {
      const Pose Y = k.X_mo * X_ob;
      if ((X.translation() - Y.translation()).norm() < merge_distance_ &&
        std::abs(wrapAngle(yawOf(X) - yawOf(Y))) < merge_angle_)
      {
        k.weight += h.weight;
        if (h.id == winner_id_) {  // the estimate keeps its identity and its credit
          k.id = h.id;
          k.credit = std::max(k.credit, h.credit);
        }
        dup = true;
        break;
      }
    }
    if (!dup && (h.weight >= min_weight_ || h.credit > 0.0 || kept.empty())) {kept.push_back(h);}
  }
  if (static_cast<int>(kept.size()) > max_hypotheses_) {
    // The estimate stays whatever its weight: its credit is what keeps it
    std::stable_partition(
      kept.begin(), kept.end(), [this](const Hypothesis & h) {return h.id == winner_id_;});
    kept.resize(max_hypotheses_);
  }
  hyps_ = std::move(kept);
}

double OrbitLocalizer::doubtScore() const
{
  return usual_score_ < 0.0 ? lost_score_ : std::max(lost_score_, 0.6 * usual_score_);
}

bool OrbitLocalizer::judged(const std::vector<Eigen::Vector3d> & points) const
{
  // A handful of echoes (a sonar over a bare floor, a laser in the open) is not evidence of
  // anything: it neither counts towards the loss of track nor sets the usual score
  return static_cast<int>(points.size()) >= std::max(1, min_evidence_points_);
}

bool OrbitLocalizer::covered(const Pose & X_mb) const
{
  for (const auto & h : hyps_) {
    const Pose Y = h.X_mo * h.X_ob_at;
    if ((X_mb.translation() - Y.translation()).norm() < merge_distance_ &&
      std::abs(wrapAngle(yawOf(X_mb) - yawOf(Y))) < merge_angle_)
    {
      return true;
    }
  }
  return false;
}

void OrbitLocalizer::spawnTwins(const Pose & X_ob)
{
  if (!graph_ready_) {return;}
  std::vector<Hypothesis> born;
  for (const auto & h : hyps_) {
    if (h.weight < 0.2) {continue;}
    for (const auto & t : graph_.twins(h.X_mo * X_ob)) {
      if (covered(t.pose)) {continue;}
      bool dup = false;
      for (const auto & b : born) {
        const Pose Y = b.X_mo * X_ob;
        if ((t.pose.translation() - Y.translation()).norm() < merge_distance_ &&
          std::abs(wrapAngle(yawOf(t.pose) - yawOf(Y))) < merge_angle_) {dup = true; break;}
      }
      if (dup) {continue;}
      Hypothesis n;
      n.X_mo = t.pose * X_ob.inverse();
      n.P = h.P;
      n.X_ob_at = X_ob;
      n.weight = spawn_weight_ * h.weight;
      n.score = h.score;
      n.id = next_id_++;
      born.push_back(n);
    }
  }
  for (auto & n : born) {
    if (static_cast<int>(hyps_.size()) >= max_hypotheses_) {break;}
    hyps_.push_back(n);
  }
}

void OrbitLocalizer::chooseWinner(bool allow_switch)
{
  if (hyps_.empty()) {return;}
  auto best = std::max_element(
    hyps_.begin(), hyps_.end(), [](const Hypothesis & a, const Hypothesis & b) {
      return a.weight < b.weight || (a.weight == b.weight && a.score < b.score);
    });
  // ROS time (the bag's in a replay): the hysteresis is in the robot's time, not the computer's
  const double now = get_node()->now().seconds();
  for (auto & h : hyps_) {
    if (&h == &*best) {
      if (h.best_since < 0.0) {h.best_since = now;}
    } else {
      h.best_since = -1.0;
    }
  }
  auto current = std::find_if(
    hyps_.begin(), hyps_.end(), [this](const Hypothesis & h) {return h.id == winner_id_;});
  // The best replaces the current winner once it has led for min_winner_time (no flapping)
  // The best by weight replaces the current winner once it has led for min_winner_time (no
  // flapping) and fits the present scan at least as well: the weight is history, the score is
  // now, and a place that explains less of what is seen right now is not the estimate
  if (current == hyps_.end() || best->id == winner_id_ ||
    (allow_switch && current->credit <= 0.0 && now - best->best_since >= min_winner_time_ &&
    best->score + 0.05 >= current->score))
  {
    current = best;
  }
  if (current->id != winner_id_ && winner_id_ >= 0) {
    const auto prev = std::find_if(
      hyps_.begin(), hyps_.end(), [this](const Hypothesis & h) {return h.id == winner_id_;});
    RCLCPP_INFO(
      get_node()->get_logger(),
      "OrbitLocalizer: hypothesis %d (score %.2f, weight %.2f) replaces %d (%s) as the "
      "estimate, %.1f m away", current->id, current->score, current->weight, winner_id_,
      (prev == hyps_.end() ? std::string("gone") : "score " +
      std::to_string(prev->score).substr(0, 4) + ", weight " +
      std::to_string(prev->weight).substr(0, 4)).c_str(),
      prev == hyps_.end() ? 0.0 :
      (current->X_mo.translation() - prev->X_mo.translation()).norm());
    std::string all;
    for (const auto & h : hyps_) {
      const Pose X = h.X_mo * h.X_ob_at;
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), " %d:(%.1f,%.1f,%.2f) s%.2f w%.2f", h.id, X.translation().x(),
        X.translation().y(), yawOf(X), h.score, h.weight);
      all += buf;
    }
    RCLCPP_INFO(get_node()->get_logger(), "OrbitLocalizer: hypotheses%s", all.c_str());
  }
  winner_id_ = current->id;
  winner_X_mo_ = current->X_mo;
  winner_P_ = current->P;
  winner_score_ = current->score;
  have_estimate_ = true;
  double second = 0.0;
  for (const auto & h : hyps_) {
    if (h.id != best->id) {second = std::max(second, h.weight);}
  }
  ambiguity_ = best->weight > 0.0 ? second / best->weight : 0.0;
}

void OrbitLocalizer::relocalize(
  NavState & nav_state, const std::vector<Eigen::Vector3d> & points,
  const std::vector<Eigen::Vector3d> & scan, const Pose & X_ob)
{
  if (!graph_ready_ || points.empty()) {return;}
  // The scan's view (the whole downsampled cloud) finds the places; the matcher refines there
  auto cands = graph_.globalMatch(
    *views_, *matcher_, points, reloc_candidates_, reloc_yaw_seeds_, reloc_min_score_,
    raw_scan_.empty() ? &scan : &raw_scan_, coverageBand());
  if (std::getenv("ORBIT_TRACE") && nav_state.has("debug.truth")) {
    // Where the robot really is (a replay knows): how the search sees that place
    const auto & tp = nav_state.get<geometry_msgs::msg::Pose>("debug.truth");
    const Pose X_t = fromXYZYaw(tp.position.x, tp.position.y, tp.position.z,
      2.0 * std::atan2(tp.orientation.z, tp.orientation.w));
    const View view = views_->fromScan(raw_scan_.empty() ? scan : raw_scan_);
    float dist = 0.0f;
    const int rank = graph_.descriptorRank(view.descriptor, X_t.translation(), &dist);
    const double fwd = matcher_->score(points, X_t);
    if (const auto * n = graph_.nearest(X_t.translation())) {
      // Both descriptors, as marginals over range (16) and height (8) plus the no-return term
      auto marg = [](const std::vector<float> & d) {
          std::string r, h;
          const int R = 16, H = static_cast<int>((d.size() - 1) / 16);
          for (int i = 0; i < R; ++i) {
            float v = 0.0f;
            for (int j = 0; j < H; ++j) {
              v += d[j * R + i];
            }
            r += ' ' + std::to_string(v).substr(0, 4);
          }
          for (int j = 0; j < H; ++j) {
            float v = 0.0f;
            for (int i = 0; i < R; ++i) {
              v += d[j * R + i];
            }
            h += ' ' + std::to_string(v).substr(0, 4);
          }
          return "range[" + r + " ] height[" + h + " ] noret " +
                 std::to_string(d.back()).substr(0, 4);
        };
      RCLCPP_INFO(
        get_node()->get_logger(),
        "OrbitLocalizer: scan view %zu pts %s\n  node view (%.1f, %.1f, %.1f) %s",
        view.points.size(), marg(view.descriptor).c_str(), n->position.x(), n->position.y(),
        n->position.z(), marg(n->descriptor).c_str());
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "OrbitLocalizer: truth (%.1f, %.1f, %.2f): descriptor rank "
      "%d (distance %.3f), forward %.2f, two-sided %.2f; %zu candidates", X_t.translation().x(),
      X_t.translation().y(), yawOf(X_t), rank, dist, fwd,
      fitScore(fwd, X_t, scan.empty() ? points : scan),
      cands.size());
  }
  // The candidates come with the two-sided score. A place that fits worse than "localized"
  // usually looks is not a place the robot can be: the search only answers with credible ones
  const double credible = std::max(reloc_min_score_, 0.75 * usual_score_);
  cands.erase(
    std::remove_if(
      cands.begin(), cands.end(), [credible](const Candidate & c) {return c.score < credible;}),
    cands.end());
  if (cands.empty()) {return;}
  std::string list;
  for (std::size_t k = 0; k < std::min<std::size_t>(cands.size(), 4); ++k) {
    char buf[64];
    std::snprintf(
      buf, sizeof(buf), " %.2f@(%.1f,%.1f,%.2f)", cands[k].score,
      cands[k].pose.translation().x(), cands[k].pose.translation().y(), yawOf(cands[k].pose));
    list += buf;
  }
  RCLCPP_WARN(
    get_node()->get_logger(), "OrbitLocalizer: t=%.1f lost (score %.2f, usual %.2f, was at "
    "(%.1f, %.1f)); %zu places fit the scan:%s", get_node()->now().seconds(), winner_score_,
    usual_score_,
    (winner_X_mo_ * X_ob).translation().x(), (winner_X_mo_ * X_ob).translation().y(),
    cands.size(), list.c_str());
  Mat6 P = Mat6::Zero();
  P.diagonal() << 0.25, 0.25, 0.25, 0.01, 0.01, 0.1;
  // The places the scan fits join the set; the current winner and the strongest hypotheses
  // stay (a dip of the score is not a proof of being elsewhere): the robot's motion decides,
  // through the weights. Half the slots are made free for the newcomers.
  std::stable_sort(
    hyps_.begin(), hyps_.end(), [this](const Hypothesis & a, const Hypothesis & b) {
      if ((a.id == winner_id_) != (b.id == winner_id_)) {return a.id == winner_id_;}
      return a.weight > b.weight;
    });
  const std::size_t keep = static_cast<std::size_t>(std::max(1, max_hypotheses_ / 2));
  if (hyps_.size() > keep) {hyps_.resize(keep);}
  for (auto & h : hyps_) {
    h.weight = std::min(h.weight, 0.5);
  }
  for (const auto & c : cands) {
    if (static_cast<int>(hyps_.size()) >= max_hypotheses_) {break;}
    if (covered(c.pose)) {continue;}
    Hypothesis h;
    h.X_mo = c.pose * X_ob.inverse();
    h.P = P;
    h.X_ob_at = X_ob;
    h.weight = c.score;
    h.score = c.score;
    h.id = next_id_++;
    hyps_.push_back(h);
  }
  // A place that fits clearly better than the estimate takes it now; otherwise the newcomers
  // earn it by moving. No new search until the robot has moved on (if asked so): the
  // candidates need the motion to be told apart
  bool spent = true;
  for (const auto & h : hyps_) {
    if (h.id == winner_id_) {spent = h.credit <= 0.0;}
  }
  chooseWinner(spent && cands.front().score >= winner_score_ + 0.3);
  reloc_X_ob_ = X_ob;
  have_reloc_ = true;
  lost_count_ = 0;
}

void OrbitLocalizer::update(NavState & nav_state)
{
  const double t0 = seconds();
  if (!ensureMap(nav_state)) {return;}
  Pose X_ob;
  bool odom_ok;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    X_ob = X_ob_;
    odom_ok = odom_ok_;
  }
  if (!odom_ok) {return;}

  std::lock_guard<std::mutex> lock(hyp_mutex_);
  if (pending_reset_) {
    Hypothesis h;
    h.X_mo = pending_reset_->first * X_ob.inverse();
    h.P = pending_reset_->second;
    h.X_ob_at = X_ob;
    h.weight = 1.0;
    h.id = next_id_++;
    hyps_.assign(1, h);
    winner_id_ = h.id;
    winner_X_mo_ = h.X_mo;
    winner_P_ = h.P;
    have_estimate_ = true;
    pending_reset_.reset();
    have_evidence_pose_ = false;   // the new pose is corrected at once, moving or not
    lost_count_ = 0;
    lost_ = false;
  }
  rclcpp::Time scan_stamp;
  const auto points = collectPoints(nav_state, scan_stamp, &scan_, &raw_scan_);
  if (!points.empty()) {X_ob = odomAt(scan_stamp);}
  if (hyps_.empty() || (global_init_ && !have_estimate_)) {
    if (!points.empty() && graph_ready_) {
      relocalize(nav_state, points, scan_, X_ob);
      global_init_ = false;
    }
    if (hyps_.empty()) {return;}
  }
  if (points.empty() && use_points_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "OrbitLocalizer: no points to correct with");
  }
  // Without odometry motion since the last correction the pose is not estimated again: a scan
  // taken again from the same place is the same evidence, not more of it, and a correction a
  // cycle on a standing robot is a random walk on whatever the scan leaves free (on a slope,
  // with the body rocking and the terrain placement, it ran 12 m uphill). The scan is only
  // judged where the estimate stands: not fitting, repeated, still does not fit, so a kidnapped
  // robot set down, or wheels frozen while it turns, reach the loss of track and the search
  const Vec6 moved = have_evidence_pose_ ? logSE3(evidence_X_ob_.inverse() * X_ob) : Vec6::Ones();
  const bool odom_still = have_evidence_pose_ && moved.head<3>().norm() < update_min_distance_ &&
    moved.tail<3>().norm() < update_min_angle_;
  if (odom_still) {
    spawnTwins(X_ob);
    if (!points.empty()) {
      Pose X = winner_X_mo_ * X_ob;   // with the attitude of now: the body rocks as it brakes
      placeOnTerrain(
        nav_state, X, (mode_ == Mode::PlanarTerrain) ? imuAttitude(nav_state) : std::nullopt);
      winner_score_ = fitScore(matcher_->score(points, X), X, scan_);
      for (auto & h : hyps_) {
        if (h.id == winner_id_) {h.score = winner_score_;}
      }
    }
    const bool doubtful = judged(points) && winner_score_ < doubtScore();
    lost_count_ = doubtful ? lost_count_ + 1 : 0;
    const bool was_lost = lost_;
    lost_ = lost_count_ >= lost_cycles_;
    if (lost_ && !was_lost) {
      const double since = have_reloc_ ?
        (reloc_X_ob_.inverse() * X_ob).translation().norm() : std::numeric_limits<double>::max();
      if (since >= reloc_min_travel_) {relocalize(nav_state, points, scan_, X_ob);}
    }
    last_cycle_ms_ = 1000.0 * (seconds() - t0);
    publish(nav_state);
    if (debug_markers_) {publishMarkers(X_ob);}
    return;
  }
  correctAll(nav_state, points, X_ob);
  const bool new_evidence = judged(points);   // a few echoes correct the pose but weigh nothing
  if (new_evidence) {
    evidence_X_ob_ = X_ob;
    have_evidence_pose_ = true;
    updateWeights(have_evidence_pose_ ? moved.head<3>().norm() : 0.0);
  }
  spawnTwins(X_ob);
  chooseWinner(new_evidence);
  if (new_evidence && std::getenv("ORBIT_TRACE")) {
    std::string all;
    for (const auto & h : hyps_) {
      const Pose X = h.X_mo * h.X_ob_at;
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), " %d:(%.2f,%.2f,%.2f) s%.2f w%.2f c%.0f", h.id, X.translation().x(),
        X.translation().y(), yawOf(X), h.score, h.weight, h.credit);
      all += buf;
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "OrbitLocalizer: trace winner %d, odom (%.2f,%.2f,%.2f):%s",
      winner_id_, X_ob.translation().x(), X_ob.translation().y(), yawOf(X_ob), all.c_str());
  }

  // Lost: the winner fails to explain the scans for a while; the map says where else it fits
  const bool doubtful = judged(points) && winner_score_ < doubtScore();
  if (judged(points) && !doubtful) {
    usual_score_ = usual_score_ < 0.0 ? winner_score_ :
      0.98 * usual_score_ + 0.02 * winner_score_;
  }
  lost_count_ = doubtful ? lost_count_ + 1 : 0;
  const bool was_lost = lost_;
  lost_ = lost_count_ >= lost_cycles_;
  if (lost_ && !was_lost) {
    const double since = have_reloc_ ?
      (reloc_X_ob_.inverse() * X_ob).translation().norm() : std::numeric_limits<double>::max();
    if (since >= reloc_min_travel_) {relocalize(nav_state, points, scan_, X_ob);}
  }
  last_cycle_ms_ = 1000.0 * (seconds() - t0);
  publish(nav_state);
  if (debug_markers_) {
    publishMarkers(X_ob);
    if (graph_ready_ && !graph_published_) {
      publishGraph();
      graph_published_ = true;
    }
  }
}

void OrbitLocalizer::publish(NavState & nav_state)
{
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const rclcpp::Time now = get_node()->now();
  auto odom = poseMessage(winner_X_mo_ * hyps_.front().X_ob_at, winner_P_);
  odom.header.stamp = now;
  nav_state.set("robot_pose", odom);
  nav_state.set("localizer.orbit.match_score", winner_score_);
  nav_state.set("localizer.orbit.hypotheses", static_cast<int>(hyps_.size()));
  nav_state.set("localizer.orbit.ambiguity", ambiguity_);
  nav_state.set("localizer.orbit.lost", lost_.load());
  nav_state.set("localizer.orbit.cycle_ms", last_cycle_ms_);
  nav_state.set("localizer.orbit.covariance_trace", odom.pose.covariance[0] +
    odom.pose.covariance[7] + odom.pose.covariance[35]);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = tf_info.map_frame;
  pose.pose = odom.pose;
  pose_pub_->publish(pose);
  geometry_msgs::msg::PoseArray arr;
  arr.header = pose.header;
  for (const auto & h : hyps_) {
    arr.poses.push_back(toMsg(h.X_mo * h.X_ob_at));
  }
  hyps_pub_->publish(arr);
}

void OrbitLocalizer::publishMarkers(const Pose & X_ob)
{
  using visualization_msgs::msg::Marker;
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  visualization_msgs::msg::MarkerArray arr;
  Marker base;
  base.header.stamp = get_node()->now();
  base.header.frame_id = tf_info.map_frame;
  base.action = Marker::ADD;
  base.pose.orientation.w = 1.0;
  Marker clear = base;
  clear.action = Marker::DELETEALL;
  arr.markers.push_back(clear);

  // Hypotheses: an arrow each, the estimate in green, the others orange fading with the weight
  double winner_credit = 0.0;
  int i = 0;
  for (const auto & h : hyps_) {
    const bool winner = h.id == winner_id_;
    if (winner) {winner_credit = h.credit;}
    Marker m = base;
    m.ns = "hypotheses";
    m.id = i;
    m.type = Marker::ARROW;
    m.pose = toMsg(h.X_mo * h.X_ob_at);
    m.scale.x = winner ? 0.6 : 0.4;
    m.scale.y = winner ? 0.08 : 0.05;
    m.scale.z = m.scale.y;
    m.color.r = winner ? 0.1f : 1.0f;
    m.color.g = winner ? 0.8f : 0.5f;
    m.color.b = 0.1f;
    m.color.a = winner ? 1.0f : static_cast<float>(0.3 + 0.7 * std::clamp(h.weight, 0.0, 1.0));
    arr.markers.push_back(m);
    Marker t = base;
    t.ns = "labels";
    t.id = i++;
    t.type = Marker::TEXT_VIEW_FACING;
    t.pose = m.pose;
    t.pose.position.z += 0.3;
    t.scale.z = 0.15;
    t.color = m.color;
    t.color.a = 1.0f;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "#%d w%.2f s%.2f c%.0f", h.id, h.weight, h.score, h.credit);
    t.text = buf;
    arr.markers.push_back(t);
  }

  // The scan where the estimate puts it: green the points the map explains, red the rest
  const Pose X_mb = winner_X_mo_ * X_ob;
  Marker pts = base;
  pts.ns = "scan";
  pts.id = 0;
  pts.type = Marker::POINTS;
  pts.scale.x = 0.06;
  pts.scale.y = 0.06;
  for (const auto & p : scan_) {
    const Eigen::Vector3d q = X_mb * p;
    const bool inlier = field_ && field_->distance(q, nullptr) <= match_options_.inlier_distance;
    geometry_msgs::msg::Point g;
    g.x = q.x();
    g.y = q.y();
    g.z = q.z();
    std_msgs::msg::ColorRGBA c;
    c.r = inlier ? 0.2f : 1.0f;
    c.g = inlier ? 0.9f : 0.2f;
    c.b = 0.2f;
    c.a = 1.0f;
    pts.points.push_back(g);
    pts.colors.push_back(c);
  }
  if (!pts.points.empty()) {arr.markers.push_back(pts);}

  // Status over the robot
  const bool lost = lost_.load();
  Marker s = base;
  s.ns = "status";
  s.id = 0;
  s.type = Marker::TEXT_VIEW_FACING;
  s.pose = toMsg(X_mb);
  s.pose.position.z += 0.8;
  s.scale.z = 0.25;
  s.color.r = lost ? 1.0f : 0.9f;
  s.color.g = lost ? 0.2f : 0.9f;
  s.color.b = lost ? 0.2f : 0.9f;
  s.color.a = 1.0f;
  char buf[128];
  std::snprintf(
    buf, sizeof(buf), "ORBIT %s  score %.2f  credit %.0f  %zu hyp  %.1f ms",
    lost ? "LOST" : "ok", winner_score_, winner_credit, hyps_.size(), last_cycle_ms_);
  s.text = buf;
  arr.markers.push_back(s);
  markers_pub_->publish(arr);
}

void OrbitLocalizer::publishGraph()
{
  using visualization_msgs::msg::Marker;
  const auto tf_info = RTTFBuffer::getInstance()->get_tf_info();
  visualization_msgs::msg::MarkerArray arr;
  Marker places;
  places.header.stamp = get_node()->now();
  places.header.frame_id = tf_info.map_frame;
  places.action = Marker::ADD;
  places.pose.orientation.w = 1.0;
  places.ns = "places";
  places.id = 0;
  places.type = Marker::POINTS;
  places.scale.x = 0.08;
  places.scale.y = 0.08;
  places.color.r = 0.2f;
  places.color.g = 0.7f;
  places.color.b = 1.0f;
  places.color.a = 0.8f;
  Marker links = places;   // a line from each place to the places that look the same
  links.ns = "symmetries";
  links.type = Marker::LINE_LIST;
  links.scale.x = 0.02;
  links.color.r = 1.0f;
  links.color.g = 0.3f;
  links.color.b = 0.9f;
  links.color.a = 0.5f;
  for (const auto & n : graph_.nodes()) {
    geometry_msgs::msg::Point g;
    g.x = n.position.x();
    g.y = n.position.y();
    g.z = n.position.z() + 0.05;
    places.points.push_back(g);
    const Pose X_n = fromXYZYaw(n.position.x(), n.position.y(), n.position.z(), 0.0);
    for (const auto & a : n.alternatives) {
      const Eigen::Vector3d q = (X_n * a.T_node_alt).translation();
      geometry_msgs::msg::Point e;
      e.x = q.x();
      e.y = q.y();
      e.z = q.z() + 0.05;
      links.points.push_back(g);
      links.points.push_back(e);
    }
  }
  if (!places.points.empty()) {arr.markers.push_back(places);}
  if (!links.points.empty()) {arr.markers.push_back(links);}
  graph_pub_->publish(arr);
}

std::vector<Hypothesis> OrbitLocalizer::hypotheses() const
{
  std::lock_guard<std::mutex> lock(hyp_mutex_);
  auto out = hyps_;
  std::sort(
    out.begin(), out.end(), [](const Hypothesis & a, const Hypothesis & b) {
      return a.weight > b.weight;
    });
  return out;
}

Pose OrbitLocalizer::estimate() const
{
  Pose X_ob;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    X_ob = X_ob_;
  }
  std::lock_guard<std::mutex> lock(hyp_mutex_);
  return winner_X_mo_ * X_ob;
}

}  // namespace orbit
}  // namespace easynav

PLUGINLIB_EXPORT_CLASS(easynav::orbit::OrbitLocalizer, easynav::LocalizerMethodBase)
