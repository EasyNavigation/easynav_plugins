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

#include "easynav_bigmap_maps_manager/BigMapMapsManager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#if defined(__GLIBC__)
#include <malloc.h>
#endif
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "easynav_common/PackageShare.hpp"
#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/header.hpp"

namespace easynav
{
namespace bigmap
{

BigMapMapsManager::BigMapMapsManager()
{
  filters_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::navmap::NavMapFilter>>(
    "easynav_navmap_maps_manager", "easynav::navmap::NavMapFilter");
}

BigMapMapsManager::~BigMapMapsManager() = default;

void BigMapMapsManager::on_initialize()
{
  auto node = get_node();
  const std::string & name = get_plugin_name();
  auto param = [&](const std::string & key, auto value) {
      easynav::declare_parameter_if_absent(*node, name + "." + key, value);
      node->get_parameter(name + "." + key, value);
      return value;
    };

  // The map
  const std::string package = param("package", std::string());
  std::string path = param("navmap_path_file", std::string());
  if (path.empty()) {
    throw std::runtime_error("[" + name + "] navmap_path_file is not set");
  }
  if (!package.empty()) {
    path = (std::filesystem::path(easynav::get_package_share_path(package)) / path).string();
  }
  ::navmap::NavMap source;
  std::error_code ec;
  if (!navmap_ros::io::load_from_file(path, source, &ec)) {
    throw std::runtime_error("[" + name + "] cannot read " + path + ": " + ec.message());
  }

  // Its levels of detail
  LodParams lp;
  lp.cell_min = param("cell_size.min", lp.cell_min);
  lp.cell_max = param("cell_size.max", lp.cell_max);
  lp.min_radius = param("min_radius", lp.min_radius);
  lp.max_radius = param("max_radius", lp.max_radius);
  forget_distance_ = param("forget_distance", -1.0);
  if (forget_distance_ <= 0.0) {forget_distance_ = lp.max_radius;}
  min_height_ = param("obstacle_min_height", min_height_);
  min_height_per_m_ = param("obstacle_min_height_per_meter", min_height_per_m_);
  max_height_ = param("obstacle_max_height", max_height_);
  sensor_range_ = param("sensor_range", -1.0);
  if (sensor_range_ <= 0.0) {sensor_range_ = lp.min_radius;}
  sensor_range_ = std::min(sensor_range_, forget_distance_);
  rebuild_period_ = param("rebuild_period", rebuild_period_);
  // The memory of sensed obstacles, as the NavMap maps manager's obstacle filter
  MemoryParams mp;
  mp.clearing = param("memory.clearing", mp.clearing);
  mp.decay_time = std::max(0.0, param("memory.decay_time", mp.decay_time));
  mp.p_hit = param("memory.p_hit", mp.p_hit);
  mp.p_miss = param("memory.p_miss", mp.p_miss);
  if (!(mp.p_hit > 0.5 && mp.p_hit < 1.0) || !(mp.p_miss > 0.0 && mp.p_miss < 0.5)) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] memory.p_hit must be in (0.5, 1) and memory.p_miss in (0, 0.5): "
      "0.7 and 0.4", name.c_str());
    mp.p_hit = 0.7;
    mp.p_miss = 0.4;
  }

  // Built once, on every core: a startup step, not part of the cycle
  const std::size_t source_navcels = source.navcels.size();
  const auto st = lod_.build(
    std::move(source), lp, std::max(1u, std::thread::hardware_concurrency()));
#if defined(__GLIBC__)
  // the building threads leave most of what they used free but kept by the allocator
  malloc_trim(0);
#endif
  std::string levels;
  for (int k = 0; k < lod_.levels(); ++k) {
    char buf[96];
    std::snprintf(
      buf, sizeof(buf), "%s%.2f m from %.0f m: %zu NavCels", k ? "; " : "", lod_.cell(k),
      lod_.radius(k), st.navcels[k]);
    levels += buf;
  }
  RCLCPP_INFO(
    node->get_logger(), "[%s] %s: %zu NavCels, %zu tiles, in %.1f s. Levels: %s", name.c_str(),
    path.c_str(), source_navcels, st.tiles, st.seconds, levels.c_str());
  // As fine as the points are thinned: every NavCel an obstacle covers gets one
  sensed_ = SensedObstacles(lod_.cell(0) / 2.0, mp);
  known_ = static_cast<float>(lod_.cell(0));

  // Filters, as in the NavMap maps manager
  std::vector<std::string> filters;
  filters = param("filters", filters);
  for (const auto & f : filters) {
    const std::string plugin = param(f + ".plugin", std::string());
    auto instance = filters_loader_->createSharedInstance(plugin);
    instance->initialize(node, name + "." + f);
    filters_.push_back(instance);
    RCLCPP_INFO(node->get_logger(), "[%s] filter %s [%s]", name.c_str(), f.c_str(), plugin.c_str());
    // The detail around obstacles reaches as far as an inflation does
    const std::string radius = name + "." + f + ".inflation_radius";
    if (node->has_parameter(radius)) {
      margin_ = std::max(margin_, node->get_parameter(radius).as_double());
    }
  }

  const std::string base = node->get_node_base_interface()->get_fully_qualified_name() +
    std::string("/") + name;
  map_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    base + "/map", rclcpp::QoS(1).transient_local().reliable());
  layer_updates_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMapLayer>(
    base + "/map_updates", rclcpp::QoS(100));
  markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    base + "/markers", rclcpp::QoS(1).transient_local().reliable());
  stats_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    base + "/stats", rclcpp::QoS(10));
}

bool BigMapMapsManager::robot_pose(
  const ::easynav::NavState & nav_state, Eigen::Vector3d & p, double & yaw) const
{
  if (!nav_state.has("robot_pose")) {return false;}
  const auto & pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  p = {pose.position.x, pose.position.y, pose.position.z};
  const auto & q = pose.orientation;
  yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return p.allFinite() && std::isfinite(yaw);
}

bool BigMapMapsManager::sense(
  const ::easynav::NavState & nav_state, const Eigen::Vector3d & robot, double yaw)
{
  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  if (perceptions.empty() || !has_map_) {return false;}
  pcl::PointCloud<pcl::PointXYZ> cloud;
  try {
    // In the map frame, each perception with the robot's pose when it was taken (TF at its
    // stamp, exact_time): with the latest pose, a cloud taken while the robot turns lands rotated
    // (a cloud may be up to the sensors' forget_time old, and is used in every cycle until the
    // next one arrives), and walls seen a fraction of a second ago fall on free ground as
    // obstacles. Cut to the box around the robot first (heights from its ground, the TF being
    // flat), then thinned.
    const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
    const double r = sensor_range_;
    PointPerceptionsOpsView view(perceptions);
    view.fuse(tf_info.map_frame, true);
    view.filter(
      {robot.x() - r, robot.y() - r, robot.z() + min_height_ - 1.0},
      {robot.x() + r, robot.y() + r, robot.z() + max_height_ + 1.0}, false);
    view.downsample(lod_.cell(0) / 2.0);
    cloud = view.as_points();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000, "[%s] perceptions: %s",
      get_plugin_name().c_str(), e.what());
    return false;
  }
  // The ground under the robot, on the map, and its tilt: the robot stands on it, so on a ramp the
  // sensor is tilted like the ramp. With a flat TF (2D odometry) the ramp ahead comes at the
  // robot's own height (going down a ramp, it would otherwise be taken for an obstacle above the
  // ramp's lower part): each point is turned about the robot by what TF lacks of that tilt
  // (nothing, if the robot's pose in TF has it already, e.g. a 3D localization)
  std::size_t s;
  ::navmap::NavCelId cid;
  Eigen::Vector3f bary, hit;
  Eigen::Vector3f ground(
    static_cast<float>(robot.x()), static_cast<float>(robot.y()), static_cast<float>(robot.z()));
  Eigen::Quaternionf tilt = Eigen::Quaternionf::Identity();
  if (map_.locate_navcel(robot.cast<float>(), s, cid, bary, &hit)) {
    ground.z() = hit.z();
    const auto & v = map_.navcels[cid].v;
    const Eigen::Vector3f a = map_.positions.at(v[0]), b = map_.positions.at(v[1]),
      d = map_.positions.at(v[2]);
    Eigen::Vector3f n = (b - a).cross(d - a);
    if (n.norm() > 1e-9f) {
      n.normalize();
      if (n.z() < 0.0f) {n = -n;}
      Eigen::Vector3f up = Eigen::Vector3f::UnitZ();   // the robot's up in TF
      try {
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        const auto t = RTTFBuffer::getInstance()->lookupTransform(
          tf_info.map_frame, tf_info.robot_footprint_frame, tf2::TimePointZero);
        const auto & r = t.transform.rotation;
        up = Eigen::Quaternionf(
          static_cast<float>(r.w), static_cast<float>(r.x), static_cast<float>(r.y),
          static_cast<float>(r.z)) * Eigen::Vector3f::UnitZ();
      } catch (const std::exception &) {
      }
      tilt = Eigen::Quaternionf::FromTwoVectors(up, n);
    }
  }
  (void)yaw;   // the heading comes with each perception's TF
  const Eigen::Vector3f at = robot.cast<float>();
  // The evidence of this cycle: the obstacles (on the ground under them), and a ray from the
  // robot to every point on the ground or on an obstacle (what it crosses is free)
  std::vector<Eigen::Vector3f> hits;
  std::vector<SensedObstacles::Ray> rays;
  rays.reserve(cloud.points.size());
  for (const auto & pt : cloud.points) {
    const Eigen::Vector3f o = Eigen::Vector3f(pt.x, pt.y, pt.z) - at;   // from the robot
    const Eigen::Vector3f q = ground + tilt * o;
    // An obstacle is above the map's surface: over no surface (a hole, a building) it is
    // the map's own
    if (!map_.locate_navcel(q, s, cid, bary, &hit)) {continue;}
    // the farther, the higher a point must be: a slight tilt of the robot or the sensor puts the
    // far ground above it
    const float h = q.z() - hit.z();
    const double range = std::hypot(o.x(), o.y());
    if (h > max_height_) {continue;}   // over the robot: neither an obstacle nor free ground
    if (h < min_height_ + min_height_per_m_ * range) {
      rays.push_back({hit, true});   // the ground: free up to it, and it too
      continue;
    }
    rays.push_back({hit, false});
    // Next to a place with no ground (within cell_min: what the map and the localization are
    // sure of) it is the map's own obstacle, seen from the side
    bool known = false;
    for (int k = 0; k < 8 && !known; ++k) {
      const float a = static_cast<float>(k * M_PI / 4.0);
      const Eigen::Vector3f n(hit.x() + known_ * std::cos(a), hit.y() + known_ * std::sin(a),
        hit.z() + 0.2f);
      Eigen::Vector3f nb, nh;
      known = !map_.locate_navcel(n, s, cid, nb, &nh);
    }
    if (known) {continue;}
    hits.push_back(hit);
  }
  return sensed_.update(hits, ground, rays, get_node()->now().seconds());
}

void BigMapMapsManager::rebuild(
  const std::vector<std::uint32_t> & tiles, const Eigen::Vector3d & robot)
{
  const auto t0 = std::chrono::steady_clock::now();
  map_ = lod_.assemble(tiles, false);   // detail() builds the accelerations
  const auto t1 = std::chrono::steady_clock::now();
  lod_.detail(map_, robot.x(), robot.y(), sensed_.points(), margin_);
  const auto t2 = std::chrono::steady_clock::now();
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 10000,
    "[%s] map around (%.1f, %.1f): %zu NavCels from %zu tiles, %zu sensed obstacles; "
    "assembled in %.0f ms, detailed in %.0f ms", get_plugin_name().c_str(), robot.x(), robot.y(),
    map_.navcels.size(), tiles.size(), sensed_.size(),
    std::chrono::duration<double, std::milli>(t1 - t0).count(),
    std::chrono::duration<double, std::milli>(t2 - t1).count());
  stats_[0] += 1.0;
  stats_[1] = select_ms_;
  stats_[2] = std::chrono::duration<double, std::milli>(t1 - t0).count();
  stats_[3] = std::chrono::duration<double, std::milli>(t2 - t1).count();
  stats_[4] = static_cast<double>(map_.navcels.size());
  tiles_ = tiles;
  built_at_ = robot.head<2>();
  built_z_ = robot.z();
  has_map_ = true;
  map_changed_ = true;
  publish_pending_ = true;
}

void BigMapMapsManager::publish(const ::navmap::NavMap & nm)
{
  std_msgs::msg::Header header;
  header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  header.stamp = get_node()->now();
  auto msg = navmap_ros::to_msg(nm);
  msg.header = header;
  map_pub_->publish(msg);

  // Markers: a circle where each level starts (with its cell size) and the tiles in use, by level
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(clear);
  const int levels = lod_.levels();
  auto color = [levels](int k, visualization_msgs::msg::Marker & m) {
      const float t = levels > 1 ? static_cast<float>(k) / (levels - 1) : 0.0f;   // fine: red
      m.color.r = 1.0f - t;
      m.color.g = 0.4f + 0.4f * t;
      m.color.b = t;
      m.color.a = 1.0f;
    };
  const double z = std::isfinite(built_z_) ? built_z_ + 0.2 : 0.2;
  int id = 0;
  for (int k = 1; k < levels; ++k) {
    visualization_msgs::msg::Marker c;
    c.header = header;
    c.ns = "level radii";
    c.id = id++;
    c.type = visualization_msgs::msg::Marker::LINE_STRIP;
    c.scale.x = 0.3;
    c.pose.orientation.w = 1.0;
    color(k, c);
    const double r = lod_.radius(k);
    for (int i = 0; i <= 120; ++i) {
      geometry_msgs::msg::Point p;
      p.x = built_at_.x() + r * std::cos(i * 2.0 * M_PI / 120.0);
      p.y = built_at_.y() + r * std::sin(i * 2.0 * M_PI / 120.0);
      p.z = z;
      c.points.push_back(p);
    }
    ma.markers.push_back(c);
    visualization_msgs::msg::Marker t;
    t.header = header;
    t.ns = "level radii";
    t.id = id++;
    t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    t.pose.position.x = built_at_.x();
    t.pose.position.y = built_at_.y() + r;
    t.pose.position.z = z + 1.0;
    t.pose.orientation.w = 1.0;
    t.scale.z = 2.0;
    color(k, t);
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.1f m cells from %.0f m", lod_.cell(k), r);
    t.text = buf;
    ma.markers.push_back(t);
  }
  std::vector<visualization_msgs::msg::Marker> tiles(levels);
  for (int k = 0; k < levels; ++k) {
    auto & m = tiles[k];
    m.header = header;
    m.ns = "tiles";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.scale.x = 0.1;
    m.pose.orientation.w = 1.0;
    color(k, m);
    m.color.a = 0.6f;
  }
  for (const auto t : tiles_) {
    const auto b = lod_.bounds(t);
    auto & m = tiles[lod_.level(t)];
    const double xs[4] = {b[0], b[1], b[1], b[0]}, ys[4] = {b[2], b[2], b[3], b[3]};
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::msg::Point a, c;
      a.x = xs[i];
      a.y = ys[i];
      c.x = xs[(i + 1) % 4];
      c.y = ys[(i + 1) % 4];
      a.z = c.z = z;
      m.points.push_back(a);
      m.points.push_back(c);
    }
  }
  for (auto & m : tiles) {
    if (!m.points.empty()) {ma.markers.push_back(std::move(m));}
  }
  markers_pub_->publish(ma);
}

void BigMapMapsManager::update(::easynav::NavState & nav_state)
{
  if (lod_.empty()) {return;}
  const double now = get_node()->now().seconds();
  Eigen::Vector3d robot;
  double yaw = 0.0;
  const bool located = robot_pose(nav_state, robot, yaw);
  if (!located) {
    if (has_map_) {robot << built_at_, 0.0;} else {robot.setZero();}
  }

  using Ms = std::chrono::duration<double, std::milli>;
  const auto ts = std::chrono::steady_clock::now();
  if (located) {
    sensed_changed_ |= sense(nav_state, robot, yaw);
    sensed_changed_ |= sensed_.forget_beyond(robot.x(), robot.y(), forget_distance_) > 0;
  }

  // The map again when the tiles change, the robot has gone a few metres (the detail moves with
  // it) or the sensed obstacles change; not more often than rebuild_period_
  stats_[5] = Ms(std::chrono::steady_clock::now() - ts).count();
  const auto tsel = std::chrono::steady_clock::now();
  const auto tiles = lod_.select(robot.x(), robot.y());
  select_ms_ = Ms(std::chrono::steady_clock::now() - tsel).count();
  const bool moved = (robot.head<2>() - built_at_).norm() > lod_.params().min_radius / 4.0;
  if (!has_map_ ||
    ((tiles != tiles_ || moved || sensed_changed_) && now - last_build_ >= rebuild_period_))
  {
    rebuild(tiles, robot);
    sensed_changed_ = false;
    last_build_ = now;
  }
  if (map_changed_) {
    nav_state.set("map.navmap", map_);
    map_changed_ = false;
  }
  // Filters update the map in NavState in place, and record which cells they change
  const std::size_t subscribers = layer_updates_pub_->get_subscription_count();
  const bool new_subscriber = subscribers > last_subscribers_;
  last_subscribers_ = subscribers;
  std_msgs::msg::Header header;
  header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  header.stamp = get_node()->now();
  const auto tf = std::chrono::steady_clock::now();
  for (const auto & filter : filters_) {
    filter->set_map_resolution(static_cast<float>(lod_.cell(0)));
    filter->update(nav_state);
    // A map not published yet carries its layers itself: an update now would be of a geometry
    // the subscribers do not have yet
    if (subscribers == 0 || publish_pending_ || !filter->is_adding_layer()) {continue;}
    const auto & nm = nav_state.get<::navmap::NavMap>("map.navmap");
    const auto layer = nm.layers.get(filter->get_layer_name());
    if (!layer || (!layer->has_changes() && !new_subscriber)) {continue;}
    layer_updates_pub_->publish(
      navmap_ros::to_layer_update_msg(
        nm, filter->get_layer_name(), header, new_subscriber ? 0.0f : 0.25f));
  }
  stats_[6] = Ms(std::chrono::steady_clock::now() - tf).count();
  nav_state.set("map.bigmap_stats", stats_);
  if (stats_pub_->get_subscription_count() > 0) {
    // [time (s), rebuilds, select, assemble, detail (ms), NavCels, sensing, filters (ms),
    //  sensed obstacles]
    std_msgs::msg::Float64MultiArray m;
    m.data.reserve(stats_.size() + 2);
    m.data.push_back(now);
    m.data.insert(m.data.end(), stats_.begin(), stats_.end());
    m.data.push_back(static_cast<double>(sensed_.size()));
    stats_pub_->publish(m);
  }
  if (nav_state.has("map.navmap")) {
    const auto & nm = nav_state.get<::navmap::NavMap>("map.navmap");
    // the map as the filters left it (with their layers, e.g. inflated_obstacles)
    if (publish_pending_ && now - last_publish_ >= publish_period_) {
      publish(nm);
      publish_pending_ = false;
      last_publish_ = now;
    }
    for (const auto & l : nm.list_layers()) {
      nm.layers.get(l)->clear_changes();
    }
  }
}

}  // namespace bigmap
}  // namespace easynav

PLUGINLIB_EXPORT_CLASS(easynav::bigmap::BigMapMapsManager, easynav::MapsManagerBase)
