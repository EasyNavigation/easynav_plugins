// Copyright 2025 Intelligent Robotics Lab
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

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "easynav_common/Parameters.hpp"
#include "easynav_navmap_maps_manager/NavMapMapsManager.hpp"
#include "Eigen/Geometry"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_common/YTSession.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "easynav_navmap_maps_manager/map_io.hpp"

#include "easynav_common/PackageShare.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{
namespace navmap
{

using std::placeholders::_1;

NavMapMapsManager::NavMapMapsManager()
{
  ::easynav::NavState::register_printer<::navmap::NavMap>(
    [](const ::navmap::NavMap & map) {
      std::ostringstream oss;
      oss << "NavMap: navcels=" << map.navcels.size()
          << " surfaces=" << map.surfaces.size()
          << " layers=" << map.list_layers().size() << " (";

      for (const auto & layer : map.list_layers()) {
        oss << layer << " ";
      }
      oss << ")";

      return oss.str();
    });

  navmap_filters_loader_ = std::make_unique<pluginlib::ClassLoader<NavMapFilter>>(
    "easynav_navmap_maps_manager", "easynav::navmap::NavMapFilter");
}

NavMapMapsManager::~NavMapMapsManager() {}

void
NavMapMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, occmap_path_file, navmap_path_file;
  easynav::declare_parameter_if_absent(*node, plugin_name + ".package", package_name);
  easynav::declare_parameter_if_absent(*node, plugin_name + ".occmap_path_file", occmap_path_file);
  easynav::declare_parameter_if_absent(*node, plugin_name + ".navmap_path_file", navmap_path_file);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".occmap_path_file", occmap_path_file);
  node->get_parameter(plugin_name + ".navmap_path_file", navmap_path_file);

  // Local mode
  {
    const std::string L = plugin_name + ".local.";
    auto & b = local_.build;
    easynav::declare_parameter_if_absent(*node, L + "enabled", local_.enabled);
    easynav::declare_parameter_if_absent(*node, L + "frame", local_.frame);
    easynav::declare_parameter_if_absent(*node, L + "range", local_.range);
    easynav::declare_parameter_if_absent(*node, L + "min_z", local_.min_z);
    easynav::declare_parameter_if_absent(*node, L + "max_z", local_.max_z);
    easynav::declare_parameter_if_absent(*node, L + "accumulate_time", local_.accumulate_time);
    easynav::declare_parameter_if_absent(*node, L + "freq", local_.freq);
    easynav::declare_parameter_if_absent(*node, L + "downsample", local_.downsample);
    double res = 0.3, max_dz = b.max_dz, slope = b.max_slope_deg, clearance = b.min_clearance;
    double min_area = 1.0, hole = b.max_hole_area;
    easynav::declare_parameter_if_absent(*node, L + "resolution", res);
    easynav::declare_parameter_if_absent(*node, L + "max_dz", max_dz);
    easynav::declare_parameter_if_absent(*node, L + "max_slope_deg", slope);
    easynav::declare_parameter_if_absent(*node, L + "min_clearance", clearance);
    easynav::declare_parameter_if_absent(*node, L + "min_surface_area", min_area);
    easynav::declare_parameter_if_absent(*node, L + "max_hole_area", hole);
    node->get_parameter(L + "enabled", local_.enabled);
    node->get_parameter(L + "frame", local_.frame);
    node->get_parameter(L + "range", local_.range);
    node->get_parameter(L + "min_z", local_.min_z);
    node->get_parameter(L + "max_z", local_.max_z);
    node->get_parameter(L + "accumulate_time", local_.accumulate_time);
    node->get_parameter(L + "freq", local_.freq);
    node->get_parameter(L + "downsample", local_.downsample);
    node->get_parameter(L + "resolution", res);
    node->get_parameter(L + "max_dz", max_dz);
    node->get_parameter(L + "max_slope_deg", slope);
    node->get_parameter(L + "min_clearance", clearance);
    node->get_parameter(L + "min_surface_area", min_area);
    node->get_parameter(L + "max_hole_area", hole);
    b.resolution = static_cast<float>(res);
    b.max_dz = static_cast<float>(max_dz);
    b.max_slope_deg = static_cast<float>(slope);
    b.min_clearance = static_cast<float>(clearance);
    b.min_surface_area = static_cast<float>(min_area);
    b.max_hole_area = static_cast<float>(hole);
    if (local_.frame.empty()) {local_.frame = RTTFBuffer::getInstance()->get_tf_info().robot_frame;}
    if (local_.enabled) {
      RCLCPP_INFO(
        node->get_logger(), "[%s] local mode: NavMap in %s, %.1f m around the robot, %.1f Hz",
        plugin_name.c_str(), local_.frame.c_str(), local_.range, local_.freq);
    }
  }

  std::vector<std::string> navmap_filters;
  easynav::declare_parameter_if_absent(*node, plugin_name + ".filters", navmap_filters);
  node->get_parameter(plugin_name + ".filters", navmap_filters);

  for (const auto & navmap_filter : navmap_filters) {
    std::string plugin;
    easynav::declare_parameter_if_absent(
      *node, plugin_name + "." + navmap_filter + ".plugin",
      plugin);
    node->get_parameter(plugin_name + "." + navmap_filter + ".plugin", plugin);

    try {
      RCLCPP_INFO(
        node->get_logger(),
        "Loading NavMapFilter %s [%s]", navmap_filter.c_str(), plugin.c_str());
      std::shared_ptr<NavMapFilter> instance;
      instance = navmap_filters_loader_->createSharedInstance(plugin);

      try {
        instance->initialize(node, plugin_name + "." + navmap_filter);
      } catch (const std::runtime_error & ex) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), ex.what());
        throw;
      }

      navmap_filters_.push_back(instance);

      RCLCPP_INFO(
        node->get_logger(),
        "Loaded NavMapFilter %s [%s]", navmap_filter.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Unable to load plugin easynav::navmap::NavMapFilter. Error: %s", ex.what());
      throw std::runtime_error(
              "Unable to load plugin easynav::navmap::NavMapFilter " +
              navmap_filter + " . Error: " + ex.what());
    }
  }

  navmap_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/map",
    rclcpp::QoS(1).transient_local().reliable());

  layer_updates_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMapLayer>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/map_updates",
    rclcpp::QoS(100));

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();


  if (!package_name.empty() && !occmap_path_file.empty()) {
    try {
      const std::string pkgpath = easynav::get_package_share_path(package_name);
      map_path_ = pkgpath + std::string("/") + occmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    nav_msgs::msg::OccupancyGrid occ_msg;
    if (auto ret = loadMapFromYaml(map_path_, occ_msg) != LOAD_MAP_SUCCESS) {
      std::cerr << "loadMapFromYaml returned" << ret << std::endl;
      throw std::runtime_error("YAML file [" + map_path_ + "] not found or invalid: ");
    }

    resolution_ = occ_msg.info.resolution;
    navmap_ = navmap_ros::from_occupancy_grid(occ_msg);

    navmap_msg_ = navmap_ros::to_msg(navmap_);
    navmap_msg_.header.frame_id = tf_info.map_frame;
    navmap_msg_.header.stamp = node->now();
    navmap_pub_->publish(navmap_msg_);
  }

  if (!package_name.empty() && !navmap_path_file.empty()) {
    try {
      const std::string pkgpath = easynav::get_package_share_path(package_name);
      map_path_ = pkgpath + std::string("/") + navmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (navmap_ros::io::load_from_file(map_path_, navmap_)) {
      navmap_msg_ = navmap_ros::to_msg(navmap_);
      navmap_msg_.header.frame_id = tf_info.map_frame;
      navmap_msg_.header.stamp = node->now();
      navmap_pub_->publish(navmap_msg_);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to load %s", map_path_.c_str());
    }
  }

  incoming_occ_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/incoming_occ_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [&](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
      resolution_ = msg->info.resolution;
      navmap_ = navmap_ros::from_occupancy_grid(*msg);
      map_changed_ = true;

      navmap_msg_ = navmap_ros::to_msg(navmap_);
      navmap_msg_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
      navmap_msg_.header.stamp = this->get_node()->now();
      navmap_pub_->publish(navmap_msg_);
    });

  incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/incoming_pc2_map",
    rclcpp::QoS(100),
    [&](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      navmap_ros::BuildParams params;
      navmap_ = navmap_ros::from_pointcloud2(*msg, navmap_msg_, params);
      map_changed_ = true;


      navmap_msg_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
      navmap_msg_.header.stamp = this->get_node()->now();
      navmap_pub_->publish(navmap_msg_);
    });


  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_node_base_interface()->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      (void)response;
      navmap_ros::io::save_to_file(navmap_, "/tmp/map.navmap");
      // ToDo
    });
}

void
NavMapMapsManager::set_static_map([[maybe_unused]] const ::navmap::NavMap & new_map)
{
  // navmap_ = new_map;
}

void
NavMapMapsManager::update(::easynav::NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  if (local_.enabled) {update_local(nav_state);}

  // A new map (loaded, received or rebuilt) replaces the one in NavState
  if (map_changed_ || !nav_state.has("map.navmap")) {
    nav_state.set("map.navmap", navmap_);
    map_changed_ = false;
  }

  // Filters update the map in NavState in place, and record which cells they change
  const std::size_t subscribers = layer_updates_pub_->get_subscription_count();
  const bool new_subscriber = subscribers > last_subscribers_;
  last_subscribers_ = subscribers;
  std_msgs::msg::Header header;
  header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  header.stamp = get_node()->now();
  for (const auto & filter : navmap_filters_) {
    filter->set_map_resolution(resolution_);
    filter->update(nav_state);

    if (subscribers == 0 || !filter->is_adding_layer()) {continue;}
    const auto & navmap = nav_state.get<::navmap::NavMap>("map.navmap");
    const auto layer = navmap.layers.get(filter->get_layer_name());
    if (!layer || (!layer->has_changes() && !new_subscriber)) {continue;}
    // Only the changed cells (or the whole layer if many changed, or for a new subscriber)
    layer_updates_pub_->publish(
      navmap_ros::to_layer_update_msg(
        navmap, filter->get_layer_name(), header, new_subscriber ? 0.0f : 0.25f));
  }

  // Every filter and the publisher have seen this cycle's changes
  if (nav_state.has("map.navmap")) {
    const auto & navmap = nav_state.get<::navmap::NavMap>("map.navmap");
    for (const auto & name : navmap.list_layers()) {
      navmap.layers.get(name)->clear_changes();
    }
  }
}


void
NavMapMapsManager::update_local(::easynav::NavState & nav_state)
{
  auto node = get_node();
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const double now = node->now().seconds();

  // 1) Accumulate the newest perceptions, in odom (it does not jump), around the robot
  {
    const auto & perceptions = nav_state.get_no_group<PointPerception>();
    const double stamp = perceptions.empty() ? 0.0 :
      get_latest_point_perceptions_stamp(perceptions).seconds();
    if (!perceptions.empty() && (local_scans_.empty() || stamp > local_scans_.back().stamp)) {
      try {
        auto cloud = PointPerceptionsOpsView(perceptions)
          .downsample(local_.downsample)
          .fuse(local_.frame)
          .filter(
          {-local_.range, -local_.range, local_.min_z},
          {local_.range, local_.range, local_.max_z}, false)
          .fuse(tf_info.odom_frame)
          .as_points();
        local_scans_.push_back({stamp, std::move(cloud)});
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 5000, "[%s] local mode: %s",
          get_plugin_name().c_str(), e.what());
      }
    }
  }
  while (!local_scans_.empty() &&
    now - local_scans_.front().stamp > local_.accumulate_time)
  {
    local_scans_.pop_front();
  }

  // 2) At local.freq: put them together in the local frame, crop, voxelize and mesh
  if (local_scans_.empty() ||
    (local_.freq > 0.0 && now - last_local_build_ < 1.0 / local_.freq))
  {
    return;
  }
  last_local_build_ = now;

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = RTTFBuffer::getInstance()->lookupTransform(
      local_.frame, tf_info.odom_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "[%s] local mode: no %s -> %s: %s",
      get_plugin_name().c_str(), tf_info.odom_frame.c_str(), local_.frame.c_str(), e.what());
    return;
  }
  const auto & q = tf.transform.rotation;
  const auto & t = tf.transform.translation;
  const Eigen::Quaternionf rot(
    static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
    static_cast<float>(q.z));
  const Eigen::Vector3f tr(
    static_cast<float>(t.x), static_cast<float>(t.y), static_cast<float>(t.z));
  const float range = static_cast<float>(local_.range);
  const float vox = static_cast<float>(std::max(local_.downsample, 1e-3));

  pcl::PointCloud<pcl::PointXYZ> local;
  std::unordered_set<std::int64_t> voxels;
  for (const auto & scan : local_scans_) {
    for (const auto & p : scan.cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
      const Eigen::Vector3f v = rot * Eigen::Vector3f(p.x, p.y, p.z) + tr;
      if (std::abs(v.x()) > range || std::abs(v.y()) > range || v.z() < local_.min_z ||
        v.z() > local_.max_z)
      {
        continue;
      }
      const std::int64_t key =
        (static_cast<std::int64_t>(std::floor(v.x() / vox)) * 73856093) ^
        (static_cast<std::int64_t>(std::floor(v.y() / vox)) * 19349663) ^
        (static_cast<std::int64_t>(std::floor(v.z() / vox)) * 83492791);
      if (voxels.insert(key).second) {local.push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));}
    }
  }

  navmap_ros_interfaces::msg::NavMap msg;
  ::navmap::NavMap nm = navmap_ros::from_points(local, msg, local_.build);
  if (nm.navcels.empty()) {return;}
  for (auto & s : nm.surfaces) {
    s.frame_id = local_.frame;
  }
  navmap_ = std::move(nm);
  map_changed_ = true;

  navmap_msg_ = navmap_ros::to_msg(navmap_);
  navmap_msg_.header.frame_id = local_.frame;
  navmap_msg_.header.stamp = node->now();
  navmap_pub_->publish(navmap_msg_);
}

}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::NavMapMapsManager, easynav::MapsManagerBase)
