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

#include <string>

#include "easynav_octomap_maps_manager/OctomapMapsManager.hpp"

#include "easynav_common/YTSession.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "easynav_octomap_maps_manager/map_io.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{
namespace octomap
{

using std::placeholders::_1;

OctomapMapsManager::OctomapMapsManager()
{
  ::easynav::NavState::register_printer<::octomap::OcTree>(
    [](const ::octomap::OcTree & map) {
      (void)map;
      std::ostringstream oss;
      oss << "Octomap";
      return oss.str();
    });

  // octomap_filters_loader_ = std::make_unique<pluginlib::ClassLoader<OctomapFilter>>(
  //   "easynav_octomap_filter", "easynav::octomap::OctomapFilter");
}

OctomapMapsManager::~OctomapMapsManager() {}

void
OctomapMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  // std::string package_name, occmap_path_file, octomap_path_file;
  // node->declare_parameter(plugin_name + ".package", package_name);
  // node->declare_parameter(plugin_name + ".occmap_path_file", occmap_path_file);

  // node->get_parameter(plugin_name + ".package", package_name);
  // node->get_parameter(plugin_name + ".occmap_path_file", occmap_path_file);
  // node->get_parameter(plugin_name + ".octomap_path_file", octomap_path_file);

  // std::vector<std::string> octomap_filters;
  // node->declare_parameter(plugin_name + ".filters", octomap_filters);
  // node->get_parameter(plugin_name + ".filters", octomap_filters);

  //  for (const auto & octomap_filter : octomap_filters) {
  //    std::string plugin;
  //    node->declare_parameter(plugin_name + "." + octomap_filter + ".plugin", plugin);
  //    node->get_parameter(plugin_name + "." + octomap_filter + ".plugin", plugin);
  //
  //    try {
  //      RCLCPP_INFO(node->get_logger(),
  //        "Loading OctomapFilter %s [%s]", octomap_filter.c_str(), plugin.c_str());
  //
  //      std::shared_ptr<OctomapFilter> instance;
  //      instance = octomap_filters_loader_->createSharedInstance(plugin);
  //
  //      auto result = instance->initialize(node, plugin_name + "." + octomap_filter,
  //        get_tf_prefix());
  //
  //      if (!result) {
  //        RCLCPP_ERROR(node->get_logger(),
  //          "Unable to initialize [%s]. Error: %s", plugin.c_str(), result.error().c_str());
  //        throw std::runtime_error("Unable to initialize " +
  //          plugin + " . Error: " + result.error());
  //      }
  //
  //      octomap_filters_.push_back(instance);
  //
  //      RCLCPP_INFO(node->get_logger(),
  //        "Loaded OctomapFilter %s [%s]", octomap_filter.c_str(), plugin.c_str());
  //    } catch (pluginlib::PluginlibException & ex) {
  //      RCLCPP_ERROR(node->get_logger(),
  //        "Unable to load plugin easynav::octomap::OctomapFilter. Error: %s", ex.what());
  //      throw std::runtime_error("Unable to load plugin easynav::octomap::OctomapFilter " +
  //        octomap_filter + " . Error: " + ex.what());
  //    }
  //  }

  octomap_pub_ = node->create_publisher<octomap_msgs::msg::Octomap>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

//   if (!package_name.empty() && !occmap_path_file.empty()) {
//     try {
//       const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
//       map_path_ = pkgpath + std::string("/") + occmap_path_file;
//     } catch (ament_index_cpp::PackageNotFoundError & ex) {
//       throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
//     }
//
//     nav_msgs::msg::OccupancyGrid occ_msg;
//     if (auto ret = loadMapFromYaml(map_path_, occ_msg) != LOAD_MAP_SUCCESS) {
//       std::cerr << "loadMapFromYaml returned" << ret << std::endl;
//       throw std::runtime_error("YAML file [" + map_path_ + "] not found or invalid: ");
//     }
//
//     resolution_ = occ_msg.info.resolution;
//     octomap_ = octomap_ros::from_occupancy_grid(occ_msg);
//
//     octomap_msg_ = octomap_ros::to_msg(octomap_);
//     octomap_msg_.header.frame_id = "map";
//     octomap_msg_.header.stamp = node->now();
//     octomap_pub_->publish(octomap_msg_);
//   }
//
//   if (!package_name.empty() && !octomap_path_file.empty()) {
//     try {
//       const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
//       map_path_ = pkgpath + std::string("/") + occmap_path_file;
//     } catch (ament_index_cpp::PackageNotFoundError & ex) {
//       throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
//     }
//
//     if (octomap_ros::io::load_from_file(map_path_, octomap_)) {
//       octomap_msg_ = octomap_ros::to_msg(octomap_);
//       octomap_msg_.header.frame_id = "map";
//       octomap_msg_.header.stamp = node->now();
//       octomap_pub_->publish(octomap_msg_);
//     } else {
//       RCLCPP_INFO(get_node()->get_logger(), "Unable to load %s", map_path_.c_str());
//     }
//   }

//  incoming_occ_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
//    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_occ_map",
//    rclcpp::QoS(1).transient_local().reliable(),
//    [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
//
//      resolution_ = msg->info.resolution;
//      octomap_ = octomap_ros::from_occupancy_grid(*msg);
//
//      octomap_msg_ = octomap_ros::to_msg(octomap_);
//      octomap_msg_.header.frame_id = "map";
//      octomap_msg_.header.stamp = this->get_node()->now();
//      octomap_pub_->publish(octomap_msg_);
//    });

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_pc2_map",
    rclcpp::QoS(100),
    [&](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

      geometry_msgs::msg::TransformStamped tf_msg;
      try {
        tf_msg = RTTFBuffer::getInstance()->lookupTransform(
          tf_info.map_frame, msg->header.frame_id, msg->header.stamp,
            rclcpp::Duration::from_seconds(0.05));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_node()->get_logger(), "OctomapMapsManager: TF failed: %s", ex.what());
        return;
      }

      float resolution = 0.3f;

      tf2::Transform tf;
      tf2::fromMsg(tf_msg.transform, tf);

      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ds(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *pcl_ds);

      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(static_cast<float>(resolution),
                 static_cast<float>(resolution),
                 static_cast<float>(resolution));
      vg.filter(*pcl_ds);

      octomap_ = std::make_shared<::octomap::OcTree>(resolution);
      octomap_->setProbHit(0.7);
      octomap_->setProbMiss(0.4);
      octomap_->setClampingThresMin(0.12);
      octomap_->setClampingThresMax(0.97);

      ::octomap::Pointcloud cloud;
      cloud.reserve(pcl_ds->size());

      for (const auto & p : pcl_ds->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}

        tf2::Vector3 ps(p.x, p.y, p.z);
        tf2::Vector3 p_map = tf * ps;
        cloud.push_back(p_map.x(), p_map.y(), p_map.z());
      }

      const auto & t = tf_msg.transform.translation;
      ::octomap::point3d origin(t.x, t.y, t.z);

      octomap_->insertPointCloud(cloud, origin, 1000.0, true, false);
      octomap_->updateInnerOccupancy();

      octomap_msg_.header.frame_id = tf_info.map_frame;
      octomap_msg_.header.stamp = this->get_node()->now();
      octomap_msg_.id = "OcTree";
      octomap_msg_.binary = true;
      if (octomap_msgs::binaryMapToMsg(*octomap_, octomap_msg_)) {
        octomap_pub_->publish(octomap_msg_);
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "OctomapMapsManager: Octomap msg creation failed");
      }
    });


  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      (void)response;

      // ToDo
    });
}

void
OctomapMapsManager::update([[maybe_unused]] ::easynav::NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  // if (!nav_state.has("map")) {
  //   nav_state.set("map", octomap_);
  // }


  // for (const auto & filter : octomap_filters_) {
  //   filter->set_map_resolution(resolution_);
  //   filter->update(nav_state);
  // }

  // octomap_ = nav_state.get<::octomap::Octomap>("map");

//   for (const auto & filter : octomap_filters_) {
//
//     if (filter->is_adding_layer() && octomap_.has_layer(filter->get_layer_name())) {
//       auto update_msg = octomap_ros::to_msg(octomap_, filter->get_layer_name());
//       layer_updates_pub_->publish(update_msg);
//     }
//   }
}


}  // namespace octomap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::octomap::OctomapMapsManager, easynav::MapsManagerBase)
