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

#include "easynav_bonxai_maps_manager/BonxaiMapsManager.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "bonxai/bonxai.hpp"
#include "bonxai/pcl_utils.hpp"
#include "easynav_bonxai_maps_manager/map_io.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav_bonxai
{

using std::placeholders::_1;

BonxaiMapsManager::BonxaiMapsManager()
{
  ::easynav::NavState::register_printer<Bonxai::ProbabilisticMap>(
    [](const Bonxai::ProbabilisticMap & map) {
      std::ostringstream oss;
      oss << "Bonxai Map with " << map.grid().activeCellsCount() <<
        " cells with resolution = " << map.grid().voxelSize();
      return oss.str();
    });
}

BonxaiMapsManager::~BonxaiMapsManager() {}

void
BonxaiMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, bonxai_path_file, occmap_path_file;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".bonxai_path_file", bonxai_path_file);
  node->declare_parameter(plugin_name + ".occmap_path_file", occmap_path_file);
  node->declare_parameter(plugin_name + ".resolution", resolution_);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".bonxai_path_file", bonxai_path_file);
  node->get_parameter(plugin_name + ".occmap_path_file", occmap_path_file);
  node->get_parameter(plugin_name + ".resolution", resolution_);

  bonxai_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  map_path_ = "/tmp/bonxai_map.pcd";

  if (!package_name.empty() && !bonxai_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + bonxai_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    std::vector<Eigen::Vector3d> bonxai_result;
    if (Bonxai::ReadPointsFromPCD(map_path_, bonxai_result)) {
      bonxai_map_ = std::make_shared<Bonxai::ProbabilisticMap>(resolution_);

      pcl::PointCloud<pcl::PointXYZ> pcl_out;
      for (const auto & p : bonxai_result) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {continue;}

        bonxai_map_->addHitPoint(p);
        pcl_out.push_back({
            static_cast<float>(p.x()),
            static_cast<float>(p.y()),
            static_cast<float>(p.z())
        });
      }

      bonxai_msg_.data.clear();
      pcl::toROSMsg(pcl_out, bonxai_msg_);

      publish_map();
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Unable to load %s", map_path_.c_str());
    }
  }

  if (!package_name.empty() && !occmap_path_file.empty()) {
    try {
      const std::string pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + std::string("/") + occmap_path_file;
    } catch (ament_index_cpp::PackageNotFoundError & ex) {
      throw std::runtime_error("Package " + package_name + " not found. Error: " + ex.what());
    }

    nav_msgs::msg::OccupancyGrid occ_msg;
    if (auto ret = loadMapFromYaml(map_path_, occ_msg) != LOAD_MAP_SUCCESS) {
      std::cerr << "loadMapFromYaml returned" << ret << std::endl;
      throw std::runtime_error("YAML file [" + map_path_ + "] not found or invalid: ");
    }

    update_from_occ(occ_msg);
    publish_map();
  }

  incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_pc2_map",
    rclcpp::QoS(100),
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      update_from_pc2(*msg);
      publish_map();
    });

  incoming_occ_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/incoming_occ_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {

      update_from_occ(*msg);
      publish_map();
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      (void)response;

      std::vector<Eigen::Vector3d> bonxai_result;
      bonxai_result.clear();
      bonxai_map_->getOccupiedVoxels(bonxai_result);
      Bonxai::WritePointsFromPCD(map_path_, bonxai_result);  // This can overwrite yaml occ maps
      // ToDo
    });
}

void
BonxaiMapsManager::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has("map.bonxai")) {
    nav_state.set("map.bonxai", bonxai_map_);
  }
}


void
BonxaiMapsManager::update_from_pc2(const sensor_msgs::msg::PointCloud2 & pc2)
{
  const auto & tf_info = ::easynav::RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = ::easynav::RTTFBuffer::getInstance()->lookupTransform(
          tf_info.map_frame, pc2.header.frame_id, pc2.header.stamp,
          rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "OctomapMapsManager: TF failed: %s", ex.what());
    return;
  }

  tf2::Transform tf;
  tf2::fromMsg(tf_msg.transform, tf);

  pcl::PointCloud<pcl::PointXYZ> pcl_ds;
  pcl::fromROSMsg(pc2, pcl_ds);

  bonxai_map_ = std::make_shared<Bonxai::ProbabilisticMap>(resolution_);

  for (const auto & p : pcl_ds.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}

    tf2::Vector3 ps(p.x, p.y, p.z);
    tf2::Vector3 p_map = tf * ps;
    bonxai_map_->addHitPoint({p_map.x(), p_map.y(), p_map.z()});
  }


  pcl::PointCloud<pcl::PointXYZ> pcl_out;
  std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_result.clear();
  bonxai_map_->getOccupiedVoxels(bonxai_result);
  for (const auto & voxel : bonxai_result) {
    pcl_out.push_back({
        static_cast<float>(voxel.x()),
        static_cast<float>(voxel.y()),
        static_cast<float>(voxel.z())
    });
  }

  bonxai_msg_.data.clear();
  pcl::toROSMsg(pcl_out, bonxai_msg_);
}

void
BonxaiMapsManager::update_from_occ(const nav_msgs::msg::OccupancyGrid & occ)
{
  resolution_ = occ.info.resolution;

// Transformación del pose de origen del OccupancyGrid (incluye rotación)
  tf2::Transform T_origin;
  {
    geometry_msgs::msg::Pose pose = occ.info.origin;
    tf2::fromMsg(pose, T_origin);
  }

  // Crear o resetear el mapa Bonxai
  bonxai_map_ = std::make_shared<Bonxai::ProbabilisticMap>(resolution_);

  const uint32_t width = occ.info.width;
  const uint32_t height = occ.info.height;
  const auto & data = occ.data;

  // Recorremos el grid
  for (uint32_t iy = 0; iy < height; ++iy) {
    for (uint32_t ix = 0; ix < width; ++ix) {
      const size_t idx = static_cast<size_t>(iy) * width + ix;
      if (idx >= data.size()) {break;}

      const int8_t v = data[idx];
      if (v == OCC_GRID_UNKNOWN) {continue;}

      const double cx = (static_cast<double>(ix) + 0.5) * resolution_;
      const double cy = (static_cast<double>(iy) + 0.5) * resolution_;

      const tf2::Vector3 p_cell(cx, cy, 0.0);
      const tf2::Vector3 p_world = T_origin * p_cell;

      if (v == OCC_GRID_FREE) {
        bonxai_map_->addHitPoint(Eigen::Vector3d(p_world.x(), p_world.y(), 0.0));
      } else if (v == OCC_GRID_OCCUPIED) {
        for (double z = 0.0; z <= height_with_occ_; z += resolution_) {
          bonxai_map_->addHitPoint(Eigen::Vector3d(p_world.x(), p_world.y(), z));
        }
      } else {
        continue;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_out;
  std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_result.clear();
  bonxai_map_->getOccupiedVoxels(bonxai_result);
  for (const auto & voxel : bonxai_result) {
    pcl_out.push_back({
        static_cast<float>(voxel.x()),
        static_cast<float>(voxel.y()),
        static_cast<float>(voxel.z())
    });
  }

  bonxai_msg_.data.clear();
  pcl::toROSMsg(pcl_out, bonxai_msg_);
}

void
BonxaiMapsManager::publish_map()
{
  const auto & tf_info = ::easynav::RTTFBuffer::getInstance()->get_tf_info();

  bonxai_msg_.header.frame_id = tf_info.map_frame;
  bonxai_msg_.header.stamp = this->get_node()->now();
  bonxai_pub_->publish(bonxai_msg_);

}

}  // namespace easynav_bonxai

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav_bonxai::BonxaiMapsManager, easynav::MapsManagerBase)
