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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_maps_manager/CostmapMapsManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

/// \brief Fixture for CostmapMapsManager tests
class CostmapMapsManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

static nav_msgs::msg::OccupancyGrid make_grid(
  int width,
  int height,
  int occ_x,
  int occ_y,
  int64_t stamp_ns)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  grid.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);

  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = 0.2f;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.data.assign(static_cast<size_t>(width * height), 0);

  if (occ_x >= 0 && occ_y >= 0 && occ_x < width && occ_y < height) {
    grid.data[static_cast<size_t>(occ_y * width + occ_x)] = 100;
  }

  return grid;
}

TEST_F(CostmapMapsManagerTest, SyncBaseMapPrefersNewerNavStateMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_sync_ns");

  easynav::TFInfo tf_info;
  tf_info.map_frame = "map";
  tf_info.odom_frame = "odom";
  tf_info.robot_frame = "base_link";
  tf_info.robot_footprint_frame = "base_footprint";
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  auto manager = std::make_shared<easynav::CostmapMapsManager>();
  manager->initialize(node, "test");

  const int64_t older_ns = 10LL * 1000000000LL;
  const int64_t newer_ns = 20LL * 1000000000LL;

  easynav::Costmap2D internal_old(make_grid(10, 10, 1, 1, older_ns));
  easynav::Costmap2D external_new(make_grid(10, 10, 2, 2, newer_ns));

  manager->set_base_map(internal_old);

  easynav::NavState navstate;
  navstate.set("map.base", external_new);

  manager->update(navstate);

  ASSERT_TRUE(navstate.has("map.base"));
  const auto & base_after = navstate.get<easynav::Costmap2D>("map.base");
  EXPECT_EQ(base_after.getLastModifiedStamp().nanoseconds(), newer_ns);

  ASSERT_TRUE(navstate.has("map.dynamic.filtered"));
  const auto dyn_ptr = navstate.get_ptr<easynav::Costmap2D>("map.dynamic.filtered");
  ASSERT_TRUE(dyn_ptr != nullptr);
  EXPECT_EQ(dyn_ptr->getLastModifiedStamp().nanoseconds(), newer_ns);
  EXPECT_EQ(dyn_ptr->getCost(2, 2), easynav::LETHAL_OBSTACLE);
  EXPECT_EQ(dyn_ptr->getCost(1, 1), easynav::FREE_SPACE);
}

TEST_F(CostmapMapsManagerTest, SyncBaseMapPrefersNewerInternalMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_sync_internal");

  easynav::TFInfo tf_info;
  tf_info.map_frame = "map";
  tf_info.odom_frame = "odom";
  tf_info.robot_frame = "base_link";
  tf_info.robot_footprint_frame = "base_footprint";
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  auto manager = std::make_shared<easynav::CostmapMapsManager>();
  manager->initialize(node, "test");

  const int64_t older_ns = 10LL * 1000000000LL;
  const int64_t newer_ns = 20LL * 1000000000LL;

  easynav::Costmap2D internal_new(make_grid(10, 10, 3, 3, newer_ns));
  easynav::Costmap2D external_old(make_grid(10, 10, 4, 4, older_ns));

  manager->set_base_map(internal_new);

  easynav::NavState navstate;
  navstate.set("map.base", external_old);

  manager->update(navstate);

  ASSERT_TRUE(navstate.has("map.base"));
  const auto & base_after = navstate.get<easynav::Costmap2D>("map.base");
  EXPECT_EQ(base_after.getLastModifiedStamp().nanoseconds(), newer_ns);

  ASSERT_TRUE(navstate.has("map.dynamic.filtered"));
  const auto dyn_ptr = navstate.get_ptr<easynav::Costmap2D>("map.dynamic.filtered");
  ASSERT_TRUE(dyn_ptr != nullptr);
  EXPECT_EQ(dyn_ptr->getLastModifiedStamp().nanoseconds(), newer_ns);
  EXPECT_EQ(dyn_ptr->getCost(3, 3), easynav::LETHAL_OBSTACLE);
  EXPECT_EQ(dyn_ptr->getCost(4, 4), easynav::FREE_SPACE);
}

TEST_F(CostmapMapsManagerTest, IncomingMapTopicUpdatesInternalAndNavState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_incoming");

  easynav::TFInfo tf_info;
  tf_info.map_frame = "map";
  tf_info.odom_frame = "odom";
  tf_info.robot_frame = "base_link";
  tf_info.robot_footprint_frame = "base_footprint";
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  auto manager = std::make_shared<easynav::CostmapMapsManager>();
  manager->initialize(node, "test");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  const std::string topic = node->get_fully_qualified_name() + std::string("/test/incoming_map");
  auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(1).transient_local().reliable());
  pub->on_activate();

  auto grid = make_grid(10, 10, 5, 5, 123LL * 1000000000LL);
  pub->publish(grid);

  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();

  easynav::NavState navstate;
  manager->update(navstate);

  ASSERT_TRUE(navstate.has("map.base"));
  const auto & base_after = navstate.get<easynav::Costmap2D>("map.base");
  EXPECT_EQ(base_after.getCost(5, 5), easynav::LETHAL_OBSTACLE);
  EXPECT_GT(base_after.getLastModifiedStamp().nanoseconds(), 0);

  ASSERT_TRUE(navstate.has("map.dynamic.filtered"));
  const auto dyn_ptr = navstate.get_ptr<easynav::Costmap2D>("map.dynamic.filtered");
  ASSERT_TRUE(dyn_ptr != nullptr);
  EXPECT_EQ(dyn_ptr->getCost(5, 5), easynav::LETHAL_OBSTACLE);
  EXPECT_EQ(dyn_ptr->getLastModifiedStamp().nanoseconds(),
    base_after.getLastModifiedStamp().nanoseconds());
}

///// \brief Dynamic map update test with point cloud
//TEST_F(CostmapMapsManagerTest, BasicDynamicUpdate)
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
//  auto manager = std::make_shared<easynav::CostmapMapsManager>();
//  manager->initialize(node, "test");
//
//  easynav::Costmap2D base_map(30, 30, 0.1, -1.5, -1.5);
//  manager->set_base_map(base_map);
//
//  easynav::NavState navstate;
//  auto perception = std::make_shared<easynav::PointPerception>();
//  perception->data.points.resize(2);
//  perception->data.points[0].x = 1.0;
//  perception->data.points[0].y = 1.0;
//  perception->data.points[0].z = 0.2;
//  perception->data.points[1].x = -1.0;
//  perception->data.points[1].y = -1.0;
//  perception->data.points[1].z = 0.2;
//  perception->frame_id = "map";
//  perception->stamp = node->now();
//  perception->valid = true;
//
//  easynav::PointPerceptions perceptions;
//  perceptions.push_back(perception);
//  navstate.set("points", perceptions);
//
//  manager->update(navstate);
//
//  ASSERT_TRUE(navstate.has("map.dynamic"));
//  const auto & map = navstate.get<easynav::Costmap2D>("map.dynamic");
//
//  unsigned int cx, cy;
//  ASSERT_TRUE(map.worldToMap(1.0, 1.0, cx, cy));
//  EXPECT_EQ(map.getCost(cx, cy), easynav::LETHAL_OBSTACLE);
//
//  ASSERT_TRUE(map.worldToMap(-1.0, -1.0, cx, cy));
//  EXPECT_EQ(map.getCost(cx, cy), easynav::LETHAL_OBSTACLE);
//}
//
///// \brief OccupancyGrid updates map via subscription
//TEST_F(CostmapMapsManagerTest, IncomingOccupancyGridUpdatesMaps)
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node2");
//  auto manager = std::make_shared<easynav::CostmapMapsManager>();
//  manager->initialize(node, "test2");
//
//  rclcpp::executors::SingleThreadedExecutor executor;
//  executor.add_node(node->get_node_base_interface());
//
//  auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
//    "test_node2/test2/incoming_map", rclcpp::QoS(1).transient_local().reliable());
//
//  pub->on_activate();
//
//  nav_msgs::msg::OccupancyGrid grid;
//  grid.header.frame_id = "map";
//  grid.info.width = 10;
//  grid.info.height = 10;
//  grid.info.resolution = 0.2;
//  grid.info.origin.position.x = -1.0;
//  grid.info.origin.position.y = -0.6;
//  grid.data.assign(100, 0);
//  grid.data[55] = 100;
//
//  pub->publish(grid);
//  executor.spin_some();
//  std::this_thread::sleep_for(std::chrono::milliseconds(100));
//
//  easynav::NavState navstate;
//  manager->update(navstate);
//
//  ASSERT_TRUE(navstate.has("map.base"));
//  const auto & map = navstate.get<easynav::Costmap2D>("map.base");
//
//  EXPECT_EQ(map.getCost(5, 5), easynav::LETHAL_OBSTACLE);
//  EXPECT_EQ(map.getCost(1, 1), easynav::FREE_SPACE);
//}
//
///// \brief Helper subclass to force map path for savemap
//class FriendCostmapMapsManager : public easynav::CostmapMapsManager
//{
//public:
//  void force_path(const std::string & path) {map_path_ = path;}
//};
//
///// \brief Test that the savemap service correctly stores YAML and PGM files
//TEST_F(CostmapMapsManagerTest, SavemapServiceWorks)
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_savemap_node");
//  auto manager = std::make_shared<FriendCostmapMapsManager>();
//  manager->initialize(node, "test_savemap");
//
//  // Create a 200x200 base costmap with resolution 0.05 and origin at (0, 0)
//  const unsigned int width = 200;
//  const unsigned int height = 200;
//  easynav::Costmap2D map_base(width, height, 0.05, 0.0, 0.0);
//
//  // Set a vertical line of lethal obstacles at x = 30
//  for (unsigned int y = 0; y < height; ++y) {
//    map_base.setCost(30, y, easynav::LETHAL_OBSTACLE);
//  }
//
//  manager->set_base_map(map_base);
//
//  const std::string yaml_path = "/tmp/savemap_test_map";
//  const std::string service_name = "/test_savemap_node/test_savemap/savemap";
//
//  manager->force_path(yaml_path);
//
//  // Create executor and service client
//  rclcpp::executors::SingleThreadedExecutor executor;
//  executor.add_node(node->get_node_base_interface());
//
//  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
//  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
//
//  // Call savemap service
//  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//  auto future = client->async_send_request(request);
//  executor.spin_until_future_complete(future);
//
//  auto response = future.get();
//  EXPECT_TRUE(response->success);
//  EXPECT_NE(response->message.find("saved"), std::string::npos);
//
//  // Reload the map from the generated YAML + PGM files
//  nav_msgs::msg::OccupancyGrid loaded_grid;
//  EXPECT_EQ(easynav::loadMapFromYaml(yaml_path + ".yaml", loaded_grid), easynav::LOAD_MAP_SUCCESS);
//
//  easynav::Costmap2D loaded_map(loaded_grid);
//
//  // Check map dimensions match
//  ASSERT_EQ(loaded_map.getSizeInCellsX(), map_base.getSizeInCellsX());
//  ASSERT_EQ(loaded_map.getSizeInCellsY(), map_base.getSizeInCellsY());
//
//  for (unsigned int y = 0; y < map_base.getSizeInCellsY(); ++y) {
//    for (unsigned int x = 0; x < map_base.getSizeInCellsX(); ++x) {
//      if (x == 30) {
//        EXPECT_EQ(loaded_map.getCost(x, y), easynav::LETHAL_OBSTACLE)
//          << "Expected LETHAL_OBSTACLE at (" << x << "," << y << ")";
//      } else {
//        EXPECT_EQ(loaded_map.getCost(x, y), easynav::FREE_SPACE)
//          << "Expected FREE_SPACE at (" << x << "," << y << ")";
//      }
//    }
//  }
//}
