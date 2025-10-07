// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.

#include <gtest/gtest.h>

#include "easynav_navmap_maps_manager/utils.hpp"

/// \brief Fixture for NavMapMapsManager tests
class NavMapMapsManagerUtilsTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

TEST_F(NavMapMapsManagerUtilsTests, BasicUsage)
{
  easynav::navmap::Params P;
  P.max_edge_len = 2.4f;     // algo > resolución del downsample
  P.max_slope_deg = 25.0f;   // ajusta según “transitabilidad”
  P.neighbor_radius = 2.4f;  // radio razonable
  P.use_radius = true;

  pcl::PointCloud<pcl::PointXYZ> input_points;

  input_points.push_back({0.5, 0.5, 0.0});
  input_points.push_back({1.9, 0.5, 0.0});
  input_points.push_back({0.5, 1.5, 0.0});
  input_points.push_back({-0.5, 1.5, 0.0});

  int seed = 0;
  auto triangles = easynav::navmap::grow_surface_from_seed(input_points, seed, P);


  std::cerr << "Triángulos: " << triangles.size() << std::endl;
  for (size_t i = 0; i < triangles.size(); i++) {
    pcl::PointXYZ v0 = input_points[triangles[i][0]];
    pcl::PointXYZ v1 = input_points[triangles[i][1]];
    pcl::PointXYZ v2 = input_points[triangles[i][2]];


    std::cerr << i << "\t(" << v0.x << ", " << v0.y << ", " << v0.z << ")" <<
      "\t(" << v1.x << ", " << v1.y << ", " << v1.z << ")" <<
      "\t(" << v2.x << ", " << v2.y << ", " << v2.z << ")" << std::endl;
  }

}


///// \brief Dynamic map update test with point cloud
//TEST_F(NavMapMapsManagerTest, BasicDynamicUpdate)
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
//  auto manager = std::make_shared<easynav::navmap::NavMapMapsManager>();
//  manager->initialize(node, "test");
//
//  easynav::NavMap2D static_map(30, 30, 0.1, -1.5, -1.5);
//  manager->set_static_map(static_map);
//
//  ::easynav::NavState ::easynav::NavState;
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
//  ::easynav::NavState.set("points", perceptions);
//
//  manager->update(::easynav::NavState);
//
//  ASSERT_TRUE(::easynav::NavState.has("map.dynamic"));
//  const auto & map = ::easynav::NavState.get<easynav::NavMap2D>("map.dynamic");
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
//TEST_F(NavMapMapsManagerTest, IncomingOccupancyGridUpdatesMaps)
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node2");
//  auto manager = std::make_shared<easynav::navmap::NavMapMapsManager>();
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
//  ::easynav::NavState ::easynav::NavState;
//  manager->update(::easynav::NavState);
//
//  ASSERT_TRUE(::easynav::NavState.has("map.static"));
//  const auto & map = ::easynav::NavState.get<easynav::NavMap2D>("map.static");
//
//  EXPECT_EQ(map.getCost(5, 5), easynav::LETHAL_OBSTACLE);
//  EXPECT_EQ(map.getCost(1, 1), easynav::FREE_SPACE);
//}
//
///// \brief Helper subclass to force map path for savemap
//class FriendNavMapMapsManager : public easynav::navmap::NavMapMapsManager
//{
//public:
//  void force_path(const std::string & path) {map_path_ = path;}
//};
//
///// \brief Test that the savemap service correctly stores YAML and PGM files
//TEST_F(NavMapMapsManagerTest, SavemapServiceWorks)
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_savemap_node");
//  auto manager = std::make_shared<FriendNavMapMapsManager>();
//  manager->initialize(node, "test_savemap");
//
//  // Create a 200x200 static ::navmap::NavMap with resolution 0.05 and origin at (0, 0)
//  const unsigned int width = 200;
//  const unsigned int height = 200;
//  easynav::NavMap2D map_static(width, height, 0.05, 0.0, 0.0);
//
//  // Set a vertical line of lethal obstacles at x = 30
//  for (unsigned int y = 0; y < height; ++y) {
//    map_static.setCost(30, y, easynav::LETHAL_OBSTACLE);
//  }
//
//  manager->set_static_map(map_static);
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
//  easynav::NavMap2D loaded_map(loaded_grid);
//
//  // Check map dimensions match
//  ASSERT_EQ(loaded_map.getSizeInCellsX(), map_static.getSizeInCellsX());
//  ASSERT_EQ(loaded_map.getSizeInCellsY(), map_static.getSizeInCellsY());
//
//  for (unsigned int y = 0; y < map_static.getSizeInCellsY(); ++y) {
//    for (unsigned int x = 0; x < map_static.getSizeInCellsX(); ++x) {
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
