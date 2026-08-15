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
#include <fstream>
#include <thread>

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_routes_maps_manager/RoutesMapsManager.hpp"
#include "easynav_routes_maps_manager/msg/routes_map.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using easynav::RoutesMapsManager;
using easynav::RoutesMap;
using easynav::RouteSegment;

class RoutesMapsManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

// Helper to create a temporary YAML file with given contents
static std::string create_temp_yaml(const std::string & contents)
{
  char filename[] = "/tmp/routes_test_XXXXXX.yaml";
  int fd = mkstemps(filename, 5);  // keep .yaml suffix
  if (fd == -1) {
    throw std::runtime_error("Unable to create temp file");
  }
  close(fd);

  std::ofstream out(filename);
  out << contents;
  out.close();
  return std::string(filename);
}

TEST_F(RoutesMapsManagerTest, LoadsRoutesFromValidYaml)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node");

  const std::string yaml =
    "routes: [route1, route2]\n"
    "route1:\n"
    "  start: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n"
    "  end:   {x: 1.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n"
    "route2:\n"
    "  start: {x: 1.0, y: 1.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n"
    "  end:   {x: 2.0, y: 1.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n";

  const auto filename = create_temp_yaml(yaml);

  // Declare parameters expected by RoutesMapsManager
  node->declare_parameter("routes.package", std::string(""));
  node->declare_parameter("routes.map_path_file", std::string(""));

  node->set_parameters({
    rclcpp::Parameter("routes.map_path_file", filename),
    rclcpp::Parameter("routes.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  easynav::TFInfo tf_info;
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  ASSERT_NO_THROW(manager->initialize(node, "routes"));

  const auto & routes = manager->get_routes();
  ASSERT_EQ(routes.size(), 2u);

  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 1.0);
  EXPECT_DOUBLE_EQ(routes[1].start.position.y, 1.0);
  EXPECT_DOUBLE_EQ(routes[1].end.position.x, 2.0);
}

TEST_F(RoutesMapsManagerTest, DefaultRouteWhenMapPathEmpty)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node_empty_path");

  node->declare_parameter("routes.package", std::string(""));
  node->declare_parameter("routes.map_path_file", std::string(""));

  node->set_parameters({
    rclcpp::Parameter("routes.map_path_file", std::string("")),
    rclcpp::Parameter("routes.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  easynav::TFInfo tf_info;
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  ASSERT_NO_THROW(manager->initialize(node, "routes"));

  const auto & routes = manager->get_routes();
  ASSERT_EQ(routes.size(), 1u);
  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].start.position.y, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 1.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.y, 0.0);
}

TEST_F(RoutesMapsManagerTest, DefaultRouteWhenYamlMissing)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node_missing");

  node->declare_parameter("routes.package", std::string(""));
  node->declare_parameter("routes.map_path_file", std::string("/tmp/non_existent_routes.yaml"));

  node->set_parameters({
    rclcpp::Parameter("routes.map_path_file", std::string("/tmp/non_existent_routes.yaml")),
    rclcpp::Parameter("routes.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  easynav::TFInfo tf_info;
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  ASSERT_NO_THROW(manager->initialize(node, "routes"));

  const auto & routes = manager->get_routes();
  ASSERT_EQ(routes.size(), 1u);
  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 1.0);
}

TEST_F(RoutesMapsManagerTest, DefaultRouteWhenNoRoutesKey)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node_no_routes_key");

  const std::string yaml = "foo: bar\n";
  const auto filename = create_temp_yaml(yaml);

  node->declare_parameter("routes.package", std::string(""));
  node->declare_parameter("routes.map_path_file", std::string(""));

  node->set_parameters({
    rclcpp::Parameter("routes.map_path_file", filename),
    rclcpp::Parameter("routes.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  easynav::TFInfo tf_info;
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  ASSERT_NO_THROW(manager->initialize(node, "routes"));

  const auto & routes = manager->get_routes();
  ASSERT_EQ(routes.size(), 1u);
  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 1.0);
}

TEST_F(RoutesMapsManagerTest, UpdateWritesRoutesIntoNavState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node_update");

  const std::string yaml =
    "routes: [route1]\n"
    "route1:\n"
    "  start: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n"
    "  end:   {x: 1.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n";

  const auto filename = create_temp_yaml(yaml);

  node->declare_parameter("routes.package", std::string(""));
  node->declare_parameter("routes.map_path_file", std::string(""));

  node->set_parameters({
    rclcpp::Parameter("routes.map_path_file", filename),
    rclcpp::Parameter("routes.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  easynav::TFInfo tf_info;
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  ASSERT_NO_THROW(manager->initialize(node, "routes"));

  easynav::NavState nav_state;
  manager->update(nav_state);

  ASSERT_TRUE(nav_state.has("routes"));
  const auto & routes = nav_state.get<RoutesMap>("routes");
  ASSERT_EQ(routes.size(), 1u);
  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 1.0);
}

TEST_F(RoutesMapsManagerTest, IncomingRoutesTopicUpdatesInternalAndNavState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node_incoming");

  node->declare_parameter("routes.package", std::string(""));
  node->declare_parameter("routes.map_path_file", std::string(""));

  auto manager = std::make_shared<RoutesMapsManager>();
  easynav::TFInfo tf_info;
  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);

  ASSERT_NO_THROW(manager->initialize(node, "routes"));

  // Before any message arrives, the manager holds the (empty-path)
  // default single segment.
  ASSERT_EQ(manager->get_routes().size(), 1u);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  const std::string topic =
    node->get_fully_qualified_name() + std::string("/routes/incoming_routes");
  auto pub = node->create_publisher<easynav_routes_maps_manager::msg::RoutesMap>(
    topic, rclcpp::QoS(1).transient_local().reliable());
  pub->on_activate();

  easynav_routes_maps_manager::msg::RoutesMap msg;

  easynav_routes_maps_manager::msg::RouteSegment seg1;
  seg1.id = "incoming1";
  seg1.start.position.x = 5.0;
  seg1.end.position.x = 6.0;
  seg1.end.orientation.w = 1.0;
  msg.routes.push_back(seg1);

  easynav_routes_maps_manager::msg::RouteSegment seg2;
  seg2.id = "incoming2";
  seg2.start.position.y = 7.0;
  seg2.end.position.y = 8.0;
  seg2.end.orientation.w = 1.0;
  msg.routes.push_back(seg2);

  pub->publish(msg);

  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();

  const auto & routes = manager->get_routes();
  ASSERT_EQ(routes.size(), 2u);
  EXPECT_EQ(routes[0].id, "incoming1");
  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 5.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 6.0);
  EXPECT_EQ(routes[1].id, "incoming2");
  EXPECT_DOUBLE_EQ(routes[1].start.position.y, 7.0);
  EXPECT_DOUBLE_EQ(routes[1].end.position.y, 8.0);

  easynav::NavState nav_state;
  manager->update(nav_state);
  ASSERT_TRUE(nav_state.has("routes"));
  const auto & nav_routes = nav_state.get<RoutesMap>("routes");
  ASSERT_EQ(nav_routes.size(), 2u);
  EXPECT_EQ(nav_routes[0].id, "incoming1");
  EXPECT_EQ(nav_routes[1].id, "incoming2");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
