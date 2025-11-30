#include <gtest/gtest.h>
#include <fstream>

#include "easynav_routes_maps_manager/RoutesMapsManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using easynav::RoutesMapsManager;
using easynav::RoutesMap;
using easynav::RouteSegment;

class RoutesMapsManagerTestNode : public ::testing::Test
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

TEST_F(RoutesMapsManagerTestNode, LoadRoutesFromYamlAndPublish)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node");

  // Create a temporary YAML file with two simple segments.
  char filename[] = "/tmp/routes_test_XXXXXX.yaml";
  int fd = mkstemps(filename, 5);  // keep .yaml suffix
  ASSERT_NE(fd, -1);
  close(fd);

  std::ofstream out(filename);
  out << "routes: [route1, route2]\n";
  out << "route1:\n";
  out << "  start: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n";
  out << "  end:   {x: 1.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n";
  out << "route2:\n";
  out << "  start: {x: 1.0, y: 1.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n";
  out << "  end:   {x: 2.0, y: 1.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}\n";
  out.close();

  // Declare parameters before setting them to avoid exceptions.
  node->declare_parameter("routes_maps_manager.package", std::string(""));
  node->declare_parameter("routes_maps_manager.map_path_file", std::string(""));

  // Override parameters directly on the node.
  node->set_parameters({
    rclcpp::Parameter("routes_maps_manager.map_path_file", std::string(filename)),
    rclcpp::Parameter("routes_maps_manager.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  auto init_result = manager->initialize(node, "routes_maps_manager");
  ASSERT_TRUE(init_result.has_value()) << init_result.error();

  // Activate lifecycle node before plugin-specific initialization.
  auto state = node->configure();
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  state = node->activate();
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto result = manager->on_initialize();
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto & routes = manager->get_routes();
  ASSERT_EQ(routes.size(), 2u);

  EXPECT_DOUBLE_EQ(routes[0].start.position.x, 0.0);
  EXPECT_DOUBLE_EQ(routes[0].end.position.x, 1.0);
  EXPECT_DOUBLE_EQ(routes[1].start.position.y, 1.0);
  EXPECT_DOUBLE_EQ(routes[1].end.position.x, 2.0);
}

TEST_F(RoutesMapsManagerTestNode, WritesRoutesIntoNavState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "routes_mapsmanager_test_node2");

  // Use an empty but valid YAML file with no routes.
  char filename[] = "/tmp/routes_test_empty_XXXXXX.yaml";
  int fd = mkstemps(filename, 5);
  ASSERT_NE(fd, -1);
  close(fd);

  std::ofstream out(filename);
  out << "routes: []\n";
  out.close();

  // Declare parameters before setting them to avoid exceptions.
  node->declare_parameter("routes_maps_manager.package", std::string(""));
  node->declare_parameter("routes_maps_manager.map_path_file", std::string(""));

  node->set_parameters({
    rclcpp::Parameter("routes_maps_manager.map_path_file", std::string(filename)),
    rclcpp::Parameter("routes_maps_manager.package", std::string(""))
  });

  auto manager = std::make_shared<RoutesMapsManager>();
  auto init_result = manager->initialize(node, "routes_maps_manager");
  ASSERT_TRUE(init_result.has_value()) << init_result.error();

  // Activate lifecycle node before plugin-specific initialization.
  auto state = node->configure();
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  state = node->activate();
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto result = manager->on_initialize();
  ASSERT_TRUE(result.has_value()) << result.error();

  easynav::NavState nav_state;
  manager->update(nav_state);

  ASSERT_TRUE(nav_state.has("routes"));
  const auto & routes = nav_state.get<RoutesMap>("routes");
  EXPECT_EQ(routes.size(), 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
