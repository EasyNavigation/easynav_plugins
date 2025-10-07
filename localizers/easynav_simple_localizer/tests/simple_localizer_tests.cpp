// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_simple_localizer/AMCLLocalizer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_srvs/srv/trigger.hpp"

#include <memory>
#include <fstream>

/// \brief Fixture for AMCLLocalizer tests (minimal)
class AMCLLocalizerTest : public ::testing::Test
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


/// \brief Dynamic map update tests
TEST_F(AMCLLocalizerTest, BasicDynamicUpdate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto manager = std::make_shared<easynav::AMCLLocalizer>();
  manager->initialize(node, "test");

  auto static_map = std::make_shared<easynav::SimpleMap>();
  static_map->initialize(30, 30, 0.1, -1.5, -1.5, 0.0);
  manager->set_static_map(static_map);

  easynav::NavState navstate;
  auto perception = std::make_shared<easynav::Perception>();

  perception->data.points.resize(6);
  perception->data.points[0].x = 1.0;
  perception->data.points[0].y = 1.0;
  perception->data.points[0].z = 0.2;
  perception->data.points[1].x = -1.0;
  perception->data.points[1].y = -1.0;
  perception->data.points[1].z = 0.2;
  perception->data.points[2].x = -10.0;
  perception->data.points[2].y = -1.0;
  perception->data.points[2].z = 0.2;
  perception->data.points[3].x = 10.0;
  perception->data.points[3].y = -1.0;
  perception->data.points[3].z = 0.2;
  perception->data.points[4].x = 1.0;
  perception->data.points[4].y = -10.0;
  perception->data.points[4].z = 0.2;
  perception->data.points[5].x = 1.0;
  perception->data.points[5].y = 10.0;
  perception->data.points[5].z = 0.2;

  perception->stamp = rclcpp::Time(0);
  perception->frame_id = "map";
  perception->valid = true;

  navstate.set("perceptions", Perceptions());
  navstate.get_mutable<Perceptions>("perceptions")->push_back(perception);

  manager->update(navstate);

  auto map_ptr = std::dynamic_pointer_cast<easynav::SimpleMap>(manager->get_dynamyc_map());
  ASSERT_TRUE(map_ptr != nullptr);

  auto cell1 = map_ptr->metric_to_cell(1.0, 1.0);
  EXPECT_TRUE(map_ptr->at(cell1.first, cell1.second));

  auto cell2 = map_ptr->metric_to_cell(-1.0, -1.0);
  EXPECT_TRUE(map_ptr->at(cell2.first, cell2.second));
}

/// \brief Map loading via subscription test
TEST_F(AMCLLocalizerTest, IncomingOccupancyGridUpdatesMaps)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node2");
  auto manager = std::make_shared<easynav::AMCLLocalizer>();
  manager->initialize(node, "test2");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "test_node2/test2/incoming_map", rclcpp::QoS(1).transient_local().reliable());

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.resolution = 0.2;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -0.6;
  grid.data.assign(100, 0);
  grid.data[55] = 100;  // Mark cell (5,5) as occupied

  pub->publish(grid);

  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto static_map = std::dynamic_pointer_cast<easynav::SimpleMap>(manager->get_static_map());
  ASSERT_TRUE(static_map != nullptr);

  EXPECT_EQ(static_map->at(5, 5), 1);
  EXPECT_EQ(static_map->at(1, 1), 0);
}

class FriendAMCLLocalizer : public easynav::AMCLLocalizer {
public:
  void force_path(const std::string & path) {map_path_ = path;}
};

TEST_F(AMCLLocalizerTest, SavemapServiceWorks)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_savemap_node");
  auto manager = std::make_shared<easynav::AMCLLocalizer>();
  manager->initialize(node, "test_savemap");

  auto static_map = std::make_shared<easynav::SimpleMap>();
  static_map->initialize(4, 4, 0.5, -1.0, -1.0, 0);
  static_map->at(1, 1) = true;
  static_map->at(2, 2) = true;
  manager->set_static_map(static_map);

  const std::string test_map_file = "/tmp/savemap_test_map.txt";
  const std::string service_name = "/test_savemap_node/test_savemap/savemap";

  std::static_pointer_cast<FriendAMCLLocalizer>(manager)->force_path(test_map_file);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  executor.spin_until_future_complete(future);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("saved"), std::string::npos);

  std::ifstream infile(test_map_file);
  ASSERT_TRUE(infile.is_open());

  std::string first_line;
  std::getline(infile, first_line);
  EXPECT_NE(first_line.find("4 4"), std::string::npos);
  EXPECT_NE(first_line.find("0.5"), std::string::npos);

  infile.close();
  std::remove(test_map_file.c_str());
}
