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

// ObstacleFilter with memory: obstacles persist in log-odds, rays clear them, decay fades them.

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_maps_manager/filters/ObstacleFilter.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using ::navmap::NavCelId;
using ::navmap::NavMap;

namespace
{

constexpr uint8_t LETHAL = navmap_ros::LETHAL_OBSTACLE;
constexpr uint8_t FREE = navmap_ros::FREE_SPACE;
constexpr uint8_t UNKNOWN = navmap_ros::NO_INFORMATION;

// 4 x 4 m at 0.1 m, free, with a static obstacle at (3.55, 2.05) and an unknown strip at x in
// [3.0, 3.3], y in [1.0, 1.2]
nav_msgs::msg::OccupancyGrid grid()
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = 0.1;
  g.info.width = 40;
  g.info.height = 40;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(40 * 40, 0);
  g.data[20 * 40 + 35] = 100;
  for (int i = 30; i < 33; ++i) {
    for (int j = 10; j < 12; ++j) {
      g.data[j * 40 + i] = -1;
    }
  }
  return g;
}

class ObstacleMemoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "obstacle_memory_" + std::to_string(count++));
    auto tf_buffer = easynav::RTTFBuffer::getInstance();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = tf_buffer->get_tf_info().map_frame;
    tf.child_frame_id = tf_buffer->get_tf_info().robot_frame;
    tf.transform.translation.x = 2.0;
    tf.transform.translation.y = 2.0;
    tf.transform.rotation.w = 1.0;
    tf_buffer->setTransform(tf, "test", true);
    nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid()));
  }

  void make(bool memory, double decay = 0.0, bool clearing = true)
  {
    node_->declare_parameter("navmap.obstacles.memory.enabled", memory);
    node_->declare_parameter("navmap.obstacles.memory.decay_time", decay);
    node_->declare_parameter("navmap.obstacles.memory.clearing", clearing);
    filter_.initialize(node_, "navmap.obstacles");
  }

  // One cycle: 0.5 m columns at `cols` and ground points at `ground` (cell centers, map frame).
  // The changes are cleared afterwards, as the maps manager does.
  const NavMap & cycle(
    const std::vector<std::pair<float, float>> & cols,
    const std::vector<std::pair<float, float>> & ground = {})
  {
    easynav::PointPerception p;
    p.frame_id = "map";
    p.stamp = node_->now();
    p.valid = true;
    for (const auto & [x, y] : cols) {
      for (float z = 0.2f; z <= 0.5f; z += 0.1f) {p.data.push_back(pcl::PointXYZ(x, y, z));}
    }
    for (const auto & [x, y] : ground) {p.data.push_back(pcl::PointXYZ(x, y, 0.0f));}
    nav_state_.set("scan", p);
    filter_.update(nav_state_);
    const auto & nm = nav_state_.get<NavMap>("map.navmap");
    last_changed_ = nm.has_layer("obstacles") ? nm.layers.get("obstacles")->changed().size() : 0;
    for (const auto & n : nm.list_layers()) {nm.layers.get(n)->clear_changes();}
    return nm;
  }

  uint8_t at(float x, float y)
  {
    const auto & nm = nav_state_.get<NavMap>("map.navmap");
    std::size_t s;
    NavCelId c;
    Eigen::Vector3f b, h;
    EXPECT_TRUE(nm.locate_navcel(Eigen::Vector3f(x, y, 0.05f), s, c, b, &h));
    return nm.layer_get<uint8_t>("obstacles", c, 123);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
  easynav::navmap::ObstacleFilter filter_;
  std::size_t last_changed_ {0};
};

}  // namespace

TEST_F(ObstacleMemoryTest, OffByDefaultNothingIsRemembered)
{
  filter_.initialize(node_, "navmap.obstacles");
  cycle({{3.03f, 2.03f}});
  EXPECT_EQ(at(3.03f, 2.03f), LETHAL);
  const auto & nm = cycle({});
  EXPECT_EQ(at(3.03f, 2.03f), FREE);
  EXPECT_FALSE(nm.has_layer("obstacles_logodds"));
  EXPECT_FALSE(nm.has_layer("obstacles_sensor"));
  EXPECT_EQ(nm.get_layer_meta("obstacles")->unknown, 255.0);  // NO_INFORMATION is unknown
}

TEST_F(ObstacleMemoryTest, AnObstacleOutOfSightIsRemembered)
{
  make(true);
  const auto & nm = cycle({{3.03f, 2.03f}});
  EXPECT_EQ(at(3.03f, 2.03f), LETHAL);
  EXPECT_TRUE(nm.has_layer("obstacles_logodds"));
  EXPECT_EQ(nm.layer_type_name("obstacles_logodds"), "int16");
  for (int i = 0; i < 5; ++i) {
    cycle({});
                                         }  // nothing seen, no ray crosses it
  EXPECT_EQ(at(3.03f, 2.03f), LETHAL);
  EXPECT_EQ(at(1.03f, 1.03f), FREE);
  EXPECT_EQ(at(3.55f, 2.05f), LETHAL);  // static
}

TEST_F(ObstacleMemoryTest, RaysThroughAnObstacleClearItInAFewCycles)
{
  make(true);
  cycle({{3.03f, 2.03f}});
  // The ground is now seen beyond it, on the same ray from the robot: it was not there anymore
  int cycles = 0;
  while (at(3.03f, 2.03f) == LETHAL && cycles < 10) {
    cycle({}, {{3.53f, 2.03f}});
    ++cycles;
  }
  EXPECT_EQ(at(3.03f, 2.03f), FREE);
  EXPECT_GE(cycles, 2);  // one miss does not undo a hit
  EXPECT_LE(cycles, 3);
  // A cleared cell changes once: later cycles with the same evidence record no change
  cycle({}, {{3.53f, 2.03f}});
  cycle({}, {{3.53f, 2.03f}});
  EXPECT_EQ(last_changed_, 0u);
}

TEST_F(ObstacleMemoryTest, SensorsFillInUnknownButNeverEraseTheStaticMap)
{
  make(true);
  // (Points off the diagonals of the grid cells: each probes one NavCel)
  EXPECT_EQ((cycle({}), at(3.16f, 1.12f)), UNKNOWN);  // unknown in the static map
  // The ground seen on the unknown strip, and beyond the static obstacle
  for (int i = 0; i < 5; ++i) {
    cycle({}, {{3.86f, 2.02f}, {3.16f, 1.12f}});
                                                                           }
  EXPECT_EQ(at(3.16f, 1.12f), FREE);
  EXPECT_EQ(at(3.55f, 2.05f), LETHAL);  // the static obstacle stays
  EXPECT_EQ(at(3.86f, 2.02f), FREE);
  // The sensor layer says what the sensors know
  const auto & nm = nav_state_.get<NavMap>("map.navmap");
  EXPECT_TRUE(nm.get_layer_meta("obstacles_sensor")->unknown.has_value());
}

TEST_F(ObstacleMemoryTest, UnobservedCellsDecayBackToTheStaticMap)
{
  make(true, 0.3);
  cycle({{3.03f, 2.03f}});
  cycle({{3.03f, 2.03f}});
  EXPECT_EQ(at(3.03f, 2.03f), LETHAL);
  // Not observed for longer than the decay time
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(700)) {
    cycle({});
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(at(3.03f, 2.03f), FREE);
  EXPECT_EQ(at(3.55f, 2.05f), LETHAL);
  // Observed cells do not decay
  for (int i = 0; i < 20; ++i) {
    cycle({{1.03f, 3.03f}});
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(at(1.03f, 3.03f), LETHAL);
}

TEST_F(ObstacleMemoryTest, WithoutClearingRaysChangeNothing)
{
  make(true, 0.0, false);
  cycle({{3.03f, 2.03f}});
  for (int i = 0; i < 5; ++i) {
    cycle({}, {{3.53f, 2.03f}});
                                                           }
  EXPECT_EQ(at(3.03f, 2.03f), LETHAL);
}

TEST_F(ObstacleMemoryTest, ANewMapForgetsEverything)
{
  make(true);
  cycle({{3.03f, 2.03f}});
  nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid()));
  const auto & nm = cycle({});
  EXPECT_EQ(at(3.03f, 2.03f), FREE);
  EXPECT_EQ(nm.layer_size("obstacles_logodds"), nm.navcels.size());
}

TEST_F(ObstacleMemoryTest, InvalidProbabilitiesFallBackToTheDefaults)
{
  node_->declare_parameter("navmap.obstacles.memory.p_hit", 0.3);
  node_->declare_parameter("navmap.obstacles.memory.p_miss", 0.9);
  make(true);
  cycle({{3.03f, 2.03f}});
  EXPECT_EQ(at(3.03f, 2.03f), LETHAL);  // a hit is evidence of occupied, whatever was asked
}
