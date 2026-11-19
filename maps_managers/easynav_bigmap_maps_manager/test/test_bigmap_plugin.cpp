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

// The plugin as EasyNav runs it: parameters on a node, initialize(), update() with a robot pose
// and what the sensors see

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "easynav_bigmap_maps_manager/BigMapMapsManager.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navmap_core/Costs.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using easynav::bigmap::BigMapMapsManager;
using navmap::NavCelId;
using navmap::NavMap;

namespace
{

// 120 x 120 m at 0.5 m: blocks of 14 x 14 m every 20 m, 6 m streets
std::string town_file()
{
  NavMap nm;
  const auto s = nm.create_surface("map");
  const float res = 0.5f;
  const int n = 240;
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.positions.x.push_back(i * res);
      nm.positions.y.push_back(j * res);
      nm.positions.z.push_back(0.0f);
    }
  }
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      const float bx = std::fmod((i + 0.5f) * res, 20.0f), by = std::fmod((j + 0.5f) * res, 20.0f);
      if (bx > 3.0f && bx < 17.0f && by > 3.0f && by < 17.0f) {continue;}
      const uint32_t a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
      nm.add_navcel_to_surface(s, nm.add_navcel(a, b, d));
      nm.add_navcel_to_surface(s, nm.add_navcel(a, d, c));
    }
  }
  const auto path = (std::filesystem::temp_directory_path() /
    ("bigmap_plugin_test_" + std::to_string(::getpid()) + ".navmap")).string();
  std::error_code ec;
  if (!navmap_ros::io::save_to_file(nm, path, {}, &ec)) {
    throw std::runtime_error("cannot write " + path + ": " + ec.message());
  }
  return path;
}

// 60 x 20 m at 0.5 m: flat up to x = 20, a ramp down to x = 40 (1 in 4: 14 deg), flat again
float ramp_z(float x) {return x < 20.0f ? 0.0f : (x > 40.0f ? -5.0f : -0.25f * (x - 20.0f));}

std::string ramp_file()
{
  NavMap nm;
  const auto s = nm.create_surface("map");
  const float res = 0.5f;
  const int nx = 120, ny = 40;
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      nm.positions.x.push_back(i * res);
      nm.positions.y.push_back(j * res - 10.0f);
      nm.positions.z.push_back(ramp_z(i * res));
    }
  }
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const uint32_t a = j * (nx + 1) + i, b = a + 1, c = a + nx + 1, d = c + 1;
      nm.add_navcel_to_surface(s, nm.add_navcel(a, b, d));
      nm.add_navcel_to_surface(s, nm.add_navcel(a, d, c));
    }
  }
  const auto path = (std::filesystem::temp_directory_path() /
    ("bigmap_plugin_ramp_" + std::to_string(::getpid()) + ".navmap")).string();
  std::error_code ec;
  if (!navmap_ros::io::save_to_file(nm, path, {}, &ec)) {
    throw std::runtime_error("cannot write " + path + ": " + ec.message());
  }
  return path;
}

float longest_edge(const NavMap & nm, NavCelId c)
{
  float l = 0.0f;
  for (int k = 0; k < 3; ++k) {
    l = std::max(
      l, (nm.positions.at(nm.navcels[c].v[k]) -
      nm.positions.at(nm.navcels[c].v[(k + 1) % 3])).norm());
  }
  return l;
}

class BigMapPluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    easynav::RTTFBuffer::getInstance()->clear();
    map_file_ = town_file();
    set("cell_size.min", 0.5);
    set("cell_size.max", 4.0);
    set("min_radius", 10.0);
    set("max_radius", 40.0);
    set("filters", std::vector<std::string>{"inflation"});
    set("inflation.plugin",
        std::string("easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter"));
    set("inflation.inflation_radius", 1.0);
    set("inflation.cost_scaling_factor", 3.0);
  }

  void TearDown() override {std::filesystem::remove(map_file_);}

  template<class T>
  void set(const std::string & key, const T & value)
  {
    overrides_.emplace_back("bigmap." + key, rclcpp::ParameterValue(value));
  }

  std::shared_ptr<BigMapMapsManager> make()
  {
    static int count = 0;
    overrides_.emplace_back("bigmap.navmap_path_file", rclcpp::ParameterValue(map_file_));
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "bigmap_plugin_test_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(overrides_));
    auto p = std::make_shared<BigMapMapsManager>();
    p->initialize(node_, "bigmap");
    return p;
  }

  void robot_at(double x, double y, double z = 0.0)
  {
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "map";
    o.pose.pose.position.x = x;
    o.pose.pose.position.y = y;
    o.pose.pose.position.z = z;
    o.pose.pose.orientation.w = 1.0;   // facing +x
    nav_state_.set("robot_pose", o);
    // and in TF (the perceptions are taken to the map frame with it)
    const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = tf_info.map_frame;
    t.child_frame_id = tf_info.robot_footprint_frame;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    t.transform.rotation.w = 1.0;
    easynav::RTTFBuffer::getInstance()->setTransform(t, "test", true);
  }

  // What the robot sees in its footprint frame: a box 0.6 x 0.6 x 1 m 3 m ahead, or the ground
  // (with ground_ahead, the ground from 1 to 6 m ahead, 2 m wide, at height 0 in that frame)
  void see(bool box)
  {
    easynav::PointPerception p;
    p.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    p.stamp = node_->now();
    p.valid = true;
    if (ground_ahead_) {
      for (float x = 1.0f; x <= 6.0f; x += 0.1f) {
        for (float y = -1.0f; y <= 1.0f; y += 0.1f) {p.data.push_back(pcl::PointXYZ(x, y, 0.0f));}
      }
    }
    if (box) {
      for (float x = 2.7f; x <= 3.3f; x += 0.05f) {
        for (float y = -0.3f; y <= 0.3f; y += 0.05f) {
          for (float z = 0.05f; z <= 1.0f; z += 0.05f) {p.data.push_back(pcl::PointXYZ(x, y, z));}
        }
      }
    } else {
      p.data.push_back(pcl::PointXYZ(0.5f, 0.0f, 0.0f));
    }
    nav_state_.set("scan", p);
  }

  // update() for `seconds` of wall time (the plugin's clock)
  void run(BigMapMapsManager & p, bool box, double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      see(box);
      p.update(nav_state_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  const NavMap & map() const {return nav_state_.get<NavMap>("map.navmap");}

  // The NavCel at (x, y)
  bool at(double x, double y, NavCelId & c) const
  {
    std::size_t s;
    Eigen::Vector3f bary;
    return map().locate_navcel(
      Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 0.0f), s, c, bary, nullptr);
  }

  std::string map_file_;
  bool ground_ahead_ = false;
  std::vector<rclcpp::Parameter> overrides_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

}  // namespace

TEST_F(BigMapPluginTest, FineAtTheBordersNearTheRobotCoarseFar)
{
  auto p = make();
  ASSERT_EQ(p->lod().levels(), 4);   // 0.5, 1, 2, 4 m
  robot_at(10.0, 20.0);              // in a street
  p->update(nav_state_);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  const auto & nm = map();
  EXPECT_LT(nm.navcels.size(), 20000u);   // the town has 49 000
  // Next to a building near the robot: small NavCels
  NavCelId c;
  ASSERT_TRUE(at(10.0, 17.2, c));
  EXPECT_LE(longest_edge(nm, c), 0.5f + 1e-3f);
  // In the middle of a street crossing far away: large ones
  ASSERT_TRUE(at(100.0, 100.0, c));
  EXPECT_GT(longest_edge(nm, c), 2.0f);
  // The filters ran on it
  EXPECT_TRUE(nm.has_layer("inflated_obstacles"));
  EXPECT_TRUE(nm.has_layer("obstacles"));
}

TEST_F(BigMapPluginTest, TheMapFollowsTheRobot)
{
  auto p = make();
  robot_at(10.0, 20.0);
  p->update(nav_state_);
  // 0.2 m from the block of x, y in [103, 117]
  NavCelId c;
  ASSERT_TRUE(at(104.0, 102.8, c));
  EXPECT_GT(longest_edge(map(), c), 0.5f + 1e-3f) << "far from the robot";
  robot_at(100.0, 100.0);
  run(*p, false, 0.7);   // the map is made again at most every 0.5 s
  ASSERT_TRUE(at(104.0, 102.8, c));
  EXPECT_LE(longest_edge(map(), c), 0.5f + 1e-3f) << "next to the robot now";
}

TEST_F(BigMapPluginTest, ASensedObstacleIsAddedFineAndForgottenFarAway)
{
  auto p = make();
  robot_at(10.0, 20.0);   // facing +x along the street: the box at (13, 20)
  run(*p, true, 0.8);
  ASSERT_GT(p->sensed().size(), 0u);
  NavCelId c;
  ASSERT_TRUE(at(13.0, 20.0, c));
  EXPECT_EQ(map().layer_get<std::uint8_t>("obstacles", c), navmap::LETHAL_OBSTACLE);
  // what is marked is fine (it is near the robot)
  std::size_t marked = 0;
  for (NavCelId k = 0; k < map().navcels.size(); ++k) {
    if (map().layer_get<std::uint8_t>("obstacles", k) != navmap::LETHAL_OBSTACLE) {continue;}
    ++marked;
    EXPECT_LE(longest_edge(map(), k), 0.5f + 1e-3f);
  }
  EXPECT_GT(marked, 2u);
  ASSERT_TRUE(at(13.0, 20.9, c));   // 0.6 m from the box
  EXPECT_GT(map().layer_get<std::uint8_t>("inflated_obstacles", c), 0) << "inflated around it";
  ASSERT_TRUE(at(13.0, 22.8, c));
  EXPECT_EQ(map().layer_get<std::uint8_t>("obstacles", c), 0) << "only where it is";

  // Farther than forget_distance (max_radius: 40 m) it is forgotten
  robot_at(100.0, 100.0);
  run(*p, false, 0.7);
  EXPECT_EQ(p->sensed().size(), 0u);
  robot_at(10.0, 20.0);
  run(*p, false, 0.7);
  ASSERT_TRUE(at(13.0, 20.0, c));
  EXPECT_EQ(map().layer_get<std::uint8_t>("obstacles", c), 0);
}

TEST_F(BigMapPluginTest, AnObstacleThatMovesAwayIsCleared)
{
  auto p = make();
  robot_at(10.0, 20.0);   // facing +x along the street: the box at (13, 20)
  run(*p, true, 0.8);
  ASSERT_GT(p->sensed().size(), 0u);
  NavCelId c;
  ASSERT_TRUE(at(13.0, 20.0, c));
  ASSERT_EQ(map().layer_get<std::uint8_t>("obstacles", c), navmap::LETHAL_OBSTACLE);
  // It goes away: the robot sees the ground where it was (rays through its cells)
  ground_ahead_ = true;
  run(*p, false, 0.8);
  EXPECT_EQ(p->sensed().size(), 0u);
  ASSERT_TRUE(at(13.0, 20.0, c));
  EXPECT_NE(map().layer_get<std::uint8_t>("obstacles", c), navmap::LETHAL_OBSTACLE);
}

TEST_F(BigMapPluginTest, WithoutClearingAnObstacleIsRemembered)
{
  set("memory.clearing", false);
  auto p = make();
  robot_at(10.0, 20.0);
  run(*p, true, 0.8);
  ASSERT_GT(p->sensed().size(), 0u);
  ground_ahead_ = true;
  run(*p, false, 0.8);
  EXPECT_GT(p->sensed().size(), 0u) << "only the distance forgets it";
}

TEST_F(BigMapPluginTest, AnObstacleNotSeenAgainDecays)
{
  set("memory.decay_time", 0.3);
  auto p = make();
  robot_at(10.0, 20.0);
  run(*p, true, 0.5);
  ASSERT_GT(p->sensed().size(), 0u);
  // Out of sight (the sensor sees only the ground at its feet): forgotten after 0.3 s
  run(*p, false, 0.8);
  EXPECT_EQ(p->sensed().size(), 0u);
}

TEST_F(BigMapPluginTest, GoingDownARampTheRampAheadIsNotAnObstacle)
{
  // On a ramp the robot is tilted like it, and its footprint frame (flat in TF) too: the ramp
  // ahead is at height 0 in it, below the robot's own ground height
  std::filesystem::remove(map_file_);
  map_file_ = ramp_file();
  auto p = make();
  ground_ahead_ = true;
  robot_at(25.0, 0.0, ramp_z(25.0f));   // on the ramp, facing down it (+x)
  run(*p, false, 0.8);
  EXPECT_EQ(p->sensed().size(), 0u);
  // An obstacle on the ramp is still seen
  robot_at(25.0, 0.0, ramp_z(25.0f) + 0.01);
  run(*p, true, 0.8);
  EXPECT_GT(p->sensed().size(), 0u);
}

TEST_F(BigMapPluginTest, ThePublishedMapHasTheFiltersLayersAndTheLevelsAreDrawn)
{
  auto p = make();
  auto listener = rclcpp::Node::make_shared("bigmap_plugin_test_listener");
  navmap_ros_interfaces::msg::NavMap::SharedPtr got;
  visualization_msgs::msg::MarkerArray::SharedPtr markers;
  const std::string base = std::string(node_->get_fully_qualified_name()) + "/bigmap";
  auto sub = listener->create_subscription<navmap_ros_interfaces::msg::NavMap>(
    base + "/map", rclcpp::QoS(1).transient_local().reliable(),
    [&](navmap_ros_interfaces::msg::NavMap::SharedPtr m) {got = m;});
  auto msub = listener->create_subscription<visualization_msgs::msg::MarkerArray>(
    base + "/markers", rclcpp::QoS(1).transient_local().reliable(),
    [&](visualization_msgs::msg::MarkerArray::SharedPtr m) {markers = m;});
  robot_at(10.0, 20.0);
  p->update(nav_state_);
  for (int i = 0; i < 50 && (!got || !markers); ++i) {
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(got);
  bool inflated = false;
  for (const auto & l : got->layers) {
    inflated |= l.name == "inflated_obstacles";
  }
  EXPECT_TRUE(inflated) << "the map is published after the filters";
  ASSERT_TRUE(markers);
  std::size_t circles = 0;
  for (const auto & m : markers->markers) {
    circles += m.ns == "level radii" && m.type == visualization_msgs::msg::Marker::LINE_STRIP;
  }
  EXPECT_EQ(circles, 3u);   // where levels 1, 2 and 3 start
}
