// Copyright 2026 José Miguel Guerrero
//
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
// and sensor points in the NavState. No simulator: a flat NavMap file made here, a box of points.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "easynav_bigmap_maps_manager/BigMapMapsManager.hpp"
#include "easynav_bigmap_maps_manager/dem_grid.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navmap_core/Costs.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

// update() is what EasyNav calls (through internal_update, which also throttles by freq)
class Plugin : public easynav::bigmap::BigMapMapsManager
{
public:
  using easynav::bigmap::BigMapMapsManager::update;
};

// A flat square NavMap of side 2*half m with cells of `res` m, saved to a temporary file
std::string flat_navmap(double half, double res, int hits_at_box = 0)
{
  ::navmap::NavMap nm;
  const int n = static_cast<int>(2 * half / res) + 1;
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      nm.positions.x.push_back(static_cast<float>(-half + i * res));
      nm.positions.y.push_back(static_cast<float>(-half + j * res));
      nm.positions.z.push_back(0.0f);
    }
  }
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "map";
  auto id = [n](int i, int j) {return static_cast<std::uint32_t>(j * n + i);};
  for (int j = 0; j + 1 < n; ++j) {
    for (int i = 0; i + 1 < n; ++i) {
      for (const auto & t : {std::array<std::uint32_t, 3>{id(i, j), id(i + 1, j), id(i + 1, j + 1)},
          std::array<std::uint32_t, 3>{id(i, j), id(i + 1, j + 1), id(i, j + 1)}})
      {
        ::navmap::NavCel c;
        c.v[0] = t[0]; c.v[1] = t[1]; c.v[2] = t[2];
        nm.surfaces[0].navcels.push_back(static_cast<::navmap::NavCelId>(nm.navcels.size()));
        nm.navcels.push_back(c);
      }
    }
  }
  nm.rebuild_geometry_accels();
  if (hits_at_box > 0) {   // a saved map that counted obstacle hits where the test's box is
    nm.add_layer<std::uint8_t>("obstacle_hits", "", "", 0);
    for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
      const auto q = nm.navcel_centroid(c);
      if (std::abs(q.x() - 3.0f) < 0.6f && std::abs(q.y()) < 0.6f) {
        nm.layer_set<std::uint8_t>("obstacle_hits", c, static_cast<std::uint8_t>(hits_at_box));
      }
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

class BigMapPluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    easynav::RTTFBuffer::getInstance()->clear();
    map_file_ = flat_navmap(15.0, 0.5);
    set("terrain.initial_navmap", map_file_);
    set("lengths.cell_size.min", 0.5);
    set("lengths.cell_size.max", 8.0);
    set("lengths.radii.full_res", 6.0);
    set("lengths.radii.low_res_start", 12.0);
    set("lengths.radii.traversed_keep", 12.0);
    set("lengths.radii.obstacle_clearance", 0.0);
    set("dynamic_map.ground_fit.enabled", false);
    set("dynamic_map.confirm_time", 0.05);
    set("dynamic_map.forget_time", 0.4);
    set("output.publish_markers", false);
    set("filters", std::vector<std::string>{"inflation"});
    set("inflation.plugin",
        std::string("easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter"));
    set("inflation.inflation_radius", 1.0);
    set("inflation.cost_scaling_factor", 3.0);
  }

  void TearDown() override {std::filesystem::remove(map_file_);}

  // As a parameter file gives them (overrides: the plugin also tells old names from new ones)
  template<class T>
  void set(const std::string & key, const T & value)
  {
    overrides_.emplace_back("bigmap." + key, rclcpp::ParameterValue(value));
  }

  std::shared_ptr<Plugin> make()
  {
    static int count = 0;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "bigmap_plugin_test_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(overrides_));
    auto p = std::make_shared<Plugin>();
    p->initialize(node_, "bigmap");
    return p;
  }

  void robot_at_origin()
  {
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "map";
    o.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", o);
  }

  // What the robot sees in its footprint frame: a box 0.6 x 0.6 x 1 m centred 3 m ahead, or nothing
  void see(bool box)
  {
    easynav::PointPerception p;
    p.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    p.stamp = node_->now();
    p.valid = true;
    if (box) {
      for (float x = 2.7f; x <= 3.3f; x += 0.05f) {
        for (float y = -0.3f; y <= 0.3f; y += 0.05f) {
          for (float z = 0.05f; z <= 1.0f; z += 0.05f) {p.data.push_back(pcl::PointXYZ(x, y, z));}
        }
      }
    } else {
      p.data.push_back(pcl::PointXYZ(0.5f, 0.0f, 0.0f));   // ground only
    }
    nav_state_.set("scan", p);
  }

  // Runs update() for `seconds` of wall time (the plugin's clock), seeing the box or not
  void run(Plugin & p, bool box, double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      see(box);
      p.update(nav_state_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  // NavCels whose centroid is within half a metre of the box's centre: their values in `layer`
  std::vector<std::uint8_t> at_box(const std::string & layer) const
  {
    const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
    std::vector<std::uint8_t> out;
    for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
      const auto q = nm.navcel_centroid(c);
      if (std::abs(q.x() - 3.0f) < 0.5f && std::abs(q.y()) < 0.5f) {
        out.push_back(nm.layer_get<std::uint8_t>(layer, c, 0));
      }
    }
    return out;
  }

  static std::size_t count_at_least(const std::vector<std::uint8_t> & v, std::uint8_t value)
  {
    std::size_t n = 0;
    for (const auto x : v) {n += x >= value;}
    return n;
  }

  std::vector<rclcpp::Parameter> overrides_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
  std::string map_file_;
  easynav::NavState * nav_state_ref_{nullptr};
};

}  // namespace

TEST_F(BigMapPluginTest, PublishesTheMapWithItsLayersAndTheInflation)
{
  auto p = make();
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  EXPECT_GT(nm.navcels.size(), 100u);
  for (const char * layer : {"occupancy", "obstacles", "obstacle_hits", "traversed", "elevation",
      "lod_size", "inflated_obstacles"})
  {
    EXPECT_TRUE(nm.has_layer(layer)) << layer;
  }
  // flat ground: nothing is an obstacle
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
}

TEST_F(BigMapPluginTest, AnObstacleCountsAfterMinTimeAndStaysOnceFixed)
{
  set("obstacles.memory.min_time", 0.3);
  set("obstacles.memory.max_hits", 2);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.15);   // seen, but for less than min_time: not an obstacle yet
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  run(*p, true, 0.9);    // > 2 x min_time: two hits, fixed
  const auto hits = at_box("obstacle_hits");
  ASSERT_FALSE(hits.empty());
  EXPECT_GT(count_at_least(hits, 2), 0u);
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  // the box goes: a fixed obstacle stays, after its detections are forgotten (forget_time 0.4 s)
  run(*p, false, 1.0);
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  EXPECT_GT(count_at_least(at_box("obstacle_hits"), 2), 0u);
}

TEST_F(BigMapPluginTest, WithoutMemoryAnObstacleGoesWhenItIsNoLongerSeen)
{
  set("obstacles.memory.enabled", false);
  set("dynamic_map.enabled", true);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.5);
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  EXPECT_EQ(count_at_least(at_box("obstacle_hits"), 1), 0u);   // nothing counted
  run(*p, false, 1.0);
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
}

TEST_F(BigMapPluginTest, NoSensorObstaclesWhenNeitherTheDynamicMapNorTheMemoryIsOn)
{
  set("obstacles.memory.enabled", false);
  set("dynamic_map.enabled", false);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.6);
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
}

TEST_F(BigMapPluginTest, TheFilterDetectorWithoutAnObstacleFilterDoesNotStart)
{
  set("obstacles.detector", std::string("navmap_filter"));
  EXPECT_THROW(make(), std::invalid_argument);
}

TEST_F(BigMapPluginTest, TheOldDetectorNameIsStillAccepted)
{
  set("obstacles.source", std::string("dynamic"));
  EXPECT_NO_THROW(make());
}

TEST_F(BigMapPluginTest, SwitchingTheMemoryOffAtRuntimeReleasesTheFixedObstacles)
{
  // what a recovery does to let the obstacles go (dynamic_map off: nothing else analyses sensors)
  set("obstacles.memory.min_time", 0.3);
  set("obstacles.memory.max_hits", 2);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.9);
  run(*p, false, 0.6);   // the box went; fixed: still blocked
  ASSERT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  node_->set_parameter(rclcpp::Parameter("bigmap.obstacles.memory.enabled", false));
  run(*p, false, 0.1);
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  EXPECT_EQ(count_at_least(at_box("obstacle_hits"), 1), 0u);   // not shown: not an obstacle now
  // switched on again: the memory kept its counts, the obstacle is back
  node_->set_parameter(rclcpp::Parameter("bigmap.obstacles.memory.enabled", true));
  run(*p, false, 0.3);
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
}

TEST_F(BigMapPluginTest, TheHitsOfASavedMapComeBackAsObstacles)
{
  set("obstacles.memory.max_hits", 3);
  std::filesystem::remove(map_file_);
  map_file_ = flat_navmap(15.0, 0.5, 3);   // fixed where the box will be
  set("terrain.initial_navmap", map_file_);
  auto p = make();
  robot_at_origin();
  run(*p, false, 0.3);   // nothing is seen there, yet it is an obstacle
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  EXPECT_GT(count_at_least(at_box("obstacle_hits"), 3), 0u);
}

TEST_F(BigMapPluginTest, TheClearServiceForgetsTheFixedObstacles)
{
  set("obstacles.memory.min_time", 0.3);
  set("obstacles.memory.max_hits", 2);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.9);
  run(*p, false, 0.6);   // the box went; fixed: still blocked
  ASSERT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);

  auto client_node = rclcpp::Node::make_shared("bigmap_plugin_test_client");
  auto client = client_node->create_client<std_srvs::srv::Trigger>(
    std::string(node_->get_fully_qualified_name()) + "/bigmap/clear_obstacles");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(client_node);
  exec.add_node(node_->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto result = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    exec.spin_until_future_complete(result, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(result.get()->success);

  run(*p, false, 0.2);   // update() does it in its next cycle
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  EXPECT_EQ(count_at_least(at_box("obstacle_hits"), 1), 0u);
  run(*p, false, 0.6);   // and it does not come back
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
}

TEST_F(BigMapPluginTest, TheInflationSeesTheObstaclesOfTheSameCycleAlsoAfterARemesh)
{
  set("obstacles.memory.min_time", 0.3);
  set("obstacles.memory.max_hits", 1);
  set("lengths.remesh_step", 0.5);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.6);
  ASSERT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  // the robot moves 0.6 m: re-mesh (new NavMap in the NavState) in this very update()
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.x = 0.6;
  o.pose.pose.orientation.w = 1.0;
  nav_state_.set("robot_pose", o);
  see(false);
  p->update(nav_state_);
  // in the same cycle: the fixed obstacle is lethal and the inflation was built on it
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  EXPECT_GT(count_at_least(at_box("inflated_obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
}

namespace
{
// The largest NavCel (lod_size) whose centroid is between r0 and r1 m from (cx, 0)
float coarsest_in_ring(const easynav::NavState & ns, double cx, double r0, double r1)
{
  const auto & nm = ns.get<::navmap::NavMap>("map.navmap");
  float worst = 0.0f;
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto q = nm.navcel_centroid(c);
    const double d = std::hypot(q.x() - cx, q.y());
    if (d >= r0 && d <= r1) {worst = std::max(worst, nm.layer_get<float>("lod_size", c, 0.0f));}
  }
  return worst;
}

void robot_at(easynav::NavState & ns, double x)
{
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.x = x;
  o.pose.pose.orientation.w = 1.0;
  ns.set("robot_pose", o);
}
}  // namespace

TEST_F(BigMapPluginTest, WithAGoalAndNoPathTheFullResolutionCircleGrowsUntilTheGoalIsGone)
{
  set("recovery.no_path_cycles", int64_t{2});
  set("recovery.max_full_res_factor", 2.0);
  auto p = make();
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  const float before = coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0);
  ASSERT_GT(before, 0.5f);   // beyond the 6 m of full resolution: coarser

  nav_msgs::msg::Goals goals;
  goals.goals.emplace_back();
  goals.goals.back().pose.position.x = 14.0;
  nav_state_.set("goals", goals);
  nav_state_.set("path", nav_msgs::msg::Path());
  p->update(nav_state_);
  EXPECT_EQ(coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0), before);   // one cycle is not enough
  p->update(nav_state_);
  const float during = coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0);
  EXPECT_LT(during, before);   // full resolution within 12 m now

  // a path: nothing changes (and no further growth: the cap is 2 x 6 m)
  nav_msgs::msg::Path path;
  path.poses.emplace_back();
  nav_state_.set("path", path);
  for (int i = 0; i < 4; ++i) {
    p->update(nav_state_);
                                                     }
  EXPECT_EQ(coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0), during);

  // the goal is gone: back to normal
  nav_state_.set("goals", nav_msgs::msg::Goals());
  p->update(nav_state_);
  EXPECT_EQ(coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0), before);
}

TEST_F(BigMapPluginTest, TheRecoveryCanBeSwitchedOff)
{
  set("recovery.enabled", false);
  set("recovery.no_path_cycles", int64_t{1});
  auto p = make();
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  const float before = coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0);
  nav_msgs::msg::Goals goals;
  goals.goals.emplace_back();
  nav_state_.set("goals", goals);
  for (int i = 0; i < 4; ++i) {
    p->update(nav_state_);
                                                     }
  EXPECT_EQ(coarsest_in_ring(nav_state_, 0.0, 7.0, 11.0), before);
}

TEST_F(BigMapPluginTest, TheReMeshInAWorkerGivesTheSameMapAsInTheCycle)
{
  auto sync = make();
  const auto sync_node = node_;   // (the plugin does not own its node)
  set("lod.async_remesh", true);
  auto async = make();
  easynav::NavState ns_sync, ns_async;
  for (auto * ns : {&ns_sync, &ns_async}) {
    robot_at(*ns, 0.0);
    easynav::PointPerception ground;   // as see(false)
    ground.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    ground.stamp = node_->now();
    ground.valid = true;
    ground.data.push_back(pcl::PointXYZ(0.5f, 0.0f, 0.0f));
    ns->set("scan", ground);
  }
  sync->update(ns_sync);
  async->update(ns_async);   // the first mesh is made in the cycle
  ASSERT_TRUE(ns_async.has("map.navmap"));
  EXPECT_EQ(ns_async.get<::navmap::NavMap>("map.navmap").navcels.size(),
    ns_sync.get<::navmap::NavMap>("map.navmap").navcels.size());

  for (const double x : {3.0, 6.0, 9.0}) {
    robot_at(ns_sync, x);
    robot_at(ns_async, x);
    sync->update(ns_sync);
    const auto before = ns_async.get<::navmap::NavMap>("map.navmap").navcels.size();
    async->update(ns_async);   // asks for it; the previous map stays meanwhile
    EXPECT_EQ(ns_async.get<::navmap::NavMap>("map.navmap").navcels.size(), before);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    std::size_t now = before;
    while (now == before && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      async->update(ns_async);
      now = ns_async.get<::navmap::NavMap>("map.navmap").navcels.size();
    }
    EXPECT_EQ(now, ns_sync.get<::navmap::NavMap>("map.navmap").navcels.size()) << x;
    EXPECT_EQ(coarsest_in_ring(ns_async, x, 7.0, 11.0), coarsest_in_ring(ns_sync, x, 7.0, 11.0));
  }
  // destroyed with nothing running, or waiting for it: never while the worker reads the plugin
  robot_at(ns_async, 12.0);
  async->update(ns_async);
  async.reset();
}

TEST_F(BigMapPluginTest, ANonFinitePoseIsNoPose)
{
  auto p = make();
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.x = std::nan("");
  o.pose.pose.orientation.w = 1.0;
  nav_state_.set("robot_pose", o);
  see(false);
  p->update(nav_state_);
  EXPECT_FALSE(nav_state_.has("map.navmap"));   // nothing meshed around NaN
  robot_at_origin();
  p->update(nav_state_);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  EXPECT_GT(nav_state_.get<::navmap::NavMap>("map.navmap").navcels.size(), 100u);
}

TEST_F(BigMapPluginTest, InvalidValuesStopTheStartWithAMessage)
{
  set("dynamic_map.ground_fit.min_cells", 0);
  EXPECT_THROW(make(), std::invalid_argument);
}

TEST_F(BigMapPluginTest, TheMapShownHasTheInflationOfTheCycleThatRemovedItsNavCels)
{
  set("obstacles.memory.min_time", 0.3);
  set("obstacles.memory.max_hits", 200);
  set("obstacles.removal.enabled", true);
  auto p = make();
  auto listener = std::make_shared<rclcpp::Node>("bigmap_plugin_test_listener");
  std::vector<navmap_ros_interfaces::msg::NavMap> maps;
  auto sub = listener->create_subscription<navmap_ros_interfaces::msg::NavMap>(
    "/" + std::string(node_->get_name()) + "/bigmap/map", rclcpp::QoS(10).reliable(),
    [&maps](navmap_ros_interfaces::msg::NavMap::ConstSharedPtr m) {maps.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  robot_at_origin();
  // the box is seen for half a second and then gone: its NavCels are removed and then given back
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1500)) {
    see(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500));
    p->update(nav_state_);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  executor.spin_some();
  const std::size_t full = nav_state_.get<::navmap::NavMap>("map.navmap").navcels.size();
  auto costly = [](const navmap_ros_interfaces::msg::NavMap & m) -> std::size_t {
      for (const auto & l : m.layers) {
        if (l.name == "inflated_obstacles") {
          return static_cast<std::size_t>(
            std::count_if(l.data_u8.begin(), l.data_u8.end(), [](auto v) {return v > 0;}));
        }
      }
      return SIZE_MAX;   // the map of the first cycle, before there is an inflation
    };
  // what the map's border alone costs: the smallest inflation of all the maps shown
  std::size_t border = SIZE_MAX;
  for (const auto & m : maps) {
    border = std::min(border, costly(m));
  }
  std::size_t with_holes = 0, without_holes = 0;
  for (const auto & m : maps) {
    if (costly(m) == SIZE_MAX) {continue;}
    // the removed NavCels are lethal: the inflation of the very same map costs around them. A map
    // without them costs the border only, not the inflation of an obstacle just given back
    const bool holes = m.navcels_v0.size() < full;
    (holes ? with_holes : without_holes)++;
    EXPECT_EQ(costly(m) > border, holes) << (holes ? "with" : "without") << " removed NavCels";
  }
  EXPECT_GT(with_holes, 0u);
  EXPECT_GT(without_holes, 0u);
}

TEST_F(BigMapPluginTest, HostileInputsNeverThrowOrLeaveTheMapInvalid)
{
  set("obstacles.memory.min_time", 0.05);
  set("obstacles.memory.max_hits", 2);
  set("lengths.remesh_step", 0.5);
  auto p = make();
  std::mt19937 rng(777);   // fixed seed: a failure is reproducible
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const double poses[][7] = {
    {0, 0, 0, 0, 0, 0, 0},                       // a zero quaternion
    {1e9, 0, 0, 0, 0, 0, 1},                     // far outside the map
    {-1e12, 1e12, 0, 0, 0, 0, 1},
    {0, 0, 1e9, 0, 0, 0, 1},                     // absurd height
    {0, 0, 0, 5, 5, 5, 5},                       // not a unit quaternion
    {nan, 0, 0, 0, 0, 0, 1}, {0, inf, 0, 0, 0, 0, 1}, {0, 0, 0, nan, 0, 0, 1},
  };
  for (int k = 0; k < 160; ++k) {
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "map";
    if (k < 8 * 4) {
      const auto & q = poses[k % 8];
      o.pose.pose.position.x = q[0]; o.pose.pose.position.y = q[1]; o.pose.pose.position.z = q[2];
      o.pose.pose.orientation.x = q[3]; o.pose.pose.orientation.y = q[4];
      o.pose.pose.orientation.z = q[5]; o.pose.pose.orientation.w = q[6];
    } else {
      o.pose.pose.position.x = 12.0 * unit(rng); o.pose.pose.position.y = 12.0 * unit(rng);
      o.pose.pose.orientation.w = 1.0;
    }
    nav_state_.set("robot_pose", o);
    easynav::PointPerception pc;
    pc.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    pc.stamp = node_->now();
    pc.valid = true;
    const int n = static_cast<int>(rng() % 400);   // sometimes none at all
    for (int j = 0; j < n; ++j) {
      float v[3] = {static_cast<float>(8 * unit(rng)), static_cast<float>(8 * unit(rng)),
        static_cast<float>(2 * unit(rng))};
      if (rng() % 17 == 0) {v[rng() % 3] = std::numeric_limits<float>::quiet_NaN();}
      if (rng() % 23 == 0) {v[rng() % 3] = std::numeric_limits<float>::infinity();}
      if (rng() % 29 == 0) {v[rng() % 3] = 3.0e38f;}
      pc.data.push_back(pcl::PointXYZ(v[0], v[1], v[2]));
    }
    nav_state_.set("scan", pc);
    ASSERT_NO_THROW(p->update(nav_state_)) << "cycle " << k;
    if (nav_state_.has("map.navmap")) {
      const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
      for (const auto & name : nm.list_layers()) {
        ASSERT_EQ(nm.layers.get(name)->size(), nm.navcels.size()) << name << " cycle " << k;
      }
      for (const auto & v : nm.positions.z) {
        ASSERT_TRUE(std::isfinite(v)) << "cycle " << k;
      }
    }
  }
}

namespace
{
// A hilly DEM file of `side` x `side` nodes of 1 m: what navmap_gis_tool --dem-out would write.
// `hole` (i0, j0, i1, j1 cells, inclusive): a building, cells without ground (--dem-holes-mesh)
std::string hilly_dem_file(int side, std::array<int, 4> hole = {-1, -1, -1, -1})
{
  bigmap_maps_manager::DemGrid g;
  g.side = side;
  g.resolution = 1.0;
  g.data_resolution = 1.0;
  g.radius = (side - 1) / 2.0;
  g.origin_lat = 40.4;
  g.origin_lon = -3.7;
  g.reference_altitude = 650.0;
  g.source = bigmap_maps_manager::DemGrid::kCopernicus;
  g.z.resize(static_cast<std::size_t>(side) * side);
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      g.z[static_cast<std::size_t>(j) * side + i] =
        static_cast<float>(2.0 * std::sin(0.2 * i) * std::cos(0.15 * j));
    }
  }
  if (hole[0] >= 0) {
    const int nc = side - 1;
    g.cell_valid.assign(static_cast<std::size_t>(nc) * nc, 1);
    for (int j = hole[1]; j <= hole[3]; ++j) {
      for (int i = hole[0]; i <= hole[2]; ++i) {
        g.cell_valid[static_cast<std::size_t>(j) * nc + i] = 0;
      }
    }
  }
  const auto path = (std::filesystem::temp_directory_path() /
    ("bigmap_plugin_test_dem_" + std::to_string(::getpid()) + ".bin")).string();
  if (!g.save(path)) {throw std::runtime_error("cannot write " + path);}
  return path;
}
}  // namespace

TEST_F(BigMapPluginTest, StartsFromADemFileAndPublishesItsTerrainAtTheGpsOrigin)
{
  const auto dem = hilly_dem_file(65);
  overrides_.erase(
    std::remove_if(
      overrides_.begin(), overrides_.end(),
      [](const rclcpp::Parameter & q) {return q.get_name() == "bigmap.terrain.initial_navmap";}),
    overrides_.end());
  set("terrain.dem_file", dem);
  auto p = make();
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  std::filesystem::remove(dem);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  EXPECT_GT(nm.navcels.size(), 50u);
  float lo = 1e9f, hi = -1e9f;
  for (const auto z : nm.positions.z) {
    lo = std::min(lo, z); hi = std::max(hi, z);
                                                                                  }
  EXPECT_GT(hi - lo, 1.0f) << "the hills of the DEM are in the mesh";
  EXPECT_LT(hi, 3.0f);
  EXPECT_GT(lo, -3.0f);
}

TEST_F(BigMapPluginTest, TheHolesOfADemFileAreHolesOfTheMap)
{
  // a 6 x 6 m building at x 8..14, y -2..4 (cells 40..45, 30..35 of a 65-node grid centred on 0)
  const auto dem = hilly_dem_file(65, {40, 30, 45, 35});
  overrides_.erase(
    std::remove_if(
      overrides_.begin(), overrides_.end(),
      [](const rclcpp::Parameter & q) {return q.get_name() == "bigmap.terrain.initial_navmap";}),
    overrides_.end());
  set("terrain.dem_file", dem);
  auto p = make();
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  std::filesystem::remove(dem);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  // whether (x, y) is in the building grown by `m` metres on every side
  const auto in_building = [](double x, double y, double m) {
      return std::abs(x - 11.0) < 3.0 + m && std::abs(y - 1.0) < 3.0 + m;
    };
  std::size_t inside = 0, around = 0;
  for (std::size_t t = 0; t < nm.navcels.size(); ++t) {
    double cx = 0.0, cy = 0.0;
    for (int k = 0; k < 3; ++k) {
      cx += nm.positions.x[nm.navcels[t].v[k]] / 3.0;
      cy += nm.positions.y[nm.navcels[t].v[k]] / 3.0;
    }
    inside += in_building(cx, cy, 0.0);
    around += in_building(cx, cy, 3.0) && !in_building(cx, cy, 0.0);
  }
  EXPECT_EQ(inside, 0u) << "no ground on the building";
  EXPECT_GT(around, 20u) << "the ground around it is there";
}

TEST_F(BigMapPluginTest, AMissingDemFileStopsTheStartWithAMessage)
{
  overrides_.erase(
    std::remove_if(
      overrides_.begin(), overrides_.end(),
      [](const rclcpp::Parameter & q) {return q.get_name() == "bigmap.terrain.initial_navmap";}),
    overrides_.end());
  set("terrain.dem_file", std::string("/nonexistent/dem.bin"));
  set("terrain.dem_wait_timeout", 0.2);
  EXPECT_THROW(make(), std::runtime_error);
}

TEST_F(BigMapPluginTest, TheSaveServicesWriteAMapThatLoadsBack)
{
  const auto dir = std::filesystem::temp_directory_path() /
    ("bigmap_plugin_test_saved_" + std::to_string(::getpid()));
  set("output.save_directory", dir.string());
  auto p = make();
  auto client_node = std::make_shared<rclcpp::Node>("bigmap_plugin_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(node_->get_node_base_interface());   // serves the plugin's services
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  const std::string base = "/" + std::string(node_->get_name()) + "/bigmap/";
  auto call = [&](const std::string & service) {
      auto client = client_node->create_client<std_srvs::srv::Trigger>(base + service);
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
        std::chrono::steady_clock::now() < end)
      {
        executor.spin_some();
        see(false);
        p->update(nav_state_);   // the save is done by update(), in its next cycle
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready ?
             future.get() : nullptr;
    };
  const auto traversed = call("save_traversed");
  ASSERT_NE(traversed, nullptr);
  EXPECT_TRUE(traversed->success) << traversed->message;
  const auto all = call("save_map");
  ASSERT_NE(all, nullptr);
  EXPECT_TRUE(all->success) << all->message;
  // a second save within the same second must not overwrite the first one
  const auto again = call("save_map");
  ASSERT_NE(again, nullptr);
  EXPECT_TRUE(again->success) << again->message;
  for (int k = 0; k < 10; ++k) {
    see(false); p->update(nav_state_);
                                                                  }
  std::size_t cells_all = 0, cells_traversed = 0, files = 0;
  for (const auto & e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() != ".navmap") {continue;}
    ++files;
    ::navmap::NavMap loaded;
    std::error_code ec;
    ASSERT_TRUE(navmap_ros::io::load_from_file(e.path().string(), loaded, &ec)) << ec.message();
    const bool only_traversed = e.path().filename().string().find("traversed") != std::string::npos;
    (only_traversed ? cells_traversed : cells_all) = loaded.navcels.size();
  }
  EXPECT_EQ(files, 3u) << "traversed + two full maps";
  EXPECT_GT(cells_traversed, 0u);
  EXPECT_GT(cells_all, cells_traversed) << "the traversed map is only the ground driven over";
  std::filesystem::remove_all(dir);
}

TEST_F(BigMapPluginTest, TheMarkersAreWhatTheLodPolicyDraws)
{
  set("output.publish_markers", true);
  auto p = make();
  auto listener = std::make_shared<rclcpp::Node>("bigmap_plugin_test_marker_listener");
  std::vector<visualization_msgs::msg::MarkerArray> got;
  auto sub = listener->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/" + std::string(node_->get_name()) + "/bigmap/markers",
    rclcpp::QoS(1).transient_local().reliable(),
    [&got](visualization_msgs::msg::MarkerArray::ConstSharedPtr m) {got.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  for (int k = 0; k < 20; ++k) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(got.empty());
  std::size_t triangles = 0;
  for (const auto & m : got.back().markers) {
    for (const auto & pt : m.points) {
      ASSERT_TRUE(std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z));
    }
    if (m.type == visualization_msgs::msg::Marker::TRIANGLE_LIST) {
      triangles += m.points.size() / 3;
    }
  }
  EXPECT_GT(triangles, 100u);
}

TEST_F(BigMapPluginTest,
  ASubscriberToTheLayerUpdatesGetsTheWholeInflationOnceAndNothingWhileItDoesNotChange)
{
  set("obstacles.removal.enabled", false);   // with removal the layers travel inside the map
  set("obstacles.memory.enabled", false);
  set("dynamic_map.enabled", true);
  auto p = make();
  auto listener = std::make_shared<rclcpp::Node>("bigmap_plugin_test_update_listener");
  std::vector<navmap_ros_interfaces::msg::NavMapLayer> got;
  auto sub = listener->create_subscription<navmap_ros_interfaces::msg::NavMapLayer>(
    "/" + std::string(node_->get_name()) + "/bigmap/map_updates", rclcpp::QoS(100),
    [&got](navmap_ros_interfaces::msg::NavMapLayer::ConstSharedPtr m) {got.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  robot_at_origin();
  // let discovery happen: the plugin only publishes updates when it has a subscriber
  for (int k = 0; k < 30; ++k) {
    see(false);
    p->update(nav_state_);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_FALSE(got.empty());
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  EXPECT_EQ(got.front().name, "inflated_obstacles");
  EXPECT_TRUE(got.front().indices.empty()) << "the first one is the whole layer";
  EXPECT_EQ(got.front().data_u8.size(), nm.navcels.size());
  const auto before = got.size();
  run(*p, true, 0.6);    // nothing reaches `obstacles` without removal: the layer does not change
  for (int k = 0; k < 10; ++k) {
    executor.spin_some();
                                                     }
  EXPECT_EQ(got.size(), before) << "an unchanged layer is not sent again";
}

TEST_F(BigMapPluginTest, LoweringViewMaxZAtRuntimeRepublishesTheMapWithoutTheHighNavCels)
{
  set("output.view_max_z", 1e9);
  auto p = make();
  auto listener = std::make_shared<rclcpp::Node>("bigmap_plugin_test_view_listener");
  std::vector<std::size_t> sizes;
  auto sub = listener->create_subscription<navmap_ros_interfaces::msg::NavMap>(
    "/" + std::string(node_->get_name()) + "/bigmap/map", rclcpp::QoS(10).reliable(),
    [&sizes](navmap_ros_interfaces::msg::NavMap::ConstSharedPtr m) {
      sizes.push_back(m->navcels_v0.size());
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  for (int k = 0; k < 10; ++k) {
    executor.spin_some();
                                                     }
  ASSERT_FALSE(sizes.empty());
  const auto full = sizes.back();
  // the ground is flat at z = 0: everything is above -1
  node_->set_parameter(rclcpp::Parameter("bigmap.output.view_max_z", -1.0));
  see(false);
  p->update(nav_state_);
  for (int k = 0; k < 10; ++k) {
    executor.spin_some();
                                                     }
  EXPECT_LT(sizes.back(), full);
  EXPECT_EQ(sizes.back(), 0u);
}

namespace
{
// Two flat floors of side 2*half m over each other (`gap` m apart, no ramp), cells of `res` m:
// what the hierarchy engine is for. The lower floor is at z = 0.
std::string two_floors_navmap(double half, double res, double gap)
{
  ::navmap::NavMap nm;
  const int n = static_cast<int>(2 * half / res) + 1;
  for (int floor = 0; floor < 2; ++floor) {
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i < n; ++i) {
        nm.positions.x.push_back(static_cast<float>(-half + i * res));
        nm.positions.y.push_back(static_cast<float>(-half + j * res));
        nm.positions.z.push_back(static_cast<float>(floor * gap));
      }
    }
  }
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "map";
  for (int floor = 0; floor < 2; ++floor) {
    auto id = [n, floor](int i, int j) {
        return static_cast<std::uint32_t>(floor * n * n + j * n + i);
      };
    for (int j = 0; j + 1 < n; ++j) {
      for (int i = 0; i + 1 < n; ++i) {
        for (const auto & t : {std::array<std::uint32_t, 3>{id(i, j), id(i + 1, j),
              id(i + 1, j + 1)}, std::array<std::uint32_t, 3>{id(i, j), id(i + 1, j + 1),
              id(i, j + 1)}})
        {
          ::navmap::NavCel c;
          c.v[0] = t[0]; c.v[1] = t[1]; c.v[2] = t[2];
          nm.surfaces[0].navcels.push_back(static_cast<::navmap::NavCelId>(nm.navcels.size()));
          nm.navcels.push_back(c);
        }
      }
    }
  }
  nm.rebuild_geometry_accels();
  const auto path = (std::filesystem::temp_directory_path() /
    ("bigmap_plugin_test_floors_" + std::to_string(::getpid()) + ".navmap")).string();
  std::error_code ec;
  if (!navmap_ros::io::save_to_file(nm, path, {}, &ec)) {
    throw std::runtime_error("cannot write " + path + ": " + ec.message());
  }
  return path;
}
}  // namespace

// The hierarchy engine (multilevel.enabled) through the plugin: two floors over each other, the
// robot on the lower one. The box it sees is an obstacle on its own floor only, the traversed
// ground is marked on its floor only, and a change of floor (z) re-meshes.
TEST_F(BigMapPluginTest, TheHierarchyEngineKeepsTheFloorsApartForObstaclesAndTraversedGround)
{
  std::filesystem::remove(map_file_);
  map_file_ = two_floors_navmap(15.0, 0.5, 3.0);
  set("terrain.initial_navmap", map_file_);
  set("multilevel.enabled", true);
  set("multilevel.vertical_tolerance", 1.0);
  set("obstacles.memory.min_time", 0.3);
  set("obstacles.memory.max_hits", 2);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.9);   // two hits: fixed on the lower floor
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  std::size_t lower = 0, upper = 0, lethal_lower = 0, lethal_upper = 0, traversed_lower = 0,
    traversed_upper = 0;
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto q = nm.navcel_centroid(c);
    const bool up = q.z() > 1.5f;
    (up ? upper : lower)++;
    if (std::abs(q.x() - 3.0f) < 0.5f && std::abs(q.y()) < 0.5f &&
      nm.layer_get<std::uint8_t>("obstacles", c, 0) == ::navmap::LETHAL_OBSTACLE)
    {
      (up ? lethal_upper : lethal_lower)++;
    }
    if (nm.layer_get<std::uint8_t>("traversed", c, 0) == 255) {
      (up ? traversed_upper : traversed_lower)++;
    }
  }
  EXPECT_GT(lower, 100u);
  EXPECT_GT(upper, 100u) << "both floors are in the map (the robot's and the one above)";
  EXPECT_GT(lethal_lower, 0u) << "the box is an obstacle on the robot's floor";
  EXPECT_EQ(lethal_upper, 0u) << "and not on the floor over it";
  EXPECT_GT(traversed_lower, 0u);
  EXPECT_EQ(traversed_upper, 0u) << "the floor above was not driven over";
  // the robot goes up to the other floor (same x, y): a re-mesh in this very cycle
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.z = 3.0;
  o.pose.pose.orientation.w = 1.0;
  nav_state_.set("robot_pose", o);
  see(false);
  p->update(nav_state_);
  const auto & after = nav_state_.get<::navmap::NavMap>("map.navmap");
  std::size_t traversed_upper_after = 0;
  for (::navmap::NavCelId c = 0; c < after.navcels.size(); ++c) {
    if (after.navcel_centroid(c).z() > 1.5f &&
      after.layer_get<std::uint8_t>("traversed", c, 0) == 255)
    {
      ++traversed_upper_after;
    }
  }
  EXPECT_GT(traversed_upper_after, 0u) << "ground driven over on the upper floor";
}

// Obstacle NavCels are painted in the published map: each gets its own three vertices with the
// tint, so a map with lethal NavCels (here: slopes steeper than max_slope_deg) has more vertices
// in the message than the mesh, and vertex colours even without imagery
TEST_F(BigMapPluginTest, ThePublishedMapPaintsTheObstacleNavCelsWithTheirOwnVertices)
{
  // a DEM with a cliff: the grid engine marks its NavCels LETHAL in `occupancy`
  bigmap_maps_manager::DemGrid g;
  g.side = 65;
  g.resolution = 0.5;
  g.data_resolution = 0.5;
  g.radius = 16.0;
  g.z.resize(65 * 65);
  for (int j = 0; j < 65; ++j) {
    for (int i = 0; i < 65; ++i) {
      g.z[static_cast<std::size_t>(j) * 65 + i] = i > 40 ? 4.0f : 0.0f;   // a 4 m step
    }
  }
  const auto dem = (std::filesystem::temp_directory_path() /
    ("bigmap_plugin_test_cliff_" + std::to_string(::getpid()) + ".bin")).string();
  ASSERT_TRUE(g.save(dem));
  overrides_.erase(
    std::remove_if(
      overrides_.begin(), overrides_.end(),
      [](const rclcpp::Parameter & q) {return q.get_name() == "bigmap.terrain.initial_navmap";}),
    overrides_.end());
  set("terrain.dem_file", dem);
  set("obstacles.tint.enabled", true);
  auto p = make();
  auto listener = std::make_shared<rclcpp::Node>("bigmap_plugin_test_tint_listener");
  std::vector<navmap_ros_interfaces::msg::NavMap> maps;
  auto sub = listener->create_subscription<navmap_ros_interfaces::msg::NavMap>(
    "/" + std::string(node_->get_name()) + "/bigmap/map", rclcpp::QoS(10).reliable(),
    [&maps](navmap_ros_interfaces::msg::NavMap::ConstSharedPtr m) {maps.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  std::filesystem::remove(dem);
  for (int k = 0; k < 20 && maps.empty(); ++k) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(maps.empty());
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  std::size_t lethal = 0;
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    lethal += nm.layer_get<std::uint8_t>("occupancy", c, 0) == ::navmap::LETHAL_OBSTACLE;
  }
  ASSERT_GT(lethal, 0u) << "the step is too steep to drive";
  const auto & m = maps.back();
  EXPECT_TRUE(m.has_vertex_rgba);
  EXPECT_EQ(m.positions_x.size(), nm.positions.x.size() + 3 * lethal)
    << "every painted NavCel has its own three vertices";
  EXPECT_EQ(m.colors_r.size(), m.positions_x.size());
  EXPECT_EQ(m.navcels_v0.size(), nm.navcels.size());
  // the painted vertices are the tint (red 255, 40, 40 at 0.65 over grey 170)
  std::size_t red = 0;
  for (std::size_t v = nm.positions.x.size(); v < m.positions_x.size(); ++v) {
    red += m.colors_r[v] > m.colors_g[v] + 50;
  }
  EXPECT_EQ(red, 3 * lethal);
}

// ---- Audit benchmarks (not run by default): colcon test never pays for them. Run with
//   test_bigmap_plugin --gtest_also_run_disabled_tests --gtest_filter='*Benchmark*'
namespace
{
double rss_mb()
{
  std::ifstream f("/proc/self/statm");
  int64_t pages = 0, resident = 0;
  f >> pages >> resident;
  return static_cast<double>(resident) * static_cast<double>(sysconf(_SC_PAGESIZE)) / 1048576.0;
}

double percentile(std::vector<double> v, double p)
{
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
}
}  // namespace

// obstacles.detector navmap_filter: the ObstaclesFilter plugin of easynav_navmap_maps_manager
// detects (through the TF map -> robot), and bigmap removes what it marked and inflates around
TEST_F(BigMapPluginTest, TheFilterDetectorRemovesWhatTheObstacleFilterMarksAndReleasesItAfter)
{
  set("obstacles.detector", std::string("navmap_filter"));
  set("filters", std::vector<std::string>{"obstacle_filter", "inflation"});
  set("obstacle_filter.plugin",
      std::string("easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter"));
  set("lengths.radii.obstacle_clearance", 0.5);
  // the filter measures through TF: the robot stands at the origin, its footprint under it
  const auto info = easynav::RTTFBuffer::getInstance()->get_tf_info();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = info.map_frame;
  tf.child_frame_id = info.robot_frame;
  tf.transform.rotation.w = 1.0;
  easynav::RTTFBuffer::getInstance()->setTransform(tf, "test", true);
  tf.header.frame_id = info.robot_frame;
  tf.child_frame_id = info.robot_footprint_frame;
  easynav::RTTFBuffer::getInstance()->setTransform(tf, "test", true);
  auto p = make();
  robot_at_origin();
  run(*p, true, 0.4);
  ASSERT_TRUE(nav_state_.has("map.navmap"));
  EXPECT_GT(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u)
    << "the filter saw the box and bigmap kept it LETHAL for the planner";
  const auto & nm = nav_state_.get<::navmap::NavMap>("map.navmap");
  ASSERT_TRUE(nm.layers.get("inflated_obstacles"));
  std::size_t inflated_around = 0;
  for (::navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto q = nm.navcel_centroid(c);
    const double d = std::hypot(q.x() - 3.0, q.y());
    if (d > 0.6 && d < 1.2 && nm.layer_get<std::uint8_t>("inflated_obstacles", c, 0) > 0) {
      ++inflated_around;
    }
  }
  EXPECT_GT(inflated_around, 0u) << "the inflation is built on this cycle's obstacles";
  // the published map has the detections removed (holes), with the clearance ring around them
  // nothing in sight: the filter has no memory, the detections and our ring go away
  run(*p, false, 0.4);
  EXPECT_EQ(count_at_least(at_box("obstacles"), ::navmap::LETHAL_OBSTACLE), 0u);
  easynav::RTTFBuffer::getInstance()->clear();
}

// EasyNav serves the services from its non-real-time executor while update() runs in the
// real-time one: the queue between them (mutex, atomic count of traversed nodes) must hold
// while both hammer it. Hundreds of requests from another thread against a robot that drives.
TEST_F(BigMapPluginTest, TheSaveServicesAreSafeToCallFromAnotherThreadWhileTheRobotDrives)
{
  const auto dir = std::filesystem::temp_directory_path() /
    ("bigmap_plugin_test_concurrent_" + std::to_string(::getpid()));
  set("output.save_directory", dir.string());
  auto p = make();
  robot_at_origin();
  see(false);
  p->update(nav_state_);
  std::atomic<bool> stop{false};
  std::atomic<int> answered{0}, accepted{0}, busy{0}, nothing_traversed{0}, other{0};
  const std::string base = "/" + std::string(node_->get_name()) + "/bigmap/";
  // the services' thread: EasyNav's non-real-time executor
  std::thread services([&]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node_->get_node_base_interface());
      while (!stop) {executor.spin_some(std::chrono::milliseconds(2));}
    });
  // a client that asks for a save as often as it can, for traversed ground and for all
  std::thread client([&]() {
      auto client_node = std::make_shared<rclcpp::Node>("bigmap_plugin_test_hammer");
      std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients = {
        client_node->create_client<std_srvs::srv::Trigger>(base + "save_traversed"),
        client_node->create_client<std_srvs::srv::Trigger>(base + "save_map")};
      for (auto & c : clients) {
        c->wait_for_service(std::chrono::seconds(5));
      }
      int k = 0;
      while (!stop) {
        auto & c = clients[k++ % 2];
        auto future = c->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {continue;}
        const auto res = future.get();
        ++answered;
        if (res->success) {
          ++accepted;
        } else if (res->message.find("in progress") != std::string::npos) {
          ++busy;
        } else if (res->message.find("not driven") != std::string::npos) {
          ++nothing_traversed;
        } else {
          ++other;
        }
      }
    });
  // the robot drives (the real-time thread): marks ground, re-meshes, writes the saves
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  int updates = 0;
  for (double t = 0.0; std::chrono::steady_clock::now() < end; t += 0.1, ++updates) {
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "map";
    o.pose.pose.position.x = 8.0 * std::sin(t);
    o.pose.pose.position.y = 8.0 * std::cos(t);
    o.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", o);
    see(false);
    ASSERT_NO_THROW(p->update(nav_state_));
  }
  stop = true;
  client.join();
  services.join();
  EXPECT_GT(updates, 50);
  EXPECT_GT(answered.load(), 20) << "the services did not answer while the robot drove";
  EXPECT_GT(accepted.load(), 0);
  EXPECT_EQ(other.load(), 0) << "an answer that is neither accepted, busy nor 'not driven'";
  // every accepted request was written (or refused by update() with a log, never lost)
  std::size_t files = 0;
  for (const auto & e : std::filesystem::directory_iterator(dir)) {
    files += e.path().extension() == ".navmap";
  }
  EXPECT_GT(files, 0u);
  EXPECT_LE(files, static_cast<std::size_t>(accepted.load()));
  std::filesystem::remove_all(dir);
}

TEST_F(BigMapPluginTest, DISABLED_BenchmarkUpdateAndMemoryWhileTheRobotDrives)
{
  std::filesystem::remove(map_file_);
  const double half =
    std::getenv("BIGMAP_BENCH_HALF") ? std::atof(std::getenv("BIGMAP_BENCH_HALF")) :
    50.0;
  map_file_ = flat_navmap(half, 0.5);
  overrides_.clear();
  set("terrain.initial_navmap", map_file_);
  set("lengths.cell_size.min", 0.5);
  set("lengths.cell_size.max", 8.0);
  set("lengths.radii.full_res", 6.0);
  set("lengths.radii.low_res_start", 12.0);
  set("lengths.radii.traversed_keep", 12.0);
  set("lengths.remesh_step", 0.5);
  set("dynamic_map.ground_fit.enabled", false);
  set("output.publish_markers", false);
  set("output.profile.enabled", std::getenv("BIGMAP_BENCH_PROFILE") != nullptr);
  set("output.profile.period", 3.0);
  set("filters", std::vector<std::string>{"inflation"});
  set("inflation.plugin",
      std::string("easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter"));
  set("inflation.inflation_radius", 1.0);
  set("inflation.cost_scaling_factor", 3.0);
  auto p = make();
  robot_at_origin();
  const int cycles =
    std::getenv("BIGMAP_BENCH_CYCLES") ? std::atoi(std::getenv("BIGMAP_BENCH_CYCLES")) :
    600;
  std::vector<double> ms;
  std::cerr << "cells " << nav_state_.has("map.navmap") << " cycle rss_MB\n";
  for (int k = 0; k < cycles; ++k) {
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "map";
    // a lap around a square of side half: new ground all the time, then the same ground again
    const double s = std::fmod(0.4 * k, 8.0 * (half - 10.0));
    const double side = 2.0 * (half - 10.0);
    const int leg = static_cast<int>(s / side);
    const double t = s - leg * side;
    const double px[4] = {-(half - 10.0) + t, half - 10.0, (half - 10.0) - t, -(half - 10.0)};
    const double py[4] = {-(half - 10.0), -(half - 10.0) + t, half - 10.0, (half - 10.0) - t};
    o.pose.pose.position.x = px[leg % 4];
    o.pose.pose.position.y = py[leg % 4];
    o.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", o);
    see(k % 3 == 0);
    const auto t0 = std::chrono::steady_clock::now();
    p->update(nav_state_);
    ms.push_back(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    if (k % 100 == 0 || k == cycles - 1) {
      std::cerr << "cycle " << k << " rss_MB " << rss_mb() << "\n";
    }
  }
  if (nav_state_.has("map.navmap")) {
    std::cerr << "navcels in the NavState map: "
              << nav_state_.get<::navmap::NavMap>("map.navmap").navcels.size() << "\n";
  }
  std::cerr << "update() ms: median " << percentile(ms, 0.5) << " p95 " << percentile(ms, 0.95)
            << " p99 " << percentile(ms, 0.99) << " max " << percentile(ms, 1.0) << "\n";
}
