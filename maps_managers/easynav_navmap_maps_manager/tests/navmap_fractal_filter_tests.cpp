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

#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros_interfaces/msg/nav_map_patch.hpp"
#include "navmap_ros/conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_navmap_maps_manager/filters/FractalFilter.hpp"

namespace
{

constexpr int kCells = 64;          // 6.4 x 6.4 m at 0.1 m
constexpr const char * kLayer = "inflated_obstacles";

}  // namespace

class NavmapFractalFilterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.1;
    grid.info.width = kCells;
    grid.info.height = kCells;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(kCells * kCells, 0);
    auto nm = navmap_ros::from_occupancy_grid(grid);
    for (std::size_t c = 0; c < nm.navcels.size(); ++c) {
      nm.layer_set<std::uint8_t>(kLayer, static_cast<::navmap::NavCelId>(c), 0);
    }
    source_navcels_ = nm.navcels.size();
    nav_state_.set("map.navmap", nm);
    robot_at(0.5, 0.5);
  }

  std::shared_ptr<easynav::navmap::FractalFilter> make_filter(
    std::vector<rclcpp::Parameter> overrides = {})
  {
    overrides.emplace_back("navmap.fractal.rebuild_period", 0.0);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "fractal_filter_test_node", rclcpp::NodeOptions().parameter_overrides(overrides));
    auto filter = std::make_shared<easynav::navmap::FractalFilter>();
    filter->initialize(node_, "navmap.fractal");
    return filter;
  }

  void robot_at(double x, double y)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    nav_state_.set("robot_pose", odom);
  }

  // Sets the cost of the source NavCel under (x, y), in place, as a filter does.
  void cost_at(float x, float y, std::uint8_t value)
  {
    auto nm = nav_state_.get_ptr<::navmap::NavMap>("map.navmap");
    std::size_t sidx = 0;
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;
    ASSERT_TRUE(nm->locate_navcel(Eigen::Vector3f(x, y, 0.0f), sidx, cid, bary, &hit));
    std::const_pointer_cast<::navmap::NavMap>(nm)->layer_set<std::uint8_t>(kLayer, cid, value);
  }

  // NavCels of the output that are not dead slots.
  std::size_t live_navcels()
  {
    const auto & out = nav_state_.get<::navmap::NavMap>("map.navmap.lod");
    std::size_t n = 0;
    for (const auto & c : out.navcels) {
      n += !(c.v[0] == c.v[1] && c.v[1] == c.v[2]);
    }
    return n;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
  std::size_t source_navcels_{0};
};

TEST_F(NavmapFractalFilterTest, NothingWithoutTheMap)
{
  auto filter = make_filter();
  easynav::NavState empty;
  filter->update(empty);
  EXPECT_FALSE(empty.has("map.navmap.lod"));
}

TEST_F(NavmapFractalFilterTest, WaitsForItsLayer)
{
  auto filter = make_filter({{"navmap.fractal.layer", "costs"}});
  filter->update(nav_state_);
  EXPECT_FALSE(nav_state_.has("map.navmap.lod"));
}

TEST_F(NavmapFractalFilterTest, AUniformMapMergesAndTheSourceIsUntouched)
{
  auto filter = make_filter();
  const auto geometry = nav_state_.get<::navmap::NavMap>("map.navmap").geometry_id();
  filter->update(nav_state_);
  ASSERT_TRUE(nav_state_.has("map.navmap.lod"));
  EXPECT_LT(live_navcels(), source_navcels_ / 100);
  const auto & src = nav_state_.get<::navmap::NavMap>("map.navmap");
  EXPECT_EQ(src.navcels.size(), source_navcels_);
  EXPECT_EQ(src.geometry_id(), geometry);
}

TEST_F(NavmapFractalFilterTest, ACostNearTheRobotSplitsAndMergesBack)
{
  auto filter = make_filter();
  filter->update(nav_state_);
  const auto merged = live_navcels();

  cost_at(1.05f, 1.05f, 200);
  filter->update(nav_state_);
  const auto split = live_navcels();
  EXPECT_GT(split, merged);

  cost_at(1.05f, 1.05f, 0);
  filter->update(nav_state_);
  EXPECT_EQ(live_navcels(), merged);
}

TEST_F(NavmapFractalFilterTest, TheOutputIsEditedInPlace)
{
  auto filter = make_filter();
  filter->update(nav_state_);
  const auto first = nav_state_.get_ptr<::navmap::NavMap>("map.navmap.lod");
  cost_at(1.05f, 1.05f, 200);
  filter->update(nav_state_);
  EXPECT_EQ(nav_state_.get_ptr<::navmap::NavMap>("map.navmap.lod").get(), first.get());
}

TEST_F(NavmapFractalFilterTest, OutsideTheWindowOnlyTheSweepSeesChanges)
{
  // Window of 1 m around (0.5, 0.5), no sweep: a cost 5 m away is not read
  auto filter = make_filter({{"navmap.fractal.window_radius", 1.0}, {"navmap.fractal.sweep", 0}});
  filter->update(nav_state_);
  const auto merged = live_navcels();
  cost_at(5.55f, 5.55f, 200);
  filter->update(nav_state_);
  EXPECT_EQ(live_navcels(), merged);

  // The robot gets there: now it is
  robot_at(5.5, 5.5);
  filter->update(nav_state_);
  EXPECT_GT(live_navcels(), merged);
}

TEST_F(NavmapFractalFilterTest, TheSweepReadsTheWholeMapInTurns)
{
  auto filter = make_filter(
    {{"navmap.fractal.window_radius", 1.0}, {"navmap.fractal.sweep", 1000}});
  filter->update(nav_state_);
  const auto merged = live_navcels();
  cost_at(5.55f, 5.55f, 200);
  // 8192 source NavCels at 1000 per cycle: read within 9 cycles
  for (int i = 0; i < 9; ++i) {
    filter->update(nav_state_);
  }
  EXPECT_GT(live_navcels(), merged);
}

TEST_F(NavmapFractalFilterTest, TheOutputCannotBeTheSourceMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "fractal_filter_bad_key_node",
    rclcpp::NodeOptions().parameter_overrides({{"navmap.fractal.output_key", "map.navmap"}}));
  easynav::navmap::FractalFilter filter;
  EXPECT_THROW(filter.initialize(node, "navmap.fractal"), std::invalid_argument);
}

TEST_F(NavmapFractalFilterTest, ANewSourceMapRebuildsTheTree)
{
  auto filter = make_filter();
  filter->update(nav_state_);
  ASSERT_TRUE(nav_state_.has("map.navmap.lod"));

  // A smaller map (e.g. loaded by the user) replaces the source
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1;
  grid.info.width = 16;
  grid.info.height = 16;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(16 * 16, 0);
  auto nm = navmap_ros::from_occupancy_grid(grid);
  for (std::size_t c = 0; c < nm.navcels.size(); ++c) {
    nm.layer_set<std::uint8_t>(kLayer, static_cast<::navmap::NavCelId>(c), 0);
  }
  nav_state_.set("map.navmap", nm);
  filter->update(nav_state_);
  const auto & out = nav_state_.get<::navmap::NavMap>("map.navmap.lod");
  ASSERT_FALSE(out.positions.x.empty());
  float max_x = 0.0f;
  for (float x : out.positions.x) {
    max_x = std::max(max_x, x);
  }
  EXPECT_NEAR(max_x, 1.6f, 1e-3) << "the output covers the new map only";
}

// ─── Publication: the whole map once, then patches ───────────────────────────────────────────

class NavmapFractalPublishTest : public NavmapFractalFilterTest
{
protected:
  using MapMsg = navmap_ros_interfaces::msg::NavMap;
  using PatchMsg = navmap_ros_interfaces::msg::NavMapPatch;

  void subscribe(const std::string & name)
  {
    listener_ = std::make_shared<rclcpp::Node>(name);
    const std::string topic = "/fractal_filter_test_node/navmap/fractal/map";
    map_sub_ = listener_->create_subscription<MapMsg>(
      topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](MapMsg::ConstSharedPtr m) {kept_ = *m; ++maps_;});
    patch_sub_ = listener_->create_subscription<PatchMsg>(
      topic + "_patch", rclcpp::QoS(10).reliable(),
      [this](PatchMsg::ConstSharedPtr p) {
        ++patches_;
        applied_ += navmap_ros::apply_patch(*p, kept_) ? 1 : 0;
      });
  }

  // One update of the filter, then what arrives
  void cycle(easynav::navmap::FractalFilter & filter)
  {
    filter.update(nav_state_);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(listener_);
    for (int i = 0; i < 20; ++i) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Until the filter's publishers see the listener
  void wait_matched(easynav::navmap::FractalFilter & filter)
  {
    (void)filter;
    for (int i = 0; i < 200; ++i) {
      if (node_->count_subscribers("/fractal_filter_test_node/navmap/fractal/map") > 0 &&
        node_->count_subscribers("/fractal_filter_test_node/navmap/fractal/map_patch") > 0)
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<MapMsg>::SharedPtr map_sub_;
  rclcpp::Subscription<PatchMsg>::SharedPtr patch_sub_;
  MapMsg kept_;
  int maps_{0}, patches_{0}, applied_{0};
};

TEST_F(NavmapFractalPublishTest, AMapThenPatchesThatKeepTheCopyExact)
{
  auto filter = make_filter(
    {{"navmap.fractal.publish_period", 0.0}, {"navmap.fractal.full_period", 0.0},
      {"navmap.fractal.tile_size", 1.0}});
  subscribe("fractal_listener_a");
  wait_matched(*filter);
  cycle(*filter);
  ASSERT_EQ(maps_, 1) << "the whole map first";
  EXPECT_EQ(patches_, 0);

  const float spots[][2] = {{1.05f, 1.05f}, {0.35f, 0.75f}, {1.45f, 0.25f}, {0.85f, 1.35f}};
  int k = 0;
  for (const auto & xy : spots) {
    cost_at(xy[0], xy[1], static_cast<std::uint8_t>(50 + 40 * k++));
    cycle(*filter);
  }
  cost_at(1.05f, 1.05f, 0);
  cycle(*filter);

  EXPECT_EQ(maps_, 1) << "no whole map again";
  EXPECT_GE(patches_, 4);
  EXPECT_EQ(applied_, patches_) << "every patch fitted the copy";
  const auto full = navmap_ros::to_msg(nav_state_.get<::navmap::NavMap>("map.navmap.lod"));
  EXPECT_EQ(kept_.geometry_id, full.geometry_id);
  EXPECT_EQ(kept_.navcels_v0, full.navcels_v0);
  EXPECT_EQ(kept_.navcels_v1, full.navcels_v1);
  EXPECT_EQ(kept_.navcels_v2, full.navcels_v2);
  EXPECT_EQ(kept_.positions_x, full.positions_x);
  ASSERT_EQ(kept_.layers.size(), full.layers.size());
  for (std::size_t i = 0; i < full.layers.size(); ++i) {
    EXPECT_EQ(kept_.layers[i].data_u8, full.layers[i].data_u8) << full.layers[i].name;
  }
}

TEST_F(NavmapFractalPublishTest, NothingChangedNothingSent)
{
  auto filter = make_filter({{"navmap.fractal.publish_period", 0.0}});
  subscribe("fractal_listener_b");
  wait_matched(*filter);
  cycle(*filter);
  ASSERT_EQ(maps_, 1);
  for (int i = 0; i < 5; ++i) {
    cycle(*filter);
                                              }
  EXPECT_EQ(maps_, 1);
  EXPECT_EQ(patches_, 0);
}

TEST_F(NavmapFractalPublishTest, ANewSubscriberGetsTheWholeMap)
{
  auto filter = make_filter({{"navmap.fractal.publish_period", 0.0}});
  subscribe("fractal_listener_c");
  wait_matched(*filter);
  cycle(*filter);
  ASSERT_EQ(maps_, 1);

  // A second receiver joins: everyone gets the whole map, so it has a base for the patches
  auto other = std::make_shared<rclcpp::Node>("fractal_listener_d");
  int other_maps = 0;
  auto sub = other->create_subscription<PatchMsg>(
    "/fractal_filter_test_node/navmap/fractal/map_patch", rclcpp::QoS(10).reliable(),
    [](PatchMsg::ConstSharedPtr) {});
  auto sub2 = other->create_subscription<MapMsg>(
    "/fractal_filter_test_node/navmap/fractal/map", rclcpp::QoS(1).transient_local().reliable(),
    [&other_maps](MapMsg::ConstSharedPtr) {++other_maps;});
  for (int i = 0; i < 200 &&
    node_->count_subscribers("/fractal_filter_test_node/navmap/fractal/map") < 2; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  cycle(*filter);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(other);
  for (int i = 0; i < 20; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(maps_, 2);
  EXPECT_GE(other_maps, 1);
}
