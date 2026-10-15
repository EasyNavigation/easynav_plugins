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

// The filters update the map in NavState in place and write only what changes; the maps manager
// publishes only the changed cells.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_maps_manager/NavMapMapsManager.hpp"
#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"
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

// side_cells x side_cells at 0.1 m, free, with an occupied cell at (1.55, 0.55)
nav_msgs::msg::OccupancyGrid grid(int side_cells = 40)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = 0.1;
  g.info.width = side_cells;
  g.info.height = side_cells;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(side_cells * side_cells, 0);
  g.data[5 * side_cells + 15] = 100;
  return g;
}

std::uint8_t at(const NavMap & nm, const std::string & layer, float x, float y)
{
  std::size_t s;
  NavCelId c;
  Eigen::Vector3f b, h;
  EXPECT_TRUE(nm.locate_navcel(Eigen::Vector3f(x, y, 0.05f), s, c, b, &h));
  return nm.layer_get<std::uint8_t>(layer, c, 123);
}

NavCelId cell(const NavMap & nm, float x, float y)
{
  std::size_t s;
  NavCelId c = 0;
  Eigen::Vector3f b, h;
  nm.locate_navcel(Eigen::Vector3f(x, y, 0.05f), s, c, b, &h);
  return c;
}

std::set<NavCelId> changed(const NavMap & nm, const std::string & layer)
{
  const auto & v = nm.layers.get(layer)->changed();
  return {v.begin(), v.end()};
}

void clear_changes(const NavMap & nm)
{
  for (const auto & n : nm.list_layers()) {
    nm.layers.get(n)->clear_changes();
  }
}

// Exposes the inscribed radius the filter took from the robot geometry
struct Inflation : easynav::navmap::InflationFilter
{
  float inscribed() const {return static_cast<float>(inscribed_radius_);}
};

class IncrementalObstaclesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "incremental_obstacles_" + std::to_string(count++));
    auto tf_buffer = easynav::RTTFBuffer::getInstance();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = tf_buffer->get_tf_info().map_frame;
    tf.child_frame_id = tf_buffer->get_tf_info().robot_frame;
    tf.transform.translation.x = 2.0;
    tf.transform.translation.y = 2.0;
    tf.transform.rotation.w = 1.0;
    tf_buffer->setTransform(tf, "test", true);
    nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid()));
    filter_.initialize(node_, "navmap.obstacles");
  }

  // One cycle seeing a 0.5 m high column at each (x, y) (cell centers), and nothing else
  const NavMap & see(const std::vector<std::pair<float, float>> & cols)
  {
    easynav::PointPerception p;
    p.frame_id = "map";
    p.stamp = node_->now();
    p.valid = true;
    for (const auto & [x, y] : cols) {
      for (float z = 0.2f; z <= 0.5f; z += 0.1f) {p.data.push_back(pcl::PointXYZ(x, y, z));}
    }
    nav_state_.set("scan", p);
    filter_.update(nav_state_);
    return nav_state_.get<NavMap>("map.navmap");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
  easynav::navmap::ObstacleFilter filter_;
};

}  // namespace

TEST_F(IncrementalObstaclesTest, AMovingObstacleChangesOnlyItsCells)
{
  const auto & nm = see({{1.03f, 2.03f}});
  EXPECT_EQ(at(nm, "obstacles", 1.03f, 2.03f), LETHAL);
  EXPECT_EQ(at(nm, "obstacles", 1.55f, 0.55f), LETHAL);  // the static map
  EXPECT_TRUE(nm.layers.get("obstacles")->all_changed());  // first cycle: a new layer
  clear_changes(nm);

  // It moves: the old cell goes back to free, the new one is marked, nothing else is written
  const NavCelId old_c = cell(nm, 1.03f, 2.03f), new_c = cell(nm, 3.03f, 2.03f);
  see({{3.03f, 2.03f}});
  EXPECT_EQ(at(nm, "obstacles", 1.03f, 2.03f), FREE);
  EXPECT_EQ(at(nm, "obstacles", 3.03f, 2.03f), LETHAL);
  EXPECT_EQ(at(nm, "obstacles", 1.55f, 0.55f), LETHAL);
  EXPECT_FALSE(nm.layers.get("obstacles")->all_changed());
  EXPECT_EQ(changed(nm, "obstacles"), (std::set<NavCelId>{old_c, new_c}));
  clear_changes(nm);

  // The same scene again: nothing changes
  see({{3.03f, 2.03f}});
  EXPECT_FALSE(nm.layers.get("obstacles")->has_changes());
  // Nothing seen: the obstacle goes, the static map stays
  see({});
  EXPECT_EQ(changed(nm, "obstacles"), (std::set<NavCelId>{new_c}));
  EXPECT_EQ(at(nm, "obstacles", 3.03f, 2.03f), FREE);
  EXPECT_EQ(at(nm, "obstacles", 1.55f, 0.55f), LETHAL);
}

TEST_F(IncrementalObstaclesTest, SeenObstaclesOnTheStaticMapLeaveItAsItWas)
{
  const auto & nm = see({{1.55f, 0.55f}, {1.53f, 0.57f}});  // on the static obstacle
  clear_changes(nm);
  see({});
  EXPECT_EQ(at(nm, "obstacles", 1.55f, 0.55f), LETHAL);
  EXPECT_FALSE(nm.layers.get("obstacles")->has_changes());  // was lethal, still is
}

TEST_F(IncrementalObstaclesTest, ANewMapRebuildsTheLayer)
{
  see({{1.03f, 2.03f}});
  // A new map with another static obstacle, same geometry
  auto g = grid();
  g.data[5 * 40 + 15] = 0;
  g.data[30 * 40 + 30] = 100;  // (3.05, 3.05)
  nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(g));
  const auto & nm = see({});
  EXPECT_EQ(at(nm, "obstacles", 1.55f, 0.55f), FREE);
  EXPECT_EQ(at(nm, "obstacles", 3.05f, 3.03f), LETHAL);
  EXPECT_EQ(at(nm, "obstacles", 1.03f, 2.03f), FREE);
  EXPECT_TRUE(nm.layers.get("obstacles")->all_changed());
  // A smaller map (other geometry)
  nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid(30)));
  const auto & small = see({{1.03f, 2.03f}});
  EXPECT_EQ(small.layer_size("obstacles"), small.navcels.size());
  EXPECT_EQ(at(small, "obstacles", 1.03f, 2.03f), LETHAL);
  EXPECT_EQ(at(small, "obstacles", 1.55f, 0.55f), LETHAL);
}

TEST(IncrementalInflation, MatchesAFullRecomputationAfterEveryChange)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("incremental_inflation");
  node->declare_parameter("navmap.inflation.inflation_radius", 0.6);
  node->declare_parameter("navmap.inflation.cost_scaling_factor", 3.0);
  Inflation filter;
  filter.initialize(node, "navmap.inflation");

  // A 4 x 4 m map (its border inflates too) with a static block in the middle
  auto g = grid();
  for (int j = 18; j < 22; ++j) {
    for (int i = 18; i < 22; ++i) {
      g.data[j * 40 + i] = 100;
    }
  }
  NavMap base = navmap_ros::from_occupancy_grid(g);
  easynav::NavState nav_state;
  nav_state.set("map.navmap", base);
  auto map = nav_state.get_ptr<NavMap>("map.navmap");
  auto obst = map->add_layer<uint8_t>("obstacles", "", "", FREE);
  const std::size_t N = map->navcels.size();

  std::mt19937 rng(7);
  std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(N - 1));
  std::uniform_int_distribution<int> V(0, 9);
  for (int step = 0; step < 40; ++step) {
    // Some cells become obstacles, unknown or free (a cluster and scattered ones)
    const int n = step == 0 ? 0 : 1 + V(rng) * 3;
    for (int k = 0; k < n; ++k) {
      const NavCelId c = C(rng);
      const int v = V(rng);
      map->layer_set<uint8_t>("obstacles", c, v < 5 ? LETHAL : (v < 7 ? UNKNOWN : FREE));
    }
    const std::vector<uint8_t> before = map->has_layer("inflated_obstacles") ?
      std::dynamic_pointer_cast<::navmap::LayerView<uint8_t>>(
      map->layers.get("inflated_obstacles"))->data() : std::vector<uint8_t>{};
    filter.update(nav_state);

    NavMap ref = *map;
    ASSERT_TRUE(filter.inflate_layer_u8(ref, "obstacles", "ref", 0.6f, 3.0f, filter.inscribed(),
      true));
    const auto inc = map->layers.get("inflated_obstacles");
    ASSERT_TRUE(inc);
    std::set<NavCelId> differ;
    for (NavCelId c = 0; c < N; ++c) {
      ASSERT_EQ(map->layer_get<uint8_t>("inflated_obstacles", c), ref.layer_get<uint8_t>("ref", c))
        << "step " << step << " cell " << c;
      if (!before.empty() && before[c] != map->layer_get<uint8_t>("inflated_obstacles", c)) {
        differ.insert(c);
      }
    }
    // Exactly the cells that changed are recorded (after the first, full, cycle)
    if (step > 0) {
      EXPECT_FALSE(inc->all_changed()) << step;
      EXPECT_EQ(changed(*map, "inflated_obstacles"), differ) << step;
    }
    clear_changes(*map);
  }
}

TEST(IncrementalInflation, ANewMapOrObstacleLayerStartsOver)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("incremental_inflation_new");
  node->declare_parameter("navmap.inflation.inflation_radius", 0.5);
  Inflation filter;
  filter.initialize(node, "navmap.inflation");
  easynav::NavState nav_state;
  // No obstacle layer yet: nothing, quietly
  nav_state.set("map.navmap", navmap_ros::from_occupancy_grid(grid()));
  filter.update(nav_state);
  EXPECT_FALSE(nav_state.get<NavMap>("map.navmap").has_layer("inflated_obstacles"));

  for (const int side : {40, 30, 40}) {
    nav_state.set("map.navmap", navmap_ros::from_occupancy_grid(grid(side)));
    auto map = nav_state.get_ptr<NavMap>("map.navmap");
    map->add_layer<uint8_t>("obstacles", "", "", FREE);
    map->layer_set<uint8_t>("obstacles", cell(*map, 1.03f, 2.03f), LETHAL);
    filter.update(nav_state);
    NavMap ref = *map;
    filter.inflate_layer_u8(ref, "obstacles", "ref", 0.5f, 3.0f, filter.inscribed(), true);
    for (NavCelId c = 0; c < map->navcels.size(); ++c) {
      ASSERT_EQ(map->layer_get<uint8_t>("inflated_obstacles", c), ref.layer_get<uint8_t>("ref", c))
        << side;
    }
    EXPECT_TRUE(map->layers.get("inflated_obstacles")->all_changed());
    clear_changes(*map);
  }
}

TEST(IncrementalMapsManager, PublishesOnlyTheChangedCells)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("incremental_mapsmanager");
  node->declare_parameter("navmap.filters", std::vector<std::string>{"obstacles"});
  node->declare_parameter(
    "navmap.obstacles.plugin",
    std::string("easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter"));
  auto tf_buffer = easynav::RTTFBuffer::getInstance();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = tf_buffer->get_tf_info().map_frame;
  tf.child_frame_id = tf_buffer->get_tf_info().robot_frame;
  tf.transform.translation.x = 2.0;
  tf.transform.translation.y = 2.0;
  tf.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tf, "test", true);

  auto mm = std::make_shared<easynav::navmap::NavMapMapsManager>();
  mm->initialize(node, "navmap");
  easynav::NavState nav_state;
  mm->update(nav_state);  // no map of its own: it publishes its empty one first
  nav_state.set("map.navmap", navmap_ros::from_occupancy_grid(grid()));
  auto scan = [&](std::vector<std::pair<float, float>> cols) {
      easynav::PointPerception p;
      p.frame_id = "map";
      p.stamp = node->now();
      p.valid = true;
      for (const auto & [x, y] : cols) {
        for (float z = 0.2f; z <= 0.5f; z += 0.1f) {
          p.data.push_back(pcl::PointXYZ(x, y, z));
        }
      }
      nav_state.set("scan", p);
    };

  // Nobody listens: nothing is published, but changes are still cleared every cycle
  scan({{1.03f, 2.03f}});
  mm->update(nav_state);
  EXPECT_FALSE(nav_state.get<NavMap>("map.navmap").layers.get("obstacles")->has_changes());

  const std::string topic = "/incremental_mapsmanager/navmap/map_updates";
  std::vector<navmap_ros_interfaces::msg::NavMapLayer> got;
  auto listener = std::make_shared<rclcpp::Node>("incremental_listener");
  auto sub = listener->create_subscription<navmap_ros_interfaces::msg::NavMapLayer>(
    topic, rclcpp::QoS(100),
    [&](navmap_ros_interfaces::msg::NavMapLayer::SharedPtr m) {got.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(listener);
  auto spin = [&]() {
      for (int i = 0; i < 20; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  for (int i = 0; i < 100 && node->count_subscribers(topic) == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto & nm = nav_state.get<NavMap>("map.navmap");
  const std::size_t N = nm.navcels.size();
  ASSERT_EQ(N, 3200u);

  // A new subscriber gets the whole layer, even without changes
  ASSERT_GT(node->count_subscribers(topic), 0u);
  mm->update(nav_state);
  spin();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].name, "obstacles");
  EXPECT_TRUE(got[0].indices.empty());
  EXPECT_EQ(got[0].data_u8.size(), N);
  EXPECT_EQ(got[0].geometry_id, nm.geometry_id());
  const auto full = got[0];

  // No changes: nothing
  got.clear();
  mm->update(nav_state);
  spin();
  EXPECT_TRUE(got.empty());

  // The obstacle moves: only its two cells
  scan({{3.03f, 2.03f}});
  mm->update(nav_state);
  spin();
  ASSERT_EQ(got.size(), 1u);
  const std::set<NavCelId> idx(got[0].indices.begin(), got[0].indices.end());
  EXPECT_EQ(idx, (std::set<NavCelId>{cell(nm, 1.03f, 2.03f), cell(nm, 3.03f, 2.03f)}));
  ASSERT_EQ(got[0].data_u8.size(), 2u);
  // A receiver applying the full layer and then the update ends with the same layer
  NavMap mirror = navmap_ros::from_occupancy_grid(grid());
  navmap_ros::from_msg(full, mirror);
  navmap_ros::from_msg(got[0], mirror);
  for (NavCelId c = 0; c < N; ++c) {
    ASSERT_EQ(mirror.layer_get<uint8_t>("obstacles", c), nm.layer_get<uint8_t>("obstacles", c));
  }
}
