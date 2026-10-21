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

// A* over the terrain: slope cost, limits of slope, step, roughness and free height, and links
// between surfaces.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "navmap_core/NavMap.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_planner/AStarPlanner.hpp"

using ::navmap::NavCelId;
using ::navmap::NavMap;

namespace
{

using Height = std::function<float(float, float)>;

// nx x ny cells of 0.1 m from (x0, y0), at height h, as one surface; cells where keep(x, y)
void add_grid(
  NavMap & nm, int nx, int ny, float x0, float y0, const Height & h,
  const std::function<bool(float, float)> & keep = nullptr)
{
  const float r = 0.1f;
  const std::size_t s = nm.create_surface("map");
  const auto base = static_cast<uint32_t>(nm.positions.size());
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      const float x = x0 + i * r, y = y0 + j * r;
      nm.add_vertex({x, y, h(x, y)});
    }
  }
  auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (nx + 1) + i);};
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      if (keep && !keep(x0 + (i + 0.5f) * r, y0 + (j + 0.5f) * r)) {continue;}
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
}

float flat(float, float) {return 0.0f;}

class TerrainPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }

  void make(const std::vector<rclcpp::Parameter> & params = {})
  {
    static int count = 0;
    rclcpp::NodeOptions opts;
    for (const auto & p : params) {
      opts.append_parameter_override("planner." + p.get_name(), p.get_parameter_value());
    }
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "navmap_terrain_test_" + std::to_string(count++), opts);
    planner_ = std::make_shared<easynav::navmap::AStarPlanner>();
    planner_->initialize(node_, "planner");
  }

  void set_map(NavMap nm)
  {
    nm.rebuild_geometry_accels();
    nm.add_layer<uint8_t>("obstacles", "", "", 0);
    nav_state_.set("map.navmap", nm);
  }

  nav_msgs::msg::Path plan(float sx, float sy, float sz, float gx, float gy, float gz)
  {
    nav_msgs::msg::Odometry robot;
    robot.header.frame_id = "map";
    robot.pose.pose.position.x = sx;
    robot.pose.pose.position.y = sy;
    robot.pose.pose.position.z = sz;
    robot.pose.pose.orientation.w = 1.0;
    nav_state_.set("robot_pose", robot);
    nav_msgs::msg::Goals goals;
    goals.header.frame_id = "map";
    goals.header.stamp = node_->now();
    geometry_msgs::msg::PoseStamped g;
    g.header = goals.header;
    g.pose.position.x = gx;
    g.pose.position.y = gy;
    g.pose.position.z = gz;
    g.pose.orientation.w = 1.0;
    goals.goals.push_back(g);
    nav_state_.set("goals", goals);
    planner_->update(nav_state_);
    return nav_state_.get<nav_msgs::msg::Path>("path");
  }

  static double max_abs_dy(const nav_msgs::msg::Path & p, double y)
  {
    double m = 0.0;
    for (const auto & ps : p.poses) {m = std::max(m, std::abs(ps.pose.position.y - y));}
    return m;
  }

  static double max_z(const nav_msgs::msg::Path & p)
  {
    double m = -1e9;
    for (const auto & ps : p.poses) {m = std::max(m, ps.pose.position.z);}
    return m;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::navmap::AStarPlanner> planner_;
  easynav::NavState nav_state_;
};

// 6 x 3 m with a 1 m high bump in the middle (radius about 0.8 m)
float hill(float x, float y)
{
  const float d2 = (x - 3.0f) * (x - 3.0f) + (y - 1.5f) * (y - 1.5f);
  return std::exp(-d2 / 0.35f);
}

// A ridge across the whole width at x = 3: up 0.6 m at 40 deg, a plateau, and down again
float ridge(float x, float)
{
  const float k = std::tan(40.0f * static_cast<float>(M_PI) / 180.0f);
  return std::clamp(std::min((x - 2.5f) * k, (3.8f - x) * k), 0.0f, 0.6f);
}

}  // namespace

TEST_F(TerrainPlannerTest, ByDefaultTheTerrainIsIgnored)
{
  make();
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, hill);
  set_map(nm);
  const auto p = plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f);
  ASSERT_FALSE(p.poses.empty());
  EXPECT_LT(max_abs_dy(p, 1.55), 0.3);  // straight over the hill
  EXPECT_GT(max_z(p), 0.8);
}

TEST_F(TerrainPlannerTest, ASlopeWeightGoesAroundAHill)
{
  make({rclcpp::Parameter("terrain.slope_weight", 6.0)});
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, hill);
  set_map(nm);
  const auto p = plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f);
  ASSERT_FALSE(p.poses.empty());
  EXPECT_GT(max_abs_dy(p, 1.55), 0.6);  // around it
  EXPECT_LT(max_z(p), 0.5);
}

TEST_F(TerrainPlannerTest, AGentleRampIsPreferredToASteepOneToTheSameHeight)
{
  // A 0.5 m plateau at x > 4: reached by a steep ramp (30 deg) for y < 1.5 and by a gentle one
  // (8 deg) for y > 1.5. Climbing the same height, the gentle one must win (not only less climb)
  const Height ramps = [](float x, float y) {
      const float deg = y < 1.5f ? 30.0f : 8.0f;
      const float run = 0.5f / std::tan(deg * static_cast<float>(M_PI) / 180.0f);
      return 0.5f * std::clamp((x - (4.0f - run)) / run, 0.0f, 1.0f);
    };
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, ramps);
  // Where the path climbs: on the steep ramp (y < 1.5) or on the gentle one
  auto climbs_gently = [&](const nav_msgs::msg::Path & p) {
      bool any = false, gentle = true;
      for (const auto & ps : p.poses) {
        const auto & q = ps.pose.position;
        if (q.z > 0.1 && q.z < 0.4) {
          any = true;
          gentle = gentle && q.y > 1.5;
        }
      }
      return any && gentle;
    };
  make();
  set_map(nm);
  const auto direct = plan(0.55f, 0.55f, 0.0f, 5.45f, 0.55f, 0.5f);
  ASSERT_FALSE(direct.poses.empty());
  EXPECT_FALSE(climbs_gently(direct));  // without a weight: straight up the steep one
  make({rclcpp::Parameter("terrain.slope_weight", 50.0)});
  set_map(nm);
  const auto gentle = plan(0.55f, 0.55f, 0.0f, 5.45f, 0.55f, 0.5f);
  ASSERT_FALSE(gentle.poses.empty());
  EXPECT_TRUE(climbs_gently(gentle));
}

TEST_F(TerrainPlannerTest, TooSteepCellsAreNotTraversable)
{
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, ridge);
  for (const double max_deg : {30.0, 45.0}) {
    make({rclcpp::Parameter("terrain.max_slope_deg", max_deg)});
    set_map(nm);
    const auto p = plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f);
    EXPECT_EQ(p.poses.empty(), max_deg < 40.0) << max_deg;
  }
  // The goal itself too steep: no plan
  make({rclcpp::Parameter("terrain.max_slope_deg", 30.0)});
  set_map(nm);
  EXPECT_TRUE(plan(0.55f, 1.55f, 0.0f, 2.85f, 1.55f, 0.25f).poses.empty());
}

TEST_F(TerrainPlannerTest, ACurbHigherThanMaxStepIsAvoided)
{
  // Up 0.15 m at x = 3: a curb (over 0.1 m) for y < 2, a gentle ramp (over 1.5 m) for y > 2
  const Height curb = [](float x, float y) {
      const float run = y < 2.0f ? 0.1f : 1.5f;
      return 0.15f * std::clamp((x - 3.0f) / run, 0.0f, 1.0f);
    };
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, curb);
  make({rclcpp::Parameter("terrain.max_step", 0.1)});
  set_map(nm);
  const auto p = plan(0.55f, 0.55f, 0.0f, 5.45f, 0.55f, 0.15f);
  ASSERT_FALSE(p.poses.empty());
  // Goes up by the ramp, never over the curb
  for (std::size_t i = 1; i < p.poses.size(); ++i) {
    const auto & a = p.poses[i - 1].pose.position, & b = p.poses[i].pose.position;
    if ((a.x - 3.05) * (b.x - 3.05) <= 0.0) {EXPECT_GT(std::min(a.y, b.y), 2.1) << i;}
  }
  // Without the limit, straight across
  make();
  set_map(nm);
  EXPECT_LT(max_abs_dy(plan(0.55f, 0.55f, 0.0f, 5.45f, 0.55f, 0.15f), 0.55), 0.3);
}

TEST_F(TerrainPlannerTest, LowCeilingsAreAvoidedWithMinClearance)
{
  // A 0.6 m high table over x in [2, 4], y in [0, 2] (another surface, not reachable)
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, flat);
  add_grid(nm, 20, 20, 2.0f, 0.0f, [](float, float) {return 0.6f;});
  make({rclcpp::Parameter("terrain.min_clearance", 1.0)});
  set_map(nm);
  const auto p = plan(0.55f, 0.95f, 0.0f, 5.45f, 0.95f, 0.0f);
  ASSERT_FALSE(p.poses.empty());
  for (const auto & ps : p.poses) {
    const auto & q = ps.pose.position;
    EXPECT_FALSE(q.x > 2.1 && q.x < 3.9 && q.y < 1.9) << q.x << " " << q.y;
    EXPECT_LT(q.z, 0.1);  // never onto the table
  }
  make();  // a robot lower than the table goes under it
  set_map(nm);
  EXPECT_LT(max_abs_dy(plan(0.55f, 0.95f, 0.0f, 5.45f, 0.95f, 0.0f), 0.95), 0.3);
}

TEST_F(TerrainPlannerTest, RoughPatchesAreAvoidedWithMaxRoughness)
{
  std::mt19937 rng(5);
  std::normal_distribution<float> N(0.0f, 0.05f);
  std::vector<float> noise(61 * 31);
  for (auto & v : noise) {
    v = N(rng);
                                     }
  // Rubble over x in [2, 4], y in [0, 2]
  const Height rubble = [&](float x, float y) {
      const int i = static_cast<int>(std::lround(x * 10.0f)),
        j = static_cast<int>(std::lround(y * 10.0f));
      return x > 2.0f && x < 4.0f && y < 2.0f ? noise[j * 61 + i] : 0.0f;
    };
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, rubble);
  make({rclcpp::Parameter("terrain.max_roughness", 0.02)});
  set_map(nm);
  const auto p = plan(0.55f, 0.95f, 0.0f, 5.45f, 0.95f, 0.0f);
  ASSERT_FALSE(p.poses.empty());
  for (const auto & ps : p.poses) {
    const auto & q = ps.pose.position;
    EXPECT_FALSE(q.x > 2.3 && q.x < 3.7 && q.y < 1.7) << q.x << " " << q.y;
  }
}

TEST_F(TerrainPlannerTest, LinksJoinSurfacesByTheirKind)
{
  // Ground floor x in [0, 2] and an upper one 3 m above at x in [3, 5]; a link between them
  NavMap base;
  add_grid(base, 20, 20, 0.0f, 0.0f, flat);
  add_grid(base, 20, 20, 3.0f, 0.0f, [](float, float) {return 3.0f;});
  base.rebuild_geometry_accels();
  std::size_t s;
  NavCelId a, b;
  Eigen::Vector3f bary;
  ASSERT_TRUE(base.locate_navcel(Eigen::Vector3f(1.93f, 1.02f, 0.1f), s, a, bary, nullptr));
  ASSERT_TRUE(base.locate_navcel(Eigen::Vector3f(3.07f, 1.02f, 3.1f), s, b, bary, nullptr));

  for (const auto kind : {::navmap::LinkType::RAMP, ::navmap::LinkType::STAIRS}) {
    NavMap nm = base;
    nm.add_link(a, b, kind, 4.0f);
    // Default kinds: generic, ramp and door
    make();
    set_map(nm);
    const auto p = plan(0.55f, 1.05f, 0.0f, 4.45f, 1.05f, 3.0f);
    EXPECT_EQ(p.poses.empty(), kind == ::navmap::LinkType::STAIRS);
    if (!p.poses.empty()) {
      EXPECT_NEAR(p.poses.back().pose.position.z, 3.0, 0.05);
      // No waypoint in the air between the floors: the link is taken from end to end
      for (const auto & ps : p.poses) {
        const double z = ps.pose.position.z;
        EXPECT_TRUE(z<0.05 || z>2.95) << z;
      }
    }
  }
  // Stairs, when the robot can take them; and no links at all
  NavMap nm = base;
  nm.add_link(a, b, ::navmap::LinkType::STAIRS, 4.0f);
  make({rclcpp::Parameter("links.types", std::vector<std::string>{"stairs", "nonsense"})});
  set_map(nm);
  EXPECT_FALSE(plan(0.55f, 1.05f, 0.0f, 4.45f, 1.05f, 3.0f).poses.empty());
  make({rclcpp::Parameter("links.enabled", false),
      rclcpp::Parameter("links.types", std::vector<std::string>{"stairs"})});
  set_map(nm);
  EXPECT_TRUE(plan(0.55f, 1.05f, 0.0f, 4.45f, 1.05f, 3.0f).poses.empty());
  // One-way link: down only
  NavMap down = base;
  down.add_link(b, a, ::navmap::LinkType::RAMP, 4.0f, false);
  make();
  set_map(down);
  EXPECT_TRUE(plan(0.55f, 1.05f, 0.0f, 4.45f, 1.05f, 3.0f).poses.empty());
  EXPECT_FALSE(plan(4.45f, 1.05f, 3.0f, 0.55f, 1.05f, 0.0f).poses.empty());
}

TEST_F(TerrainPlannerTest, NewLimitsAndNewTerrainApplyOnTheNextPlan)
{
  NavMap nm;
  add_grid(nm, 60, 30, 0.0f, 0.0f, ridge);
  make({rclcpp::Parameter("terrain.max_slope_deg", 30.0)});
  set_map(nm);
  EXPECT_TRUE(plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f).poses.empty());
  // Reconfigured: initialize() again with another limit
  node_->set_parameter(rclcpp::Parameter("planner.terrain.max_slope_deg", 60.0));
  planner_->initialize(node_, "planner");
  EXPECT_FALSE(plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f).poses.empty());
  // Same size, flat now: the terrain is recomputed for the new geometry
  node_->set_parameter(rclcpp::Parameter("planner.terrain.max_slope_deg", 30.0));
  planner_->initialize(node_, "planner");
  NavMap flat_nm;
  add_grid(flat_nm, 60, 30, 0.0f, 0.0f, flat);
  set_map(flat_nm);
  EXPECT_FALSE(plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f).poses.empty());
  set_map(nm);
  EXPECT_TRUE(plan(0.55f, 1.55f, 0.0f, 5.45f, 1.55f, 0.0f).poses.empty());
}
