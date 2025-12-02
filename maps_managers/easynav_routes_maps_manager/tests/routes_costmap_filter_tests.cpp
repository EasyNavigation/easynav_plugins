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

#include "easynav_common/types/NavState.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"

#include "easynav_routes_maps_manager/filters/RoutesCostmapFilter.hpp"

#include "easynav_routes_maps_manager/RoutesMapsManager.hpp"  // for RoutesMap and RouteSegment typedefs

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using easynav::RoutesCostmapFilter;
using easynav::RoutesMap;
using easynav::RouteSegment;
using easynav::NavState;
using easynav::Costmap2D;

class RoutesCostmapFilterTest : public ::testing::Test
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

TEST_F(RoutesCostmapFilterTest, DoesNothingWhenNavStateMissingKeys)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("routes_costmap_filter_missing");

  RoutesCostmapFilter filter;
  auto init_result = filter.initialize(node, "routes.routes_costmap", "");
  ASSERT_TRUE(init_result.has_value()) << init_result.error();

  NavState nav_state;
  // No 'routes' and no 'map.dynamic.filtered' keys -> update should not crash
  EXPECT_NO_THROW(filter.update(nav_state));
}

TEST_F(RoutesCostmapFilterTest, RaisesCostOutsideRoutes)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("routes_costmap_filter_basic");

  RoutesCostmapFilter filter;
  auto init_result = filter.initialize(node, "routes.routes_costmap", "");
  ASSERT_TRUE(init_result.has_value()) << init_result.error();

  // Simple costmap 10x1, resolution 1.0, origin at (0,0)
  Costmap2D map(10, 1, 1.0, 0.0, 0.0);
  // Initialize all costs to 0
  for (unsigned int x = 0; x < 10; ++x) {
    map.setCost(x, 0, 0);
  }

  // Single route from (2,0) to (7,0)
  RoutesMap routes;
  RouteSegment seg;
  seg.id = "route0";
  seg.start.position.x = 2.0;
  seg.start.position.y = 0.0;
  seg.start.orientation.w = 1.0;
  seg.end.position.x = 7.0;
  seg.end.position.y = 0.0;
  seg.end.orientation.w = 1.0;
  routes.push_back(seg);

  NavState nav_state;
  nav_state.set("routes", routes);
  nav_state.set("map.dynamic.filtered", map);

  filter.update(nav_state);

  auto map_after = nav_state.get<Costmap2D>("map.dynamic.filtered");

  // Cells clearly along the interior of the route should remain 0,
  // others should be >= 50 (default min_cost). The last endpoint
  // cell (x=7) may fall outside the corridor depending on the
  // distance test, so we only assert [2,6] as "on route" here.
  for (unsigned int x = 0; x < 10; ++x) {
    if (x >= 2 && x <= 6) {
      EXPECT_EQ(map_after.getCost(x, 0), 0) << "Cell on route changed at x=" << x;
    } else {
      EXPECT_GE(map_after.getCost(x, 0), 50) << "Cell outside route not raised at x=" << x;
    }
  }
}

TEST_F(RoutesCostmapFilterTest, IgnoresRoutePointsOutsideMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("routes_costmap_filter_outside");

  RoutesCostmapFilter filter;
  auto init_result = filter.initialize(node, "routes.routes_costmap", "");
  ASSERT_TRUE(init_result.has_value()) << init_result.error();

  // Costmap 5x1 from x=0..5
  Costmap2D map(5, 1, 1.0, 0.0, 0.0);
  for (unsigned int x = 0; x < 5; ++x) {
    map.setCost(x, 0, 0);
  }

  // Route completely outside map, e.g., from (10,0) to (11,0)
  RoutesMap routes;
  RouteSegment seg;
  seg.id = "route_outside";
  seg.start.position.x = 10.0;
  seg.start.position.y = 0.0;
  seg.start.orientation.w = 1.0;
  seg.end.position.x = 11.0;
  seg.end.position.y = 0.0;
  seg.end.orientation.w = 1.0;
  routes.push_back(seg);

  NavState nav_state;
  nav_state.set("routes", routes);
  nav_state.set("map.dynamic.filtered", map);

  filter.update(nav_state);

  auto map_after = nav_state.get<Costmap2D>("map.dynamic.filtered");

  // No cell should be considered "on route"; all should be raised to >= 50
  for (unsigned int x = 0; x < 5; ++x) {
    EXPECT_GE(map_after.getCost(x, 0), 50) << "Cell not raised at x=" << x;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
