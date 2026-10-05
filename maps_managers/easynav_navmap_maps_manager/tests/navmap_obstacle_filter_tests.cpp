// Copyright 2026 Intelligent Robotics Lab
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

#include <algorithm>
#include <cstdint>
#include <memory>

#include <string>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_navmap_maps_manager/filters/ObstacleFilter.hpp"

class NavmapObstacleFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Obstacles layer value of the NavCel at (x, y).
  static std::uint8_t obstacles_at(const ::navmap::NavMap & nm, float x, float y)
  {
    std::size_t sidx = 0;
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;
    EXPECT_TRUE(nm.locate_navcel(Eigen::Vector3f(x, y, 0.0f), sidx, cid, bary, &hit));
    return nm.layer_get<std::uint8_t>("obstacles", cid, 123);
  }
};

TEST_F(NavmapObstacleFilterTest, TheStaticMapIsKeptWhenTheSensorsDoNotSeeIt)
{
  // 2 x 2 m at 0.1 m: one occupied cell, one unknown, the rest free.
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1;
  grid.info.width = 20;
  grid.info.height = 20;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(400, 0);
  grid.data[5 * 20 + 15] = 100;   // occupied at (1.55, 0.55)
  grid.data[15 * 20 + 5] = -1;    // unknown at (0.55, 1.55)

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("maps_manager_node");
  easynav::navmap::ObstacleFilter filter;
  filter.initialize(node, "navmap.obstacles");

  easynav::NavState nav_state;
  nav_state.set("map.navmap", navmap_ros::from_occupancy_grid(grid));
  // A perception with nothing on it: the sensors see no obstacle at all.
  easynav::PointPerception perception;
  perception.frame_id = "map";
  perception.stamp = node->now();
  perception.valid = true;
  nav_state.set("scan", perception);

  filter.update(nav_state);
  const auto & nm = nav_state.get<::navmap::NavMap>("map.navmap");
  ASSERT_TRUE(nm.has_layer("obstacles"));
  EXPECT_EQ(obstacles_at(nm, 1.55f, 0.55f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(obstacles_at(nm, 0.55f, 1.55f), navmap_ros::NO_INFORMATION);
  EXPECT_EQ(obstacles_at(nm, 1.05f, 1.05f), navmap_ros::FREE_SPACE);

  // Updated again (each cycle): still there, not accumulated or cleared.
  filter.update(nav_state);
  const auto & again = nav_state.get<::navmap::NavMap>("map.navmap");
  EXPECT_EQ(obstacles_at(again, 1.55f, 0.55f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(obstacles_at(again, 1.05f, 1.05f), navmap_ros::FREE_SPACE);
}

// Range and height limits, with the robot at (4, 4) in an 8 x 8 m free map.
class NavmapObstacleFilterLimitsTest : public NavmapObstacleFilterTest
{
protected:
  void SetUp() override
  {
    NavmapObstacleFilterTest::SetUp();
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "maps_manager_node_" + std::to_string(count++));

    auto tf_buffer = easynav::RTTFBuffer::getInstance();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = tf_buffer->get_tf_info().map_frame;
    tf.child_frame_id = tf_buffer->get_tf_info().robot_frame;
    tf.transform.translation.x = 4.0;
    tf.transform.translation.y = 4.0;
    tf.transform.rotation.w = 1.0;
    tf_buffer->setTransform(tf, "test", true);

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.1;
    grid.info.width = 80;
    grid.info.height = 80;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(80 * 80, 0);
    nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid));

    perception_.frame_id = "map";
    perception_.stamp = node_->now();
    perception_.valid = true;
  }

  // Vertical column of points at (x, y) in the map, from z0 to z1. The filter marks the NavCel
  // at the center of each 0.3 m voxel, so (x, y) are voxel centers; the points are put a bit
  // off them, away from the voxel borders.
  void column(float x, float y, float z0, float z1)
  {
    for (float z = z0; z <= z1 + 1e-3f; z += 0.1f) {
      perception_.data.push_back(pcl::PointXYZ(x + 0.05f, y + 0.05f, z));
    }
  }

  const ::navmap::NavMap & run(easynav::navmap::ObstacleFilter & filter)
  {
    filter.initialize(node_, "navmap.obstacles");
    nav_state_.set("scan", perception_);
    filter.update(nav_state_);
    return nav_state_.get<::navmap::NavMap>("map.navmap");
  }

  // Highest value of the two triangles of the cell at (x, y): a voxel center lies on their diagonal.
  static std::uint8_t cell_at(const ::navmap::NavMap & nm, float x, float y)
  {
    return std::max(obstacles_at(nm, x + 0.02f, y - 0.02f), obstacles_at(nm, x - 0.02f, y + 0.02f));
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
  easynav::PointPerception perception_;
};

TEST_F(NavmapObstacleFilterLimitsTest, ByDefaultFarAndHighPointsAreObstacles)
{
  column(4.65f, 4.05f, 0.0f, 0.8f);    // near
  column(7.35f, 4.05f, 0.0f, 0.8f);    // 3.35 m away
  column(4.05f, 2.25f, 1.5f, 2.2f);    // above the robot
  easynav::navmap::ObstacleFilter filter;
  const auto & nm = run(filter);
  EXPECT_EQ(cell_at(nm, 4.65f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(cell_at(nm, 7.35f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(cell_at(nm, 4.05f, 2.25f), navmap_ros::LETHAL_OBSTACLE);
}

TEST_F(NavmapObstacleFilterLimitsTest, PointsBeyondTheRangeAreIgnored)
{
  node_->declare_parameter("navmap.obstacles.max_range", 3.0);
  column(4.65f, 4.05f, 0.0f, 0.8f);    // 0.65 m
  column(6.75f, 4.05f, 0.0f, 0.8f);    // 2.75 m: inside
  column(7.35f, 4.05f, 0.0f, 0.8f);    // 3.35 m: outside
  column(4.05f, 0.45f, 0.0f, 0.8f);    // 3.55 m behind (y): outside
  easynav::navmap::ObstacleFilter filter;
  const auto & nm = run(filter);
  EXPECT_EQ(cell_at(nm, 4.65f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(cell_at(nm, 6.75f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(cell_at(nm, 7.35f, 4.05f), navmap_ros::FREE_SPACE);
  EXPECT_EQ(cell_at(nm, 4.05f, 0.45f), navmap_ros::FREE_SPACE);
}

TEST_F(NavmapObstacleFilterLimitsTest, PointsOutsideTheHeightBandAreIgnored)
{
  node_->declare_parameter("navmap.obstacles.min_height", 0.05);
  node_->declare_parameter("navmap.obstacles.max_height", 1.2);
  column(4.65f, 4.05f, 0.0f, 0.8f);    // within the band
  column(4.05f, 2.25f, 1.5f, 2.2f);    // above it
  column(2.25f, 4.05f, -0.6f, 0.0f);   // below it
  easynav::navmap::ObstacleFilter filter;
  const auto & nm = run(filter);
  EXPECT_EQ(cell_at(nm, 4.65f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(cell_at(nm, 4.05f, 2.25f), navmap_ros::FREE_SPACE);
  EXPECT_EQ(cell_at(nm, 2.25f, 4.05f), navmap_ros::FREE_SPACE);
}

TEST_F(NavmapObstacleFilterLimitsTest, AColumnCrossingTheBandIsStillAnObstacle)
{
  node_->declare_parameter("navmap.obstacles.max_height", 1.2);
  column(4.65f, 4.05f, 0.0f, 2.5f);    // a shelf post, higher than the robot
  easynav::navmap::ObstacleFilter filter;
  const auto & nm = run(filter);
  EXPECT_EQ(cell_at(nm, 4.65f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
}

TEST_F(NavmapObstacleFilterLimitsTest, TheStaticMapIsKeptBeyondTheRange)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1;
  grid.info.width = 80;
  grid.info.height = 80;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(80 * 80, 0);
  grid.data[40 * 80 + 73] = 100;       // static obstacle at (7.35, 4.05)
  nav_state_.set("map.navmap", navmap_ros::from_occupancy_grid(grid));
  node_->declare_parameter("navmap.obstacles.max_range", 3.0);
  column(4.65f, 4.05f, 0.0f, 0.8f);
  easynav::navmap::ObstacleFilter filter;
  const auto & nm = run(filter);
  EXPECT_EQ(cell_at(nm, 7.35f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
  EXPECT_EQ(cell_at(nm, 4.65f, 4.05f), navmap_ros::LETHAL_OBSTACLE);
}
