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

#include <array>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_diagnostic_recovery/ObstacleProximity.hpp"

class ObstacleProximityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }
};

TEST_F(ObstacleProximityTest, ReturnsInfiniteDistanceWithNoPerceptions)
{
  easynav::NavState nav_state;
  auto result = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state);
  EXPECT_FALSE(std::isfinite(result.distance));
}

TEST_F(ObstacleProximityTest, FindsNearestPointAheadOfTheRobot)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";  // same as robot_frame: no TF lookup needed
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  perception.data.points.resize(2);
  perception.data.points[0].x = 3.0;
  perception.data.points[0].y = 0.0;
  perception.data.points[0].z = 0.0;
  perception.data.points[1].x = 1.0;   // nearer, straight ahead
  perception.data.points[1].y = 0.0;
  perception.data.points[1].z = 0.0;

  easynav::NavState nav_state;
  nav_state.set("scan", perception);

  auto result = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state);

  ASSERT_TRUE(std::isfinite(result.distance));
  EXPECT_NEAR(result.distance, 1.0, 1e-6);
  EXPECT_NEAR(result.bearing, 0.0, 1e-6);  // straight ahead
}

TEST_F(ObstacleProximityTest, ReportsBearingForAnObstacleBehindTheRobot)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  perception.data.points.resize(1);
  perception.data.points[0].x = -1.0;  // directly behind
  perception.data.points[0].y = 0.0;
  perception.data.points[0].z = 0.0;

  easynav::NavState nav_state;
  nav_state.set("scan", perception);

  auto result = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state);

  ASSERT_TRUE(std::isfinite(result.distance));
  EXPECT_NEAR(result.distance, 1.0, 1e-6);
  EXPECT_NEAR(std::abs(result.bearing), M_PI, 1e-6);  // behind: bearing near +-pi
}

namespace
{

easynav::PointPerception scan_with(std::vector<std::array<float, 3>> points, bool valid = true)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";
  perception.stamp = rclcpp::Time(0);
  perception.valid = valid;
  for (const auto & p : points) {
    perception.data.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
  }
  return perception;
}

}  // namespace

TEST_F(ObstacleProximityTest, NotPerceivedWithoutPerceptionsOrData)
{
  easynav::NavState nav_state;
  EXPECT_FALSE(easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state).perceived);

  nav_state.set("scan", scan_with({{1.0, 0.0, 0.2}}, false));  // Before its first data
  const auto result = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state);
  EXPECT_FALSE(result.perceived);
  EXPECT_FALSE(std::isfinite(result.distance));
}

TEST_F(ObstacleProximityTest, PerceivedButNothingInRangeIsAClearScene)
{
  easynav::NavState nav_state;
  nav_state.set("scan", scan_with({}));
  const auto result = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state);
  EXPECT_TRUE(result.perceived);
  EXPECT_FALSE(std::isfinite(result.distance));
}

TEST_F(ObstacleProximityTest, IgnoresPointsOutsideTheHeightRange)
{
  // E.g. the ground seen by a 3D lidar, and a ceiling above the robot.
  easynav::NavState nav_state;
  nav_state.set("scan", scan_with({{0.3, 0.0, -0.2}, {0.4, 0.0, 2.5}, {1.5, 0.0, 0.5}}));

  auto all = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state);
  EXPECT_NEAR(all.distance, 0.3, 1e-6) << "no range: every point counts";

  auto in_range = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state, 0.0, 1.0);
  EXPECT_TRUE(in_range.perceived);
  EXPECT_NEAR(in_range.distance, 1.5, 1e-6);

  auto none = easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state, 0.6, 1.0);
  EXPECT_TRUE(none.perceived);
  EXPECT_FALSE(std::isfinite(none.distance));
}

TEST_F(ObstacleProximityTest, HeightRangeIsInclusive)
{
  easynav::NavState nav_state;
  nav_state.set("scan", scan_with({{1.0, 0.0, 0.0}, {2.0, 0.0, 1.0}}));
  EXPECT_NEAR(
    easynav_diagnostic_recovery::compute_nearest_obstacle(nav_state, 0.0, 1.0).distance, 1.0,
    1e-6) << "a 2D laser at the robot frame's height counts";
}

// ─── free_distance_along_x ──────────────────────────────────────────────────────────────────

namespace
{

void scene(easynav::NavState & nav_state, std::vector<std::array<double, 2>> points)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  for (const auto & p : points) {
    perception.data.points.emplace_back(p[0], p[1], 0.0);
  }
  nav_state.set("scan", perception);
}

}  // namespace

using easynav_diagnostic_recovery::free_distance_along_x;

TEST_F(ObstacleProximityTest, FreeDistanceToAPointStraightAhead)
{
  easynav::NavState nav_state;
  scene(nav_state, {{1.0, 0.0}});
  EXPECT_NEAR(free_distance_along_x(nav_state, 1, 0.3), 0.7, 1e-6);
  EXPECT_TRUE(std::isinf(free_distance_along_x(nav_state, -1, 0.3))) << "behind is clear";
}

TEST_F(ObstacleProximityTest, FreeDistanceToAPointOffTheAxis)
{
  // The disk touches it when its center is sqrt(0.3^2 - 0.2^2) short of it.
  easynav::NavState nav_state;
  scene(nav_state, {{1.0, 0.2}});
  EXPECT_NEAR(free_distance_along_x(nav_state, 1, 0.3), 1.0 - std::sqrt(0.05), 1e-6);
}

TEST_F(ObstacleProximityTest, PointsOutsideTheCorridorDoNotCount)
{
  easynav::NavState nav_state;
  scene(nav_state, {{0.5, 0.3}, {0.5, -0.35}});
  EXPECT_TRUE(std::isinf(free_distance_along_x(nav_state, 1, 0.3)));
}

TEST_F(ObstacleProximityTest, APointAlreadyTouchingLeavesNoFreeDistance)
{
  easynav::NavState nav_state;
  scene(nav_state, {{-0.2, 0.0}});
  EXPECT_DOUBLE_EQ(free_distance_along_x(nav_state, -1, 0.3), 0.0);
}

TEST_F(ObstacleProximityTest, TheNearestPointInTheWayCounts)
{
  easynav::NavState nav_state;
  scene(nav_state, {{2.0, 0.0}, {0.8, 0.1}, {-0.5, 0.0}});
  EXPECT_NEAR(free_distance_along_x(nav_state, 1, 0.3), 0.8 - std::sqrt(0.08), 1e-6);
  EXPECT_NEAR(free_distance_along_x(nav_state, -1, 0.3), 0.2, 1e-6);
}

TEST_F(ObstacleProximityTest, WithoutPerceptionNothingIsClear)
{
  easynav::NavState nav_state;
  EXPECT_DOUBLE_EQ(free_distance_along_x(nav_state, 1, 0.3), 0.0);
}

TEST_F(ObstacleProximityTest, FreeDistanceIgnoresPointsOutsideTheHeightRange)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  perception.data.points.emplace_back(0.5, 0.0, 0.0);  // ground
  easynav::NavState nav_state;
  nav_state.set("scan", perception);
  EXPECT_TRUE(std::isinf(free_distance_along_x(nav_state, 1, 0.3, 0.1, 0.5)));
}
