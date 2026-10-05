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

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_mpc_controller/MPCController.hpp"

class MpcNoObstaclesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // A straight path ahead of the robot and a perception with the given points.
  static void fill(
    easynav::NavState & nav_state, const rclcpp::Time & now,
    const std::vector<pcl::PointXYZ> & points)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i <= 20; ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = 0.1 * i;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    nav_state.set("path", path);

    nav_msgs::msg::Odometry robot;
    robot.header.frame_id = "map";
    robot.pose.pose.orientation.w = 1.0;
    nav_state.set("robot_pose", robot);

    easynav::PointPerception perception;
    perception.data.insert(perception.data.end(), points.begin(), points.end());
    perception.frame_id = "map";
    perception.stamp = now;
    perception.valid = true;
    nav_state.set("scan", perception);
  }
};

TEST_F(MpcNoObstaclesTest, NoObstaclePointsDoNotAbort)
{
  // No points at all, and points outside the obstacle filter: the filtered cloud is empty,
  // which pcl::toROSMsg() could not convert.
  for (const auto & points : std::vector<std::vector<pcl::PointXYZ>>{
    {}, {{5.0, 0.0, 0.5}}, {{-1.0, 3.0, 0.5}, {1.0, -3.0, 0.5}}})
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_node");
    easynav::MPCController mpc;
    mpc.initialize(node, "mpc");
    easynav::NavState nav_state;
    fill(nav_state, node->now(), points);
    ASSERT_NO_THROW(mpc.update_rt(nav_state)) << points.size() << " points";
    ASSERT_TRUE(nav_state.has("cmd_vel")) << points.size() << " points";
  }
}
