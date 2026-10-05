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

/// \file
/// \brief MPPI takes the obstacle points in the robot frame, towards where it moves.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_mppi_controller/MPPIController.hpp"

namespace
{

class TestMppi : public easynav::MPPIController
{
public:
  using MPPIController::obstacle_points;
};

}  // namespace

class MppiObstaclePointsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Robot at (x, y, yaw) in the map; MPPI with obstacle_range 2.0 and z_min_filter 0.1.
  void make(double x, double y, double yaw)
  {
    static int count = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "mppi_points_" + std::to_string(count++), rclcpp::NodeOptions().parameter_overrides(
        {{"mppi.obstacle_range", 2.0}, {"mppi.z_min_filter", 0.1}}));
    mppi_.initialize(node_, "mppi");

    auto tf_buffer = easynav::RTTFBuffer::getInstance();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = tf_buffer->get_tf_info().map_frame;
    tf.child_frame_id = tf_buffer->get_tf_info().robot_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_buffer->setTransform(tf, "test", true);
  }

  // Points (map frame) in a perception.
  void perceive(const std::vector<pcl::PointXYZ> & points)
  {
    easynav::PointPerception perception;
    perception.data.insert(perception.data.end(), points.begin(), points.end());
    perception.frame_id = "map";
    perception.stamp = node_->now();
    perception.valid = true;
    nav_state_.set("scan", perception);
  }

  // Whether a point near (x, y) in the map is among the obstacle points.
  bool kept(double x, double y, bool backward = false)
  {
    for (const auto & p : mppi_.obstacle_points(nav_state_, backward)) {
      if (std::hypot(p.x - x, p.y - y) < 0.1) {return true;}
    }
    return false;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  TestMppi mppi_;
  easynav::NavState nav_state_;
};

TEST_F(MppiObstaclePointsTest, AheadAndBesideAreKeptBehindIsNot)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.0, 0.0, 0.3}, {0.0, 1.0, 0.3}, {-1.0, 0.0, 0.3}, {-0.2, 0.0, 0.3}});
  EXPECT_TRUE(kept(1.0, 0.0));
  EXPECT_TRUE(kept(0.0, 1.0));
  EXPECT_FALSE(kept(-1.0, 0.0));
  EXPECT_TRUE(kept(-0.2, 0.0));    // within the robot radius (0.3)
}

TEST_F(MppiObstaclePointsTest, BeyondTheRangeIsNotKept)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.9, 0.0, 0.3}, {2.3, 0.0, 0.3}, {0.5, 2.3, 0.3}, {0.5, -1.9, 0.3}});
  EXPECT_TRUE(kept(1.9, 0.0));
  EXPECT_FALSE(kept(2.3, 0.0));
  EXPECT_FALSE(kept(0.5, 2.3));
  EXPECT_TRUE(kept(0.5, -1.9));
}

TEST_F(MppiObstaclePointsTest, BackwardLooksBehind)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.0, 0.0, 0.3}, {-1.0, 0.0, 0.3}});
  EXPECT_FALSE(kept(1.0, 0.0, true));
  EXPECT_TRUE(kept(-1.0, 0.0, true));
}

TEST_F(MppiObstaclePointsTest, TheGroundAndAboveTheRobotAreNotKept)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.0, 0.0, 0.05}, {1.0, 0.5, 0.45}, {1.0, -0.5, 0.8}});
  EXPECT_FALSE(kept(1.0, 0.0));     // below z_min_filter
  EXPECT_TRUE(kept(1.0, 0.5));      // within the robot height (0.5)
  EXPECT_FALSE(kept(1.0, -0.5));    // above it
}

TEST_F(MppiObstaclePointsTest, ItIsTheRobotsFrameNotTheMaps)
{
  // Robot at (3, 1) facing +y: "ahead" is +y in the map.
  make(3.0, 1.0, M_PI / 2.0);
  perceive({{3.0, 2.0, 0.3}, {3.0, 0.0, 0.3}, {4.0, 1.0, 0.3}});
  EXPECT_TRUE(kept(3.0, 2.0));
  EXPECT_FALSE(kept(3.0, 0.0));
  EXPECT_TRUE(kept(4.0, 1.0));      // beside it
}

TEST_F(MppiObstaclePointsTest, NoPerceptionsGiveNoPoints)
{
  make(0.0, 0.0, 0.0);
  EXPECT_TRUE(mppi_.obstacle_points(nav_state_, false).empty());
}
