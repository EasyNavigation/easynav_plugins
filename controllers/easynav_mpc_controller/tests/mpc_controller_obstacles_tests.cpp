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
/// \brief MPC obstacles: points taken in the robot frame towards where it moves, and penalized
/// in the cost along the predicted (chained) trajectory.

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
#include "easynav_mpc_controller/MPCController.hpp"
#include "easynav_mpc_controller/MPCOptimizer.hpp"

namespace
{

class TestMpc : public easynav::MPCController
{
public:
  using MPCController::obstacle_points;
};

}  // namespace

class MpcObstaclesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Robot at (x, y, yaw) in the map; MPC with obstacle_range 2.0 and z_min_filter 0.1.
  void make(double x, double y, double yaw, const std::vector<rclcpp::Parameter> & extra = {})
  {
    static int count = 0;
    std::vector<rclcpp::Parameter> overrides{
      {"mpc.obstacle_range", 2.0}, {"mpc.z_min_filter", 0.1}};
    overrides.insert(overrides.end(), extra.begin(), extra.end());
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "mpc_points_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(overrides));
    mpc_.initialize(node_, "mpc");

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

  void perceive(const std::vector<pcl::PointXYZ> & points)
  {
    easynav::PointPerception perception;
    perception.data.insert(perception.data.end(), points.begin(), points.end());
    perception.frame_id = "map";
    perception.stamp = node_->now();
    perception.valid = true;
    nav_state_.set("scan", perception);
  }

  bool kept(double x, double y, bool backward = false)
  {
    for (const auto & p : mpc_.obstacle_points(nav_state_, backward)) {
      if (std::hypot(p.x - x, p.y - y) < 0.1) {return true;}
    }
    return false;
  }

  // Cost of a constant (v, w) over 10 steps of 0.1 s from the origin, towards \p goal.
  static double cost(
    double v, double w, const Eigen::Vector2d & goal,
    const pcl::PointCloud<pcl::PointXYZ> & points = {}, double safety_radius = 0.0)
  {
    easynav::MPCParameters params(goal, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, points, 10, 0.1);
    params.safety_radius = safety_radius;
    std::vector<double> u;
    for (int i = 0; i < 10; ++i) {
      u.push_back(v);
      u.push_back(w);
    }
    std::vector<double> grad;
    easynav::MPCOptimizer optimizer;
    return optimizer.cost_function(u, grad, &params);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  TestMpc mpc_;
  easynav::NavState nav_state_;
};

TEST_F(MpcObstaclesTest, AheadAndBesideAreKeptBehindIsNot)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.0, 0.0, 0.3}, {0.0, 1.0, 0.3}, {-1.0, 0.0, 0.3}, {2.3, 0.0, 0.3}});
  EXPECT_TRUE(kept(1.0, 0.0));
  EXPECT_TRUE(kept(0.0, 1.0));
  EXPECT_FALSE(kept(-1.0, 0.0));
  EXPECT_FALSE(kept(2.3, 0.0));
}

TEST_F(MpcObstaclesTest, BackwardLooksBehind)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.0, 0.0, 0.3}, {-1.0, 0.0, 0.3}});
  EXPECT_FALSE(kept(1.0, 0.0, true));
  EXPECT_TRUE(kept(-1.0, 0.0, true));
}

TEST_F(MpcObstaclesTest, TheGroundAndAboveTheRobotAreNotKept)
{
  make(0.0, 0.0, 0.0);
  perceive({{1.0, 0.0, 0.05}, {1.0, 0.5, 0.45}, {1.0, -0.5, 0.8}});
  EXPECT_FALSE(kept(1.0, 0.0));
  EXPECT_TRUE(kept(1.0, 0.5));
  EXPECT_FALSE(kept(1.0, -0.5));
}

TEST_F(MpcObstaclesTest, ItIsTheRobotsFrameNotTheMaps)
{
  make(3.0, 1.0, M_PI / 2.0);
  perceive({{3.0, 2.0, 0.3}, {3.0, 0.0, 0.3}});
  EXPECT_TRUE(kept(3.0, 2.0));
  EXPECT_FALSE(kept(3.0, 0.0));
}

TEST_F(MpcObstaclesTest, TheRemovedCollisionCheckerParameterIsHarmless)
{
  EXPECT_NO_THROW(make(0.0, 0.0, 0.0, {{"mpc.use_collision_checker", true}}));
}

// With the steps chained, 0.5 m/s reaches a goal 0.5 m ahead in 1 s; 1 m/s overshoots it.
// Unchained (every step from the start), the faster one would look better.
TEST_F(MpcObstaclesTest, TheHorizonIsChained)
{
  EXPECT_LT(cost(0.5, 0.0, {0.5, 0.0}), cost(1.0, 0.0, {0.5, 0.0}));
}

TEST_F(MpcObstaclesTest, AnObstacleOnTheWayRaisesTheCost)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back({0.6, 0.0, 0.1});
  const double clear = cost(1.0, 0.0, {2.0, 0.0});
  EXPECT_GT(cost(1.0, 0.0, {2.0, 0.0}, points, 0.5), clear + 100.0);
  // Turning away from it costs less than driving into it.
  EXPECT_LT(cost(1.0, 1.5, {2.0, 0.0}, points, 0.5), cost(1.0, 0.0, {2.0, 0.0}, points, 0.5));
}

TEST_F(MpcObstaclesTest, FarObstaclesOrNoSafetyRadiusDoNotCount)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back({0.6, 3.0, 0.1});
  const double clear = cost(1.0, 0.0, {2.0, 0.0});
  EXPECT_DOUBLE_EQ(cost(1.0, 0.0, {2.0, 0.0}, points, 0.5), clear);
  points.push_back({0.6, 0.0, 0.1});
  EXPECT_DOUBLE_EQ(cost(1.0, 0.0, {2.0, 0.0}, points, 0.0), clear);
}
