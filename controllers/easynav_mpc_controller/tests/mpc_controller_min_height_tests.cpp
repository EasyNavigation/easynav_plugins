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
/// \brief MPCController "min_height": points below it are floor hits, not obstacles.

#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_mpc_controller/MPCController.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

using namespace std::chrono_literals;

namespace
{

class TestMpc : public easynav::MPCController
{
public:
  double min_height() const {return min_height_;}
};

}  // namespace

class MpcMinHeightTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Runs one MPC cycle with three obstacle points, at heights 0.05, 0.08 and 0.20 m and
  // x = -0.5, -1.0, -1.5 (inside the MPC's detection box), and returns the x of the points
  // published on /mpc/detection (the obstacles the MPC takes into account).
  static std::set<double> detected(double min_height_param, double * min_height = nullptr)
  {
    static int count = 0;
    const std::string suffix = std::to_string(count++);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_node_" + suffix);
    if (min_height_param >= 0.0) {
      node->declare_parameter("mpc.min_height", min_height_param);
    }
    TestMpc mpc;
    mpc.initialize(node, "mpc");
    if (min_height) {
      *min_height = mpc.min_height();
    }
    node->configure();
    node->activate();

    std::set<double> xs;
    bool received = false;
    auto listener = rclcpp::Node::make_shared("detection_listener_" + suffix);
    auto sub = listener->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mpc/detection", 10, [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
        if (cloud->width * cloud->height > 0) {
          for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud, "x"); it != it.end(); ++it) {
            xs.insert(std::round(*it * 10.0) / 10.0);
          }
        }
        received = true;
      });

    easynav::NavState nav_state;
    const auto & map_frame = easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame;
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame;
    for (int i = 0; i <= 20; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame;
      pose.pose.position.x = 0.1 * i;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    nav_state.set("path", path);
    nav_msgs::msg::Odometry robot_pose;
    robot_pose.header.frame_id = map_frame;
    robot_pose.pose.pose.orientation.w = 1.0;
    nav_state.set("robot_pose", robot_pose);

    easynav::PointPerception perception;
    perception.frame_id = map_frame;
    perception.stamp = node->now();
    perception.valid = true;
    perception.data.push_back(pcl::PointXYZ(-0.5f, 0.0f, 0.05f));
    perception.data.push_back(pcl::PointXYZ(-1.0f, 0.0f, 0.08f));
    perception.data.push_back(pcl::PointXYZ(-1.5f, 0.0f, 0.20f));
    nav_state.set("scan", perception);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener);
    // Discovery: wait until the listener is matched with the detection publisher
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (listener->count_publishers("/mpc/detection") == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    mpc.update_rt(nav_state);
    const auto spin_deadline = std::chrono::steady_clock::now() + 3s;
    while (!received && std::chrono::steady_clock::now() < spin_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_TRUE(received) << "no /mpc/detection message";
    return xs;
  }
};

TEST_F(MpcMinHeightTest, DefaultIgnoresPointsBelowTenCentimeters)
{
  double min_height = 0.0;
  EXPECT_EQ(detected(-1.0, &min_height), (std::set<double>{-1.5}));
  EXPECT_DOUBLE_EQ(min_height, 0.1);
}

TEST_F(MpcMinHeightTest, LowerMinHeightKeepsALowLaser)
{
  double min_height = 0.0;
  EXPECT_EQ(detected(0.07, &min_height), (std::set<double>{-1.0, -1.5}));
  EXPECT_DOUBLE_EQ(min_height, 0.07);
}

TEST_F(MpcMinHeightTest, HigherMinHeightIgnoresMore)
{
  EXPECT_EQ(detected(0.3), (std::set<double>{}));
}
