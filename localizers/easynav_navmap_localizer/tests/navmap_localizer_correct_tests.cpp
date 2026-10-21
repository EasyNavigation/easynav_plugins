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
/// \brief The correction: which points are scored, and how, in a 6 x 6 m room with a floor, four
/// walls and a ceiling, seen by a sensor from a known pose.

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "bonxai/bonxai.hpp"
#include "bonxai/probabilistic_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_navmap_localizer/AMCLLocalizer.hpp"

namespace
{

class TestAmcl : public easynav::navmap::AMCLLocalizer
{
public:
  using AMCLLocalizer::correct;
  using AMCLLocalizer::particles_;
};

constexpr double kCeiling = 2.2;

// Is (x, y, z) on the room: floor, ceiling or a wall of [0, 6] x [0, 6]
std::shared_ptr<Bonxai::ProbabilisticMap> room()
{
  auto map = std::make_shared<Bonxai::ProbabilisticMap>(0.1);
  for (double a = 0.0; a <= 6.0 + 1e-9; a += 0.05) {
    for (double b = 0.0; b <= 6.0 + 1e-9; b += 0.05) {
      map->addHitPoint({a, b, 0.0});
      map->addHitPoint({a, b, kCeiling});
    }
    for (double z = 0.0; z <= kCeiling + 1e-9; z += 0.05) {
      map->addHitPoint({a, 0.0, z});
      map->addHitPoint({a, 6.0, z});
      map->addHitPoint({0.0, a, z});
      map->addHitPoint({6.0, a, z});
    }
  }
  return map;
}

// Where a ray from o along d first meets the room
Eigen::Vector3d cast(const Eigen::Vector3d & o, const Eigen::Vector3d & d)
{
  double t = std::numeric_limits<double>::infinity();
  auto hit = [&](double from, double to, double dir) {
      if (std::abs(dir) > 1e-9) {
        const double s = ((dir > 0 ? to : from) - 0.0) / dir;
        if (s > 1e-6) {t = std::min(t, s);}
      }
    };
  hit(-o.x(), 6.0 - o.x(), d.x());
  hit(-o.y(), 6.0 - o.y(), d.y());
  hit(-o.z(), kCeiling - o.z(), d.z());
  return o + t * d;
}

}  // namespace

class NavmapAmclCorrectTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void make(std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> params {
      rclcpp::Parameter("loc.num_particles", 5),
      rclcpp::Parameter("loc.downsampled_cloud_size", 0.0),
      rclcpp::Parameter("loc.correct_max_points", 100000),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    node_ = std::make_shared<easynav::LocalizerNode>(
      rclcpp::NodeOptions().parameter_overrides(params));
    amcl_ = std::make_shared<TestAmcl>();
    amcl_->initialize(node_, "loc");
    nav_state_.set("map.bonxai", room_);
    tf_ = easynav::RTTFBuffer::getInstance();
    now_ = node_->now();
    odom_at(now_, 0.0);
  }

  // odom -> base_footprint at time t: the robot x metres ahead
  void odom_at(const rclcpp::Time & t, double x)
  {
    const auto info = tf_->get_tf_info();
    geometry_msgs::msg::TransformStamped tr;
    tr.header.stamp = t;
    tr.header.frame_id = info.odom_frame;
    tr.child_frame_id = info.robot_footprint_frame;
    tr.transform.translation.x = x;
    tr.transform.rotation.w = 1.0;
    tf_->setTransform(tr, "test", false);
    tr.child_frame_id = "laser";  // the sensor, sensor_z over the footprint
    tr.header.frame_id = info.robot_footprint_frame;
    tr.transform.translation.x = 0.0;
    tr.transform.translation.z = sensor_z_;
    tf_->setTransform(tr, "test", true);
  }

  // The cloud the sensor sees from the robot at (x, y, yaw): rays in all directions
  std::shared_ptr<easynav::PointPerception> see(
    double x, double y, double yaw, const rclcpp::Time & stamp)
  {
    auto p = std::make_shared<easynav::PointPerception>();
    const Eigen::Vector3d o(x, y, sensor_z_);
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (double el = -0.6; el <= 0.6 + 1e-9; el += 0.1) {
      for (double az = -M_PI; az < M_PI; az += 0.05) {
        const Eigen::Vector3d d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
          std::sin(el));
        const Eigen::Vector3d w = cast(o, d) - o;  // in the map, from the sensor
        // In the sensor frame (the robot's yaw undone)
        p->data.push_back(
          pcl::PointXYZ(
            static_cast<float>(c * w.x() + s * w.y()), static_cast<float>(-s * w.x() + c * w.y()),
            static_cast<float>(w.z())));
      }
    }
    p->frame_id = "laser";
    p->stamp = stamp;
    p->valid = true;
    p->new_data = true;
    return p;
  }

  // The particles at these poses (x, y, yaw), each one scored from scratch
  void place(const std::vector<std::array<double, 3>> & poses)
  {
    auto & P = amcl_->particles_;
    P.resize(poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, poses[i][2]);
      P[i].pose = tf2::Transform(q, tf2::Vector3(poses[i][0], poses[i][1], 0.0));
      P[i].hits = 0.0f;
      P[i].possible_hits = 0.0f;
      P[i].weight = 1.0 / static_cast<double>(poses.size());
    }
  }

  double score(std::size_t i) const
  {
    const auto & p = amcl_->particles_[i];
    return p.possible_hits > 0 ? p.hits / p.possible_hits : 0.0;
  }

  std::shared_ptr<Bonxai::ProbabilisticMap> room_ = room();
  double sensor_z_ = 0.5;
  std::shared_ptr<easynav::LocalizerNode> node_;
  std::shared_ptr<TestAmcl> amcl_;
  easynav::NavState nav_state_;
  easynav::RTTFBuffer * tf_ {nullptr};
  rclcpp::Time now_;
};

TEST_F(NavmapAmclCorrectTest, ThePoseThatSeesTheCloudScoresBest)
{
  make({rclcpp::Parameter("loc.min_point_height", 0.3),
      rclcpp::Parameter("loc.max_point_height", 1.8)});
  nav_state_.set("laser", see(2.0, 3.5, 0.3, now_));
  place({{2.0, 3.5, 0.3}, {2.5, 3.5, 0.3}, {2.0, 3.0, 0.3}, {1.6, 3.8, 0.3}, {2.0, 3.5, 0.6},
      {2.0, 3.5, 0.0}});
  amcl_->correct(nav_state_);
  EXPECT_GT(score(0), 0.6);  // the map's probability where the points fall
  for (std::size_t i = 1; i < amcl_->particles_.size(); ++i) {
    EXPECT_LT(score(i), score(0) - 0.1) << "particle " << i;
  }
}

TEST_F(NavmapAmclCorrectTest, OnlyPointsWithinTheHeightBandAreScored)
{
  // Floor and ceiling fit any particle under them: out of the band, they are not scored
  make({rclcpp::Parameter("loc.min_point_height", 0.3),
      rclcpp::Parameter("loc.max_point_height", 1.8)});
  auto cloud = see(3.0, 3.0, 0.0, now_);
  std::size_t in_band = 0;
  for (const auto & p : cloud->data) {
    const double z = p.z + sensor_z_;
    if (z >= 0.3 && z <= 1.8) {++in_band;}
  }
  ASSERT_GT(in_band, 0u);
  ASSERT_LT(in_band, cloud->data.size());
  nav_state_.set("laser", cloud);
  place({{3.0, 3.0, 0.0}});
  amcl_->correct(nav_state_);
  EXPECT_EQ(static_cast<std::size_t>(amcl_->particles_[0].possible_hits), in_band);
  // By default, all of them
  make();
  nav_state_.set("laser", cloud);
  place({{3.0, 3.0, 0.0}});
  amcl_->correct(nav_state_);
  EXPECT_EQ(static_cast<std::size_t>(amcl_->particles_[0].possible_hits), cloud->data.size());
}

TEST_F(NavmapAmclCorrectTest, ASensorNearTheFloorIsNotBlindedByIt)
{
  // A sensor within the floor's voxels: its rays to the walls run through the floor. With the
  // occlusion check many look blocked and the true pose scores far less; without it, the true
  // pose is the best.
  sensor_z_ = 0.05;
  make({rclcpp::Parameter("loc.min_point_height", 0.3),
      rclcpp::Parameter("loc.ray_occlusion", true)});
  nav_state_.set("laser", see(3.0, 2.5, 0.2, now_));
  place({{3.0, 2.5, 0.2}, {3.4, 2.5, 0.2}});
  amcl_->correct(nav_state_);
  const double occluded = score(0);
  make({rclcpp::Parameter("loc.min_point_height", 0.3),
      rclcpp::Parameter("loc.ray_occlusion", false)});
  nav_state_.set("laser", see(3.0, 2.5, 0.2, now_));
  place({{3.0, 2.5, 0.2}, {3.4, 2.5, 0.2}});
  amcl_->correct(nav_state_);
  EXPECT_GT(score(0), 0.6);
  EXPECT_GT(score(0), score(1) + 0.1);
  EXPECT_LT(occluded, 0.5 * score(0));
}

TEST_F(NavmapAmclCorrectTest, AnOldCloudIsNotScored)
{
  make();
  auto cloud = see(3.0, 3.0, 0.0, now_);
  cloud->valid = false;  // forget_time passed
  nav_state_.set("laser", cloud);
  place({{3.0, 3.0, 0.0}});
  amcl_->correct(nav_state_);
  EXPECT_EQ(amcl_->particles_[0].possible_hits, 0.0f);
}

TEST_F(NavmapAmclCorrectTest, PointsThatAreNotFiniteAreNotCounted)
{
  make();
  auto cloud = see(3.0, 3.0, 0.0, now_);
  const auto n = cloud->data.size();
  const float inf = std::numeric_limits<float>::infinity();
  cloud->data.push_back(pcl::PointXYZ(inf, inf, inf));  // no return
  cloud->data.push_back(pcl::PointXYZ(-inf, 0.0f, 0.0f));
  cloud->data.push_back(pcl::PointXYZ(std::nanf(""), 1.0f, 1.0f));
  nav_state_.set("laser", cloud);
  place({{3.0, 3.0, 0.0}});
  amcl_->correct(nav_state_);
  EXPECT_EQ(static_cast<std::size_t>(amcl_->particles_[0].possible_hits), n);
  EXPECT_GT(score(0), 0.6);
}

TEST_F(NavmapAmclCorrectTest, AtMostCorrectMaxPointsPerSensor)
{
  for (int cap : {100, 1000}) {
    make({rclcpp::Parameter("loc.correct_max_points", cap)});
    auto cloud = see(3.0, 3.0, 0.0, now_);
    ASSERT_GT(cloud->data.size(), 1000u);
    nav_state_.set("laser", cloud);
    place({{3.0, 3.0, 0.0}});
    amcl_->correct(nav_state_);
    EXPECT_EQ(static_cast<int>(amcl_->particles_[0].possible_hits), cap);
    EXPECT_GT(score(0), 0.6) << cap;  // spread over the cloud, still the room
  }
}

TEST_F(NavmapAmclCorrectTest, AnOlderCloudIsMovedWithTheOdometry)
{
  // The cloud was taken 0.3 s ago, 0.6 m back (odometry); the particles are where the robot is
  // now: the one at the true current pose scores best
  make({rclcpp::Parameter("loc.min_point_height", 0.3)});
  const rclcpp::Time then = now_ - rclcpp::Duration::from_seconds(0.3);
  odom_at(then, 0.0);
  odom_at(now_, 0.6);
  nav_state_.set("laser", see(2.0, 3.0, 0.0, then));
  place({{2.6, 3.0, 0.0}, {2.0, 3.0, 0.0}});
  amcl_->correct(nav_state_);
  EXPECT_GT(score(0), 0.6);
  EXPECT_GT(score(0), score(1) + 0.1);
}
