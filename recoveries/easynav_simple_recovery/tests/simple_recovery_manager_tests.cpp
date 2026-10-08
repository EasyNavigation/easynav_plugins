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
/// \brief Tests for SimpleRecoveryManager: each case, its actions and sequences of cases.

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/testing/LogCapture.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_core/SystemActions.hpp"
#include "easynav_core/VelocityCommand.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_recovery/RecoveryManagerNode.hpp"
#include "easynav_simple_recovery/SimpleRecoveryManager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;
using easynav::VelocitySource;
using Mitigation = easynav::SimpleRecoveryManager::Mitigation;
namespace vc = easynav::velocity_command;

namespace
{

class RecordingSystemActions : public easynav::SystemActions
{
public:
  void abort_mission(const std::string & reason) override {aborted.push_back(reason);}
  void request_shutdown(const std::string & reason) override {shutdowns.push_back(reason);}
  void hold_mission_progress(bool hold) override {holds.push_back(hold);}
  bool request_reconfigure(
    const std::vector<easynav::ParameterChange> & changes, const std::string &) override
  {
    reconfigures.push_back(changes);
    return accept;
  }
  bool request_restore_parameters(const std::string &) override
  {
    ++restores;
    return accept;
  }
  bool accept {true};
  std::vector<std::string> aborted;
  std::vector<std::string> shutdowns;
  std::vector<bool> holds;
  std::vector<std::vector<easynav::ParameterChange>> reconfigures;
  int restores {0};
};

}  // namespace

class SimpleRecoveryManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void make_manager(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    std::vector<rclcpp::Parameter> params{
      {"recovery_manager.stuck_time", 0.1},
      {"recovery_manager.backup_time", 0.1},
      {"recovery_manager.relocalize_timeout", 0.1},
    };
    params.insert(params.end(), overrides.begin(), overrides.end());
    // The system's geometry, as SystemNode leaves it (not configured explicitly).
    easynav::RobotGeometryRegistry::getInstance()->set_geometry({0.3, 0.3, 2.0});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "simple_recovery_test_node", rclcpp::NodeOptions().parameter_overrides(params));
    manager_ = std::make_shared<easynav::SimpleRecoveryManager>();
    manager_->initialize(node_, "recovery_manager");
    actions_ = std::make_shared<RecordingSystemActions>();
    manager_->set_system_actions(actions_);
    nav_state_ = std::make_unique<easynav::NavState>();
    set_pose(0.0, 0.0);
  }

  void SetUp() override {make_manager();}

  // What EasyNav does with the last request_reconfigure()/request_restore_parameters(): records
  // the changed parameters in NavState and reloads the recovery system (a new instance).
  void apply_reconfigure(bool restore = false)
  {
    std::vector<std::string> changed;
    if (!restore) {
      for (const auto & change : actions_->reconfigures.back()) {
        changed.push_back(change.node + "/" + change.parameter.get_name());
      }
    }
    nav_state_->set("reconfigured_parameters", changed);
    manager_ = std::make_shared<easynav::SimpleRecoveryManager>();
    manager_->initialize(node_, "recovery_manager");
    manager_->set_system_actions(actions_);
    manager_->on_activate();
  }

  void set_mission(bool active)
  {
    nav_msgs::msg::Goals goals;
    if (active) {
      goals.goals.emplace_back();
    }
    nav_state_->set("goals", goals);
  }

  void set_pose(double x, double y, double variance = 0.01)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.covariance[0] = variance;
    odom.pose.covariance[7] = variance;
    nav_state_->set("robot_pose", odom);
  }

  // The controller's command, as ControllerNode leaves it before the recovery's RT cycle.
  void controller_commands(double vx, double wz = 0.0)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = vx;
    cmd.twist.angular.z = wz;
    nav_state_->set("cmd_vel", cmd);
    vc::propose(*nav_state_, VelocitySource::CONTROLLER, cmd);
  }

  // A scan in the robot frame, age seconds old.
  void perceive(const std::vector<std::tuple<float, float, float>> & points, double age = 0.0)
  {
    easynav::PointPerception perception;
    for (const auto & [x, y, z] : points) {
      perception.data.push_back(pcl::PointXYZ(x, y, z));
    }
    perception.frame_id = "base_link";
    perception.stamp = node_->now() - rclcpp::Duration::from_seconds(age);
    perception.valid = true;
    nav_state_->set("scan", perception);
  }

  // One RT cycle; returns what the mux would get from the recovery (takeover, override).
  std::pair<std::optional<geometry_msgs::msg::TwistStamped>,
    std::optional<geometry_msgs::msg::TwistStamped>> rt_cycle()
  {
    manager_->internal_update_rt(*nav_state_);
    return {vc::take(*nav_state_, VelocitySource::TAKEOVER),
      vc::take(*nav_state_, VelocitySource::OVERRIDE)};
  }

  void cycle() {manager_->internal_update(*nav_state_);}

  // Reaches the stuck case: commanded forward, not moving, for longer than stuck_time.
  void get_stuck()
  {
    controller_commands(0.3);
    cycle();
    rclcpp::sleep_for(150ms);
    cycle();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<easynav::SimpleRecoveryManager> manager_;
  std::shared_ptr<RecordingSystemActions> actions_;
  std::unique_ptr<easynav::NavState> nav_state_;
};

// ─── Fast recovery (RT): obstacle ahead ─────────────────────────────────────────────────────

TEST_F(SimpleRecoveryManagerTest, NothingToDoByDefault)
{
  set_mission(true);
  controller_commands(0.3);
  perceive({{2.0, 0.0, 0.5}});
  cycle();

  auto [takeover, override_cmd] = rt_cycle();
  EXPECT_FALSE(takeover);
  EXPECT_FALSE(override_cmd);
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_TRUE(actions_->holds.empty());
  EXPECT_TRUE(actions_->aborted.empty());
  EXPECT_TRUE(actions_->shutdowns.empty());
}

TEST_F(SimpleRecoveryManagerTest, BrakesDeadBeforeAnObstacleAhead)
{
  controller_commands(0.3, 0.2);
  perceive({{0.5, 0.0, 0.5}});  // Within robot_radius + stop_distance = 0.6

  EXPECT_TRUE(manager_->internal_update_rt(*nav_state_));
  auto stop = vc::take(*nav_state_, VelocitySource::OVERRIDE);
  ASSERT_TRUE(stop);
  EXPECT_DOUBLE_EQ(stop->twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(stop->twist.angular.z, 0.0);
}

TEST_F(SimpleRecoveryManagerTest, BrakesWithoutAMission)
{
  set_mission(false);
  controller_commands(0.3);
  perceive({{0.4, 0.1, 0.5}});
  EXPECT_TRUE(rt_cycle().second);
}

TEST_F(SimpleRecoveryManagerTest, IgnoresPointsOutsideTheAreaAhead)
{
  controller_commands(0.3);
  for (const auto & point : std::vector<std::tuple<float, float, float>>{
    {0.7, 0.0, 0.5},      // Farther than 0.6
    {0.4, 0.4, 0.5},      // Beside the robot
    {0.4, -0.4, 0.5},
    {-0.4, 0.0, 0.5},     // Behind
    {0.4, 0.0, 0.01},     // Floor
    {0.4, 0.0, 2.5}})     // Above the robot
  {
    perceive({point});
    EXPECT_FALSE(rt_cycle().second) <<
      std::get<0>(point) << ", " << std::get<1>(point) << ", " << std::get<2>(point);
  }
}

TEST_F(SimpleRecoveryManagerTest, BrakesIfAnyPointIsAhead)
{
  controller_commands(0.3);
  perceive({{3.0, 0.0, 0.5}, {-0.4, 0.0, 0.5}, {0.35, -0.25, 0.3}});
  EXPECT_TRUE(rt_cycle().second);
}

TEST_F(SimpleRecoveryManagerTest, DoesNotBrakeUnlessMovingForward)
{
  perceive({{0.4, 0.0, 0.5}});

  EXPECT_FALSE(rt_cycle().second) << "no controller command";
  for (const auto & [vx, wz] : std::vector<std::pair<double, double>>{
    {0.0, 0.0}, {0.0, 0.5}, {-0.2, 0.0}})
  {
    controller_commands(vx, wz);
    EXPECT_FALSE(rt_cycle().second) << vx << ", " << wz;
  }
}

TEST_F(SimpleRecoveryManagerTest, DoesNotBrakeWithoutPerceptions)
{
  controller_commands(0.3);
  EXPECT_FALSE(rt_cycle().second);
}

TEST_F(SimpleRecoveryManagerTest, AreaAheadFollowsTheRobotGeometry)
{
  make_manager();
  easynav::RobotGeometryRegistry::getInstance()->set_geometry(
    {0.6, 0.6, 1.0}, {"radius", "height"});
  manager_ = std::make_shared<easynav::SimpleRecoveryManager>();
  manager_->initialize(node_, "recovery_manager");
  controller_commands(0.3);

  perceive({{0.8, 0.5, 0.5}});  // Within radius 0.6 + stop_distance 0.3, and its width
  EXPECT_TRUE(rt_cycle().second);
  perceive({{0.4, 0.0, 1.5}});  // Above its height
  EXPECT_FALSE(rt_cycle().second);
}

TEST_F(SimpleRecoveryManagerTest, DeprecatedGeometryParametersStillApply)
{
  make_manager(
  {
    {"recovery_manager.robot_radius", 0.6},
    {"recovery_manager.max_obstacle_z", 1.0}});
  controller_commands(0.3);
  perceive({{0.8, 0.5, 0.5}});
  EXPECT_TRUE(rt_cycle().second);
  perceive({{0.4, 0.0, 1.5}});
  EXPECT_FALSE(rt_cycle().second);
}

TEST_F(SimpleRecoveryManagerTest, RobotGeometryTakesPrecedenceOverDeprecatedParameters)
{
  make_manager({{"recovery_manager.robot_radius", 0.6}});
  easynav::RobotGeometryRegistry::getInstance()->set_geometry({0.3, 0.3, 2.0}, {"radius"});
  manager_ = std::make_shared<easynav::SimpleRecoveryManager>();
  manager_->initialize(node_, "recovery_manager");
  controller_commands(0.3);
  perceive({{0.8, 0.5, 0.5}});
  EXPECT_FALSE(rt_cycle().second) << "radius 0.3: outside the area ahead";
}

TEST_F(SimpleRecoveryManagerTest, WarnsAboutDeprecatedGeometryParameters)
{
  {
    easynav::testing::LogCapture log;
    make_manager(
    {
      {"recovery_manager.robot_radius", 0.6},
      {"recovery_manager.max_obstacle_z", 1.0}});
    EXPECT_EQ(
      log.count(
        {"'recovery_manager.robot_radius' is deprecated: configure",
          "system_node.robot_geometry.radius"}), 1u);
    EXPECT_EQ(
      log.count(
        {"'recovery_manager.max_obstacle_z' is deprecated: configure",
          "system_node.robot_geometry.height"}), 1u);
  }
  {
    make_manager({{"recovery_manager.robot_radius", 0.6}});
    easynav::RobotGeometryRegistry::getInstance()->set_geometry({0.3, 0.3, 2.0}, {"radius"});
    easynav::testing::LogCapture log;
    manager_ = std::make_shared<easynav::SimpleRecoveryManager>();
    manager_->initialize(node_, "recovery_manager");
    EXPECT_EQ(log.count({"'recovery_manager.robot_radius' is deprecated and ignored"}), 1u);
  }
  {
    easynav::testing::LogCapture log;
    make_manager();
    EXPECT_EQ(log.count({"deprecated"}), 0u);
  }
}

TEST_F(SimpleRecoveryManagerTest, CustomStopDistance)
{
  make_manager({{"recovery_manager.stop_distance", 1.0}});
  controller_commands(0.3);
  perceive({{1.2, 0.0, 0.5}});
  EXPECT_TRUE(rt_cycle().second);
}

// ─── Case 1: sensors lost ────────────────────────────────────────────────────────────────────

// sensors_timeout = 0.2 s; the first cycle starts watching the sensors.
class SensorsLostTest : public SimpleRecoveryManagerTest
{
protected:
  void SetUp() override
  {
    make_manager({{"recovery_manager.sensors_timeout", 0.2}});
    set_mission(true);
  }

  // As a sensor leaves its perception before receiving anything.
  void sensor_without_data(const std::string & name = "scan")
  {
    nav_state_->set(name, easynav::PointPerception());
  }

  bool shut_down() const {return !actions_->shutdowns.empty();}
};

TEST_F(SensorsLostTest, NoDataAtStartIsNotLostWithinTheTimeout)
{
  sensor_without_data();
  cycle();
  rclcpp::sleep_for(100ms);
  cycle();
  EXPECT_FALSE(shut_down());
}

TEST_F(SensorsLostTest, NoDataEverIsLostAfterTheTimeout)
{
  sensor_without_data();
  cycle();
  rclcpp::sleep_for(250ms);
  cycle();
  ASSERT_EQ(actions_->shutdowns.size(), 1u);
  EXPECT_NE(actions_->shutdowns[0].find("no sensor data"), std::string::npos);
}

TEST_F(SensorsLostTest, DataArrivingInTimeIsNotLostUntilItStops)
{
  sensor_without_data();
  cycle();
  rclcpp::sleep_for(150ms);
  perceive({{2.0, 0.0, 0.5}});
  cycle();
  rclcpp::sleep_for(150ms);
  cycle();
  EXPECT_FALSE(shut_down()) << "last data 0.15 s ago";

  rclcpp::sleep_for(100ms);
  cycle();
  EXPECT_TRUE(shut_down()) << "last data 0.25 s ago";
}

TEST_F(SensorsLostTest, InvalidDataDoesNotCount)
{
  cycle();
  rclcpp::sleep_for(250ms);
  easynav::PointPerception not_valid;
  not_valid.stamp = node_->now();
  nav_state_->set("scan", not_valid);
  cycle();
  EXPECT_TRUE(shut_down());
}

TEST_F(SensorsLostTest, OneSensorWithNewDataIsEnough)
{
  perceive({{2.0, 0.0, 0.5}});
  cycle();
  for (int i = 0; i < 3; ++i) {
    rclcpp::sleep_for(100ms);
    easynav::PointPerception other;
    other.valid = true;
    other.stamp = node_->now();
    nav_state_->set("other_scan", other);  // "scan" stays stale
    cycle();
  }
  EXPECT_FALSE(shut_down());
}

TEST_F(SensorsLostTest, ASensorWithoutDataDoesNotHideALostOne)
{
  perceive({{2.0, 0.0, 0.5}});
  cycle();
  sensor_without_data("no_data_yet");
  rclcpp::sleep_for(250ms);
  cycle();
  EXPECT_TRUE(shut_down());
}

TEST_F(SensorsLostTest, SameDataAgainIsNotNewData)
{
  perceive({{2.0, 0.0, 0.5}});
  cycle();
  const auto scan = nav_state_->get<easynav::PointPerception>("scan");
  for (int i = 0; i < 3; ++i) {
    rclcpp::sleep_for(100ms);
    nav_state_->set("scan", scan);  // Sensors keep leaving their last perception
    cycle();
  }
  EXPECT_TRUE(shut_down());
}

TEST_F(SensorsLostTest, NoPointSensorsIsNeverLost)
{
  cycle();
  rclcpp::sleep_for(250ms);
  cycle();
  EXPECT_FALSE(shut_down());
}

TEST_F(SensorsLostTest, LostEvenWithoutAMission)
{
  set_mission(false);
  sensor_without_data();
  cycle();
  rclcpp::sleep_for(250ms);
  cycle();
  EXPECT_TRUE(shut_down());
}

TEST_F(SensorsLostTest, ReactivationGivesTheTimeoutAgain)
{
  perceive({{2.0, 0.0, 0.5}});
  cycle();
  rclcpp::sleep_for(250ms);  // E.g. deactivated for a while
  manager_->on_activate();
  cycle();
  EXPECT_FALSE(shut_down());

  rclcpp::sleep_for(250ms);
  cycle();
  EXPECT_TRUE(shut_down());
}

// Simulated time: stops with the simulator, and restarts from zero with it.
class SensorsLostSimTimeTest : public SensorsLostTest
{
protected:
  void SetUp() override
  {
    make_manager(
    {
      {"recovery_manager.sensors_timeout", 0.2},
      {"use_sim_time", true}});    // No /clock: the node's time does not advance
    set_mission(true);
  }

  void scan_at(double sim_seconds)
  {
    easynav::PointPerception scan;
    scan.valid = true;
    scan.stamp = rclcpp::Time(static_cast<int64_t>(sim_seconds * 1e9), RCL_ROS_TIME);
    nav_state_->set("scan", scan);
  }
};

TEST_F(SensorsLostSimTimeTest, SimulatorStoppedIsLost)
{
  scan_at(100.0);
  cycle();
  rclcpp::sleep_for(100ms);
  cycle();
  EXPECT_FALSE(shut_down());

  rclcpp::sleep_for(150ms);  // No new data (and no /clock) for 0.25 s of real time
  cycle();
  EXPECT_TRUE(shut_down());
}

TEST_F(SensorsLostSimTimeTest, DataKeepsArrivingIsNotLost)
{
  for (int i = 0; i < 4; ++i) {
    scan_at(100.0 + 0.1 * i);
    cycle();
    rclcpp::sleep_for(100ms);
  }
  EXPECT_FALSE(shut_down());
}

TEST_F(SensorsLostSimTimeTest, SimulatorRestartedIsNewData)
{
  scan_at(100.0);
  cycle();
  rclcpp::sleep_for(150ms);
  scan_at(0.5);  // Time went back
  cycle();
  rclcpp::sleep_for(150ms);
  cycle();
  EXPECT_FALSE(shut_down());
}

TEST_F(SensorsLostTest, PrevailsOverOtherCases)
{
  set_pose(0.0, 0.0, 5.0);
  sensor_without_data();
  cycle();
  ASSERT_EQ(manager_->get_mitigation(), Mitigation::ROTATE);

  rclcpp::sleep_for(250ms);
  cycle();
  EXPECT_TRUE(shut_down());
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_EQ(actions_->holds, std::vector<bool>({true, false}));
}

// ─── Case 2: localization lost ───────────────────────────────────────────────────────────────

TEST_F(SimpleRecoveryManagerTest, LocalizationLostHoldsAndRotatesUntilRelocalized)
{
  set_mission(true);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::ROTATE);
  EXPECT_EQ(actions_->holds, std::vector<bool>({true}));

  auto [takeover, override_cmd] = rt_cycle();
  ASSERT_TRUE(takeover);
  EXPECT_FALSE(override_cmd);
  EXPECT_DOUBLE_EQ(takeover->twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(takeover->twist.angular.z, 0.5);

  cycle();  // Still lost: keeps rotating, holds once
  EXPECT_EQ(actions_->holds, std::vector<bool>({true}));
  EXPECT_TRUE(rt_cycle().first) << "commanded on every RT cycle";

  set_pose(0.0, 0.0, 0.1);
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_EQ(actions_->holds, std::vector<bool>({true, false}));
  EXPECT_FALSE(rt_cycle().first);
  EXPECT_TRUE(actions_->aborted.empty());
}

TEST_F(SimpleRecoveryManagerTest, LostInYAlsoCounts)
{
  set_mission(true);
  nav_msgs::msg::Odometry odom;
  odom.pose.covariance[7] = 5.0;
  nav_state_->set("robot_pose", odom);
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::ROTATE);
}

TEST_F(SimpleRecoveryManagerTest, LocalizationLostWithoutAMissionIsIgnored)
{
  set_mission(false);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_TRUE(actions_->holds.empty());
}

TEST_F(SimpleRecoveryManagerTest, NotRelocalizedInTimeAbortsTheMission)
{
  set_mission(true);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  rclcpp::sleep_for(150ms);
  cycle();

  EXPECT_EQ(actions_->aborted, std::vector<std::string>({"could not relocalize"}));
  EXPECT_EQ(actions_->holds, std::vector<bool>({true, false}));
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_FALSE(rt_cycle().first);
}

TEST_F(SimpleRecoveryManagerTest, MissionEndingReleasesTheHold)
{
  set_mission(true);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  set_mission(false);  // E.g. cancelled
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_EQ(actions_->holds, std::vector<bool>({true, false}));
}

TEST_F(SimpleRecoveryManagerTest, RotatingNeverBrakes)
{
  set_mission(true);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  controller_commands(0.3);
  perceive({{0.4, 0.0, 0.5}});
  auto [takeover, override_cmd] = rt_cycle();
  EXPECT_TRUE(takeover);
  EXPECT_FALSE(override_cmd);
}

// ─── Case 3: stuck ───────────────────────────────────────────────────────────────────────────

TEST_F(SimpleRecoveryManagerTest, StuckBacksUpThenTheControllerDrivesAgain)
{
  set_mission(true);
  get_stuck();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::BACK_UP);

  auto [takeover, override_cmd] = rt_cycle();
  ASSERT_TRUE(takeover);
  EXPECT_DOUBLE_EQ(takeover->twist.linear.x, -0.1);
  EXPECT_DOUBLE_EQ(takeover->twist.angular.z, 0.0);

  cycle();  // Before backup_time
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::BACK_UP);

  rclcpp::sleep_for(150ms);
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_FALSE(rt_cycle().first);
  EXPECT_TRUE(actions_->holds.empty()) << "backing up does not hold the mission";
  EXPECT_TRUE(actions_->aborted.empty());
}

TEST_F(SimpleRecoveryManagerTest, MovingIsNotStuck)
{
  set_mission(true);
  controller_commands(0.3);
  for (int i = 0; i < 4; ++i) {
    set_pose(0.1 * i, 0.0);
    cycle();
    rclcpp::sleep_for(50ms);
  }
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
}

TEST_F(SimpleRecoveryManagerTest, NotCommandedIsNotStuck)
{
  set_mission(true);
  for (double vx : {0.0, 0.005}) {
    controller_commands(vx);
    cycle();
    rclcpp::sleep_for(150ms);
    cycle();
    EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE) << vx;
  }
}

TEST_F(SimpleRecoveryManagerTest, StuckWithoutAMissionIsIgnored)
{
  set_mission(false);
  get_stuck();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
}

TEST_F(SimpleRecoveryManagerTest, TooManyAttemptsSlowDownThenAbort)
{
  make_manager({{"recovery_manager.max_backup_attempts", 2}});
  set_mission(true);

  auto back_up_twice = [this]() {
      for (int attempt = 1; attempt <= 2; ++attempt) {
        get_stuck();
        ASSERT_EQ(manager_->get_mitigation(), Mitigation::BACK_UP) << attempt;
        rclcpp::sleep_for(150ms);
        cycle();
        ASSERT_EQ(manager_->get_mitigation(), Mitigation::NONE) << attempt;
      }
    };

  back_up_twice();
  get_stuck();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_TRUE(actions_->aborted.empty()) << "slows down first";
  ASSERT_EQ(actions_->reconfigures.size(), 1u);
  ASSERT_EQ(actions_->reconfigures[0].size(), 1u);
  EXPECT_EQ(actions_->reconfigures[0][0].node, "controller_node");
  EXPECT_EQ(actions_->reconfigures[0][0].parameter.get_name(), "robot_limits.max_linear_vel");
  EXPECT_DOUBLE_EQ(actions_->reconfigures[0][0].parameter.as_double(), 0.1);

  // EasyNav reconfigures: a new instance, which learns from NavState that it slowed down.
  apply_reconfigure();
  back_up_twice();
  get_stuck();
  EXPECT_EQ(actions_->reconfigures.size(), 1u) << "slows down once";
  EXPECT_EQ(actions_->aborted, std::vector<std::string>({"stuck after 2 attempts"}));
}

TEST_F(SimpleRecoveryManagerTest, RejectedSlowDownAbortsTheMission)
{
  // E.g. EasyNav in safety mode: its configuration is frozen.
  make_manager({{"recovery_manager.max_backup_attempts", 0}});
  actions_->accept = false;
  set_mission(true);
  get_stuck();
  EXPECT_EQ(actions_->reconfigures.size(), 1u);
  EXPECT_EQ(actions_->aborted, std::vector<std::string>({"stuck, and unable to slow down"}));

  // A new mission, stuck again: asks again (it may be accepted now), not in a loop.
  set_mission(false);
  cycle();
  set_mission(true);
  get_stuck();
  EXPECT_EQ(actions_->reconfigures.size(), 2u);
  EXPECT_EQ(actions_->aborted.size(), 2u);
  EXPECT_EQ(actions_->restores, 0) << "nothing was slowed down";
}

TEST_F(SimpleRecoveryManagerTest, NoSlowDownIfDisabled)
{
  make_manager(
  {
    {"recovery_manager.max_backup_attempts", 1},
    {"recovery_manager.slow_down_max_linear_vel", 0.0}});
  set_mission(true);
  get_stuck();
  rclcpp::sleep_for(150ms);
  cycle();
  get_stuck();
  EXPECT_TRUE(actions_->reconfigures.empty());
  EXPECT_EQ(actions_->aborted, std::vector<std::string>({"stuck after 1 attempts"}));
}

TEST_F(SimpleRecoveryManagerTest, SpeedRestoredWhenTheMissionEnds)
{
  make_manager(
  {
    {"recovery_manager.max_backup_attempts", 0},
    {"recovery_manager.slow_down_max_linear_vel", 0.05}});
  set_mission(true);
  get_stuck();
  ASSERT_EQ(actions_->reconfigures.size(), 1u);
  EXPECT_DOUBLE_EQ(actions_->reconfigures[0][0].parameter.as_double(), 0.05);
  apply_reconfigure();

  cycle();
  EXPECT_EQ(actions_->restores, 0) << "still in the mission";

  set_mission(false);
  cycle();
  EXPECT_EQ(actions_->restores, 1);
  apply_reconfigure(true);
  cycle();
  EXPECT_EQ(actions_->restores, 1) << "nothing left to restore";
}

TEST_F(SimpleRecoveryManagerTest, NothingToRestoreWithoutASlowDown)
{
  set_mission(true);
  cycle();
  set_mission(false);
  cycle();
  nav_state_->set("reconfigured_parameters", std::vector<std::string>{"other_node/other"});
  cycle();
  EXPECT_EQ(actions_->restores, 0);
}

TEST_F(SimpleRecoveryManagerTest, AttemptsRestartWithANewMission)
{
  make_manager({{"recovery_manager.max_backup_attempts", 1}});
  set_mission(true);
  get_stuck();
  rclcpp::sleep_for(150ms);
  cycle();

  set_mission(false);
  cycle();
  set_mission(true);
  get_stuck();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::BACK_UP);
  EXPECT_TRUE(actions_->aborted.empty());
}

TEST_F(SimpleRecoveryManagerTest, LocalizationLostPrevailsOverStuck)
{
  set_mission(true);
  get_stuck();
  ASSERT_EQ(manager_->get_mitigation(), Mitigation::BACK_UP);

  set_pose(0.0, 0.0, 5.0);
  cycle();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::ROTATE);
  EXPECT_EQ(actions_->holds, std::vector<bool>({true}));
}

TEST_F(SimpleRecoveryManagerTest, BackingUpNeverBrakes)
{
  set_mission(true);
  get_stuck();
  perceive({{0.4, 0.0, 0.5}});
  controller_commands(0.3);
  auto [takeover, override_cmd] = rt_cycle();
  EXPECT_TRUE(takeover);
  EXPECT_FALSE(override_cmd);
}

// ─── Lifecycle ───────────────────────────────────────────────────────────────────────────────

TEST_F(SimpleRecoveryManagerTest, DeactivationStopsTheMitigation)
{
  set_mission(true);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  manager_->on_deactivate();
  EXPECT_EQ(manager_->get_mitigation(), Mitigation::NONE);
  EXPECT_EQ(actions_->holds, std::vector<bool>({true, false}));
  EXPECT_FALSE(rt_cycle().first);
}

TEST_F(SimpleRecoveryManagerTest, ReadsItsParameters)
{
  make_manager(
  {
    {"recovery_manager.rotate_speed", 0.8},
    {"recovery_manager.backup_speed", 0.25}});
  set_mission(true);
  set_pose(0.0, 0.0, 5.0);
  cycle();
  EXPECT_DOUBLE_EQ(rt_cycle().first->twist.angular.z, 0.8);

  set_pose(0.0, 0.0, 0.1);
  get_stuck();
  EXPECT_DOUBLE_EQ(rt_cycle().first->twist.linear.x, -0.25);
}

TEST_F(SimpleRecoveryManagerTest, SurvivesASecondInitialize)
{
  manager_ = std::make_shared<easynav::SimpleRecoveryManager>();
  EXPECT_NO_THROW(manager_->initialize(node_, "recovery_manager"));
}

TEST_F(SimpleRecoveryManagerTest, LoadedByRecoveryNode)
{
  auto recovery_node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions().append_parameter_override(
      "recovery_manager.plugin", std::string("easynav_simple_recovery/SimpleRecoveryManager")));
  recovery_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    recovery_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(
    std::dynamic_pointer_cast<easynav::SimpleRecoveryManager>(
      recovery_node->get_recovery_manager()), nullptr);
}
