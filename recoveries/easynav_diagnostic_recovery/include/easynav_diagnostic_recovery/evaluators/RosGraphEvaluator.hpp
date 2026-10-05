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
/// \brief Declaration of the RosGraphEvaluator plugin.

#ifndef EASYNAV_DIAGNOSTIC_RECOVERY__EVALUATORS__ROSGRAPHEVALUATOR_HPP_
#define EASYNAV_DIAGNOSTIC_RECOVERY__EVALUATORS__ROSGRAPHEVALUATOR_HPP_

#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "easynav_diagnostic_recovery/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class RosGraphEvaluator
 * @brief Level-1 recovery evaluator that checks EasyNav's wiring in the ROS graph.
 *
 * Every time it is activated, it discovers the topology of the EasyNav nodes — which also
 * covers every subscription their plugins made, since plugins subscribe through their parent
 * node: their subscriptions, and their velocity outputs (Twist/TwistStamped topics they
 * publish). The ROS graph does not tell which process a node belongs to, so EasyNav nodes are
 * identified by the fixed names EasyNav gives them, in this evaluator's own namespace (which
 * every EasyNav node shares). Each cycle then only checks that discovered topology against the
 * graph, reporting ERROR with hardware_id "ros_graph" when:
 * - a discovered subscription has no publisher of the same type (a misconfigured or missing
 *   topic: EasyNav is waiting for data that will never arrive), or
 * - no discovered velocity output has a subscriber outside EasyNav (nothing drives the robot).
 *   Monitoring tools listed in "ignored_consumers" (EasyNav's TUI, `ros2 topic echo`) do not
 *   count as subscribers.
 *
 * It runs in wall time (steady clock), not on the node's clock: with use_sim_time and no
 * simulator, /clock never arrives, simulated time never advances and every other plugin stays
 * silent. That case is reported on its own ("nothing publishes /clock"), instead of as one
 * unfed /clock subscription per EasyNav node.
 *
 * On OK, the message and values only report which velocity output is consumed: its topic
 * and whether it is stamped or unstamped.
 *
 * Topics listed in "ignored_topics" are optional inputs (e.g. "goal_pose", "initialpose") that
 * normally have no publisher and are not part of the topology.
 *
 * The ERROR does not depend on having an active goal: a miswired EasyNav cannot navigate
 * correctly either way, and it is meant to be handled by ShutdownRecovery, which terminates
 * EasyNav. Two margins keep that from firing too early (WARN meanwhile):
 * - "startup_grace" seconds after activation, so the nodes outside EasyNav (robot driver,
 *   simulator, sensors) have time to come up and be discovered;
 * - "error_debounce" seconds a problem must persist, to ride out transient graph changes.
 */
class RosGraphEvaluator : public easynav_diagnostic_recovery::RecoveryEvaluatorBase
{
public:
  RosGraphEvaluator() = default;
  ~RosGraphEvaluator() = default;

  void on_initialize() override;

  /// @brief Discovers EasyNav's topology in the ROS graph.
  void on_activate() override;

  /// @brief Drops the discovered topology until the next activation.
  void on_deactivate() override;

protected:
  /// @brief At "freq" in wall time: the graph must be checked even if /clock never arrives.
  bool is_time_to_update() override;

  void update(NavState & nav_state) override;

private:
  /// @brief A subscription made by an EasyNav node.
  struct Subscription
  {
    std::string node;
    std::string topic;
    std::string type;
  };

  /// @brief EasyNav's side of the ROS graph, discovered on activation.
  struct Topology
  {
    std::set<std::pair<std::string, std::string>> easynav_nodes;
    std::vector<Subscription> subscriptions;
    /// @brief (topic, type) of each Twist/TwistStamped topic published by an EasyNav node.
    std::vector<std::pair<std::string, std::string>> velocity_outputs;
    /// @brief EasyNav nodes not present in the graph at discovery time.
    std::vector<std::string> missing_nodes;
  };

  /// @brief Result of checking the topology against the current graph.
  struct GraphReport
  {
    /// @brief Discovered subscriptions that currently have no publisher of their type.
    std::vector<Subscription> unfed_subscriptions;

    /// @brief Velocity output actually consumed outside EasyNav, if any.
    std::string consumed_topic;
    std::string consumed_type;
  };

  GraphReport check_topology(const Topology & topology) const;
  bool is_ignored(const std::string & topic) const;
  bool is_ignored_consumer(const std::string & node_name) const;

  /// @brief Explains the subscriptions without publisher and how to declare them optional.
  std::string describe_unfed(const std::vector<Subscription> & unfed) const;

  /// @brief Entry to add to "ignored_topics" for \p topic: relative to this node's namespace, so
  /// it also works for other EasyNav instances in other namespaces.
  std::string ignored_topics_entry(const std::string & topic) const;

  /// @brief Topics left out of the topology. An entry matches a fully-qualified topic name
  /// exactly, or its trailing path component(s).
  std::vector<std::string> ignored_topics_ {
    "goal_pose", "initialpose", "incoming_map", "incoming_occ_map", "incoming_pc2_map"};

  /// @brief Nodes whose subscription to a velocity output does not count as "consumed": tools
  /// that only watch cmd_vel (EasyNav's TUI, `ros2 topic echo`) and would otherwise hide that
  /// nothing drives the robot. A trailing "*" matches a node-name prefix.
  std::vector<std::string> ignored_consumers_ {"easynav_tui_*", "_ros2cli*"};

  /// @brief Seconds after activation during which problems are only WARN.
  double startup_grace_ {15.0};

  /// @brief Seconds a problem must persist before being reported as ERROR.
  double error_debounce_ {2.0};

  /// @brief Graph discovery happens in wall time, whatever the node's clock is (with
  /// use_sim_time, /clock may not even be running yet at activation).
  rclcpp::Clock steady_clock_ {RCL_STEADY_TIME};

  /// @brief Update rate ("<plugin_name>.freq", declared by MethodBase), and last update.
  double frequency_ {10.0};
  rclcpp::Time last_update_ {0, 0, RCL_STEADY_TIME};

  /// @brief When the topology was last discovered (i.e. the last activation), steady clock.
  rclcpp::Time activated_at_ {0, 0, RCL_STEADY_TIME};

  /// @brief Since when the current problem has been observed continuously (steady clock).
  std::optional<rclcpp::Time> problem_since_;

  /// @brief Discovered on activation (lifecycle transition), read every cycle (non-RT thread).
  std::mutex topology_mutex_;
  std::optional<Topology> topology_;
};

}  // namespace easynav

#endif  // EASYNAV_DIAGNOSTIC_RECOVERY__EVALUATORS__ROSGRAPHEVALUATOR_HPP_
