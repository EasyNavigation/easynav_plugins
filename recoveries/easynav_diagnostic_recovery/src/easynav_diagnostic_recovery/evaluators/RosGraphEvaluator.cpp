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
/// \brief Implementation of the RosGraphEvaluator class.

#include <algorithm>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "easynav_common/Parameters.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "easynav_diagnostic_recovery/evaluators/RosGraphEvaluator.hpp"

namespace easynav
{

namespace
{

constexpr char kTwistType[] = "geometry_msgs/msg/Twist";
constexpr char kTwistStampedType[] = "geometry_msgs/msg/TwistStamped";
constexpr char kClockType[] = "rosgraph_msgs/msg/Clock";

// Names EasyNav gives its nodes (see each *Node constructor); SystemNode creates all of them in
// its own namespace.
const std::vector<std::string> kEasyNavNodes {
  "system_node", "controller_node", "localizer_node", "maps_manager_node", "planner_node",
  "sensors_node", "recovery_node"};

std::string fully_qualified(const std::string & name, const std::string & ns)
{
  return (ns.empty() || ns.back() == '/') ? ns + name : ns + "/" + name;
}

// "nav_msgs/msg/Odometry" -> "nav_msgs/Odometry"
std::string short_type(const std::string & type)
{
  const auto pos = type.find("/msg/");
  return pos == std::string::npos ? type : type.substr(0, pos) + type.substr(pos + 4);
}

std::string join(const std::vector<std::string> & items, const std::string & sep)
{
  std::string ret;
  for (const auto & item : items) {
    if (!ret.empty()) {ret += sep;}
    ret += item;
  }
  return ret;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

}  // namespace

void RosGraphEvaluator::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  if (!node->has_parameter(plugin_name + ".ignored_topics")) {
    node->declare_parameter<std::vector<std::string>>(
      plugin_name + ".ignored_topics", ignored_topics_);
  }
  if (!node->has_parameter(plugin_name + ".ignored_consumers")) {
    node->declare_parameter<std::vector<std::string>>(
      plugin_name + ".ignored_consumers", ignored_consumers_);
  }
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".startup_grace",
      startup_grace_);
  easynav::declare_parameter_if_absent<double>(*node, plugin_name + ".error_debounce",
      error_debounce_);

  node->get_parameter<std::vector<std::string>>(plugin_name + ".ignored_topics", ignored_topics_);
  node->get_parameter<std::vector<std::string>>(
    plugin_name + ".ignored_consumers", ignored_consumers_);
  node->get_parameter<double>(plugin_name + ".startup_grace", startup_grace_);
  node->get_parameter<double>(plugin_name + ".error_debounce", error_debounce_);

  node->get_parameter<double>(plugin_name + ".freq", frequency_);
}

bool RosGraphEvaluator::is_time_to_update()
{
  const auto now = steady_clock_.now();
  if (last_update_.nanoseconds() != 0 && (now - last_update_).seconds() < 1.0 / frequency_) {
    return false;
  }
  last_update_ = now;
  return true;
}

bool RosGraphEvaluator::is_ignored(const std::string & topic) const
{
  return std::any_of(
    ignored_topics_.begin(), ignored_topics_.end(), [&topic](const std::string & entry) {
      if (entry.empty()) {return false;}
      if (topic == entry) {return true;}
      const std::string suffix = entry.front() == '/' ? entry : "/" + entry;
      // ">=": in the root namespace "/goal_pose" is exactly the suffix of "goal_pose".
      return topic.size() >= suffix.size() &&
             topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0;
    });
}

void RosGraphEvaluator::on_activate()
{
  Topology topology;
  auto graph = get_node()->get_node_graph_interface();
  const std::string own_ns = get_node()->get_namespace();

  for (const auto & name : kEasyNavNodes) {
    topology.easynav_nodes.emplace(name, own_ns);
  }

  std::set<std::pair<std::string, std::string>> present;
  for (const auto & name_ns : graph->get_node_names_and_namespaces()) {
    present.insert(name_ns);
  }

  // SystemNode configures every EasyNav node before activating any, so by now all their
  // subscriptions and publishers (their plugins' included) already exist.
  for (const auto & [name, ns] : topology.easynav_nodes) {
    const std::string fq_name = fully_qualified(name, ns);
    if (present.count({name, ns}) == 0) {
      topology.missing_nodes.push_back(fq_name);
      continue;
    }

    std::map<std::string, std::vector<std::string>> subscriptions, publications;
    try {
      subscriptions = graph->get_subscriber_names_and_types_by_node(name, ns);
      publications = graph->get_publisher_names_and_types_by_node(name, ns);
    } catch (const std::exception &) {
      // Vanished between listing the graph and querying it.
      topology.missing_nodes.push_back(fq_name);
      continue;
    }

    for (const auto & [topic, types] : subscriptions) {
      if (is_ignored(topic)) {continue;}
      for (const auto & type : types) {
        topology.subscriptions.push_back({fq_name, topic, type});
      }
    }

    for (const auto & [topic, types] : publications) {
      for (const auto & type : types) {
        if (type == kTwistType || type == kTwistStampedType) {
          topology.velocity_outputs.emplace_back(topic, type);
        }
      }
    }
  }

  const std::string missing = topology.missing_nodes.empty() ? "" :
    " (missing: " + join(topology.missing_nodes, ", ") + ")";
  RCLCPP_INFO(
    get_node()->get_logger(),
    "RosGraphEvaluator [%s]: discovered %zu subscriptions and %zu velocity outputs in %zu "
    "EasyNav nodes%s", get_plugin_name().c_str(), topology.subscriptions.size(),
    topology.velocity_outputs.size(),
    topology.easynav_nodes.size() - topology.missing_nodes.size(), missing.c_str());

  std::lock_guard<std::mutex> lock(topology_mutex_);
  topology_ = std::move(topology);
  activated_at_ = steady_clock_.now();
  problem_since_.reset();
}

void RosGraphEvaluator::on_deactivate()
{
  std::lock_guard<std::mutex> lock(topology_mutex_);
  topology_.reset();
  problem_since_.reset();
}

bool RosGraphEvaluator::is_ignored_consumer(const std::string & node_name) const
{
  return std::any_of(
    ignored_consumers_.begin(), ignored_consumers_.end(), [&node_name](const std::string & entry) {
      if (!entry.empty() && entry.back() == '*') {
        return node_name.compare(0, entry.size() - 1, entry, 0, entry.size() - 1) == 0;
      }
      return node_name == entry;
    });
}

RosGraphEvaluator::GraphReport RosGraphEvaluator::check_topology(const Topology & topology) const
{
  GraphReport report;
  auto graph = get_node()->get_node_graph_interface();

  for (const auto & sub : topology.subscriptions) {
    const auto publishers = graph->get_publishers_info_by_topic(sub.topic);
    const bool fed = std::any_of(
      publishers.begin(), publishers.end(), [&sub](const rclcpp::TopicEndpointInfo & info) {
        return info.topic_type() == sub.type;
      });
    if (!fed) {
      report.unfed_subscriptions.push_back(sub);
    }
  }

  // Only consumers outside EasyNav count: a plugin listening to EasyNav's own cmd_vel (e.g. a
  // localizer using it as control input) is not what moves the robot, and neither is a tool
  // that only watches it (ignored_consumers).
  for (const auto & [topic, type] : topology.velocity_outputs) {
    const auto subscriptions = graph->get_subscriptions_info_by_topic(topic);
    const bool consumed = std::any_of(
      subscriptions.begin(), subscriptions.end(),
      [this, &topology, & type = type](const rclcpp::TopicEndpointInfo & info) {
        return info.topic_type() == type &&
               topology.easynav_nodes.count({info.node_name(), info.node_namespace()}) == 0 &&
               !is_ignored_consumer(info.node_name());
      });
    if (consumed) {
      report.consumed_topic = topic;
      report.consumed_type = type;
      break;
    }
  }

  return report;
}

std::string RosGraphEvaluator::ignored_topics_entry(const std::string & topic) const
{
  const std::string ns = get_node()->get_namespace();
  const std::string prefix = (ns == "/") ? "/" : ns + "/";
  if (topic.compare(0, prefix.size(), prefix) == 0) {
    return topic.substr(prefix.size());
  }
  return topic;
}

std::string RosGraphEvaluator::describe_unfed(const std::vector<Subscription> & unfed) const
{
  std::string msg = "no publisher for:";
  std::vector<std::string> entries;
  for (const auto & sub : unfed) {
    msg += "\n  " + sub.topic + " [" + short_type(sub.type) + "], needed by " + sub.node;
    entries.push_back("\"" + ignored_topics_entry(sub.topic) + "\"");
  }
  msg += "\n  if optional, add to " + get_plugin_name() + ".ignored_topics of " +
    get_node()->get_node_base_interface()->get_fully_qualified_name() + ": " + join(entries, ", ");
  return msg;
}

void RosGraphEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_plugin_name();
  status.hardware_id = "ros_graph";

  std::lock_guard<std::mutex> lock(topology_mutex_);

  if (!topology_.has_value()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "ROS graph topology not discovered yet (node not activated)";
    publish_diagnostic(nav_state, status);
    return;
  }

  const auto & topology = *topology_;
  const auto report = check_topology(topology);

  std::vector<std::string> problems;
  if (topology.velocity_outputs.empty()) {
    problems.push_back("no velocity output: system_node has no cmd_vel publisher");
  } else if (report.consumed_topic.empty()) {
    std::vector<std::string> outputs;
    for (const auto & [topic, type] : topology.velocity_outputs) {
      outputs.push_back(topic + " [" + short_type(type) + "]");
    }
    problems.push_back(
      "no subscriber for velocity output " + join(outputs, ", ") +
      " (driver/simulator down, wrong topic, or use_cmd_vel_stamped mismatch?)");
  }
  // /clock is subscribed by every node using simulated time: one clear cause, not N entries.
  std::vector<Subscription> unfed;
  std::vector<std::string> clock_subscribers;
  for (const auto & sub : report.unfed_subscriptions) {
    if (sub.type == kClockType) {
      clock_subscribers.push_back(sub.node);
    } else {
      unfed.push_back(sub);
    }
  }
  if (!clock_subscribers.empty()) {
    problems.push_back("use_sim_time is set but /clock is not published (simulator down?)");
  }
  if (!unfed.empty()) {
    problems.push_back(describe_unfed(unfed));
  }

  if (!report.consumed_topic.empty()) {
    const bool stamped = report.consumed_type == kTwistStampedType;
    status.values.push_back(key_value("velocity_topic", report.consumed_topic));
    status.values.push_back(key_value("velocity_type", stamped ? "stamped" : "unstamped"));
  }

  if (problems.empty()) {
    problem_since_.reset();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "velocity: " + report.consumed_topic + " [" +
      (report.consumed_type == kTwistStampedType ? "stamped" : "unstamped") + "]";
    publish_diagnostic(nav_state, status);
    return;
  }

  // Details only when something is wrong.
  status.values.push_back(
    key_value("unfed_subscriptions", std::to_string(report.unfed_subscriptions.size())));
  for (const auto & sub : report.unfed_subscriptions) {
    status.values.push_back(
      key_value("unfed", sub.topic + " [" + sub.type + "] <- " + sub.node));
    status.values.push_back(key_value("suggested_ignored_topic", ignored_topics_entry(sub.topic)));
  }
  if (!topology.missing_nodes.empty()) {
    status.values.push_back(key_value("missing_nodes", join(topology.missing_nodes, ", ")));
  }

  const auto now = steady_clock_.now();
  if (!problem_since_.has_value()) {
    problem_since_ = now;
  }
  const std::string problems_msg = join(problems, "\n");

  if ((now - activated_at_).seconds() < startup_grace_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "(startup grace) " + problems_msg;
  } else if ((now - *problem_since_).seconds() < error_debounce_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = problems_msg;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = problems_msg;
  }

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::RosGraphEvaluator,
  easynav_diagnostic_recovery::RecoveryEvaluatorBase)
