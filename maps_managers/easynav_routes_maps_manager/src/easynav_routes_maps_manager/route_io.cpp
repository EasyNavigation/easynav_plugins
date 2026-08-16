// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

#include "easynav_routes_maps_manager/route_io.hpp"

#include <fstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace easynav
{

namespace
{

RouteSegment default_segment()
{
  RouteSegment segment;
  segment.id = "route0";
  segment.start.position.x = 0.0;
  segment.start.position.y = 0.0;
  segment.start.position.z = 0.0;
  segment.start.orientation.x = 0.0;
  segment.start.orientation.y = 0.0;
  segment.start.orientation.z = 0.0;
  segment.start.orientation.w = 1.0;

  segment.end.position.x = 1.0;
  segment.end.position.y = 0.0;
  segment.end.position.z = 0.0;
  segment.end.orientation.x = 0.0;
  segment.end.orientation.y = 0.0;
  segment.end.orientation.z = 0.0;
  segment.end.orientation.w = 1.0;
  return segment;
}

}  // namespace

RoutesMap load_routes_from_yaml(const std::string & yaml_file)
{
  RoutesMap routes;

  if (yaml_file.empty()) {
    routes.push_back(default_segment());
    return routes;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_file);
  } catch (const std::exception &) {
    // File missing or invalid: fall back to a default single route.
    routes.push_back(default_segment());
    return routes;
  }

  if (!root["routes"]) {
    // No explicit routes list: use a default single route.
    routes.push_back(default_segment());
    return routes;
  }

  // routes: [route1, route2, ...]
  const auto & names_node = root["routes"];
  for (std::size_t i = 0; i < names_node.size(); ++i) {
    const auto name = names_node[i].as<std::string>();

    if (!root[name]) {
      continue;
    }

    const auto & route_node = root[name];
    if (!route_node["start"] || !route_node["end"]) {
      continue;
    }

    RouteSegment segment;
    segment.id = name;

    const auto & start = route_node["start"];
    const auto & end = route_node["end"];

    segment.start.position.x = start["x"].as<double>();
    segment.start.position.y = start["y"].as<double>();
    segment.start.position.z = start["z"].as<double>(0.0);

    segment.start.orientation.x = start["qx"].as<double>(0.0);
    segment.start.orientation.y = start["qy"].as<double>(0.0);
    segment.start.orientation.z = start["qz"].as<double>(0.0);
    segment.start.orientation.w = start["qw"].as<double>(1.0);

    segment.end.position.x = end["x"].as<double>();
    segment.end.position.y = end["y"].as<double>();
    segment.end.position.z = end["z"].as<double>(0.0);

    segment.end.orientation.x = end["qx"].as<double>(0.0);
    segment.end.orientation.y = end["qy"].as<double>(0.0);
    segment.end.orientation.z = end["qz"].as<double>(0.0);
    segment.end.orientation.w = end["qw"].as<double>(1.0);

    routes.push_back(segment);
  }

  return routes;
}

easynav_routes_maps_manager::msg::RoutesMap to_msg(const RoutesMap & routes)
{
  easynav_routes_maps_manager::msg::RoutesMap msg;
  msg.routes.reserve(routes.size());
  for (const auto & seg : routes) {
    easynav_routes_maps_manager::msg::RouteSegment seg_msg;
    seg_msg.id = seg.id;
    seg_msg.start = seg.start;
    seg_msg.end = seg.end;
    msg.routes.push_back(seg_msg);
  }
  return msg;
}

RoutesMap from_msg(const easynav_routes_maps_manager::msg::RoutesMap & msg)
{
  RoutesMap routes;
  routes.reserve(msg.routes.size());
  for (const auto & seg_msg : msg.routes) {
    RouteSegment seg;
    seg.id = seg_msg.id;
    seg.start = seg_msg.start;
    seg.end = seg_msg.end;
    routes.push_back(seg);
  }
  return routes;
}

bool save_routes_to_yaml(
  const std::string & yaml_file, const RoutesMap & routes, std::string & error_message)
{
  // routes: [route1, route2]
  // route1: { start: ..., end: ... }
  YAML::Emitter out;
  out << YAML::BeginMap;

  // Collect route names from ids (or generate generic ones).
  std::vector<std::string> names;
  names.reserve(routes.size());
  for (std::size_t i = 0; i < routes.size(); ++i) {
    const auto & seg = routes[i];
    if (!seg.id.empty()) {
      names.push_back(seg.id);
    } else {
      names.push_back("route" + std::to_string(i));
    }
  }

  out << YAML::Key << "routes" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const auto & n : names) {
    out << n;
  }
  out << YAML::EndSeq;

  // Now define each route as a separate key in the map.
  for (std::size_t i = 0; i < routes.size(); ++i) {
    const auto & seg = routes[i];
    const auto & name = names[i];

    out << YAML::Key << name << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "start" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << seg.start.position.x;
    out << YAML::Key << "y" << YAML::Value << seg.start.position.y;
    out << YAML::Key << "z" << YAML::Value << seg.start.position.z;
    out << YAML::Key << "qx" << YAML::Value << seg.start.orientation.x;
    out << YAML::Key << "qy" << YAML::Value << seg.start.orientation.y;
    out << YAML::Key << "qz" << YAML::Value << seg.start.orientation.z;
    out << YAML::Key << "qw" << YAML::Value << seg.start.orientation.w;
    out << YAML::EndMap;

    out << YAML::Key << "end" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << seg.end.position.x;
    out << YAML::Key << "y" << YAML::Value << seg.end.position.y;
    out << YAML::Key << "z" << YAML::Value << seg.end.position.z;
    out << YAML::Key << "qx" << YAML::Value << seg.end.orientation.x;
    out << YAML::Key << "qy" << YAML::Value << seg.end.orientation.y;
    out << YAML::Key << "qz" << YAML::Value << seg.end.orientation.z;
    out << YAML::Key << "qw" << YAML::Value << seg.end.orientation.w;
    out << YAML::EndMap;

    out << YAML::EndMap;
  }

  out << YAML::EndMap;

  if (!out.good()) {
    error_message = "Failed to serialize routes to YAML: " + out.GetLastError();
    return false;
  }

  std::ofstream file(yaml_file);
  if (!file.is_open()) {
    error_message = "Could not open file for writing: " + yaml_file;
    return false;
  }
  file << out.c_str();
  file.close();
  if (file.fail()) {
    error_message = "Failed writing routes to file: " + yaml_file;
    return false;
  }

  return true;
}

}  // namespace easynav
