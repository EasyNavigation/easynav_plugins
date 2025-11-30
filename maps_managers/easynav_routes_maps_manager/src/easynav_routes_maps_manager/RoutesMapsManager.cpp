#include "easynav_routes_maps_manager/RoutesMapsManager.hpp"

#include <expected>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

namespace easynav
{

RoutesMapsManager::RoutesMapsManager()
{
  NavState::register_printer<RoutesMap>(
      [](const RoutesMap & routes) {
        std::ostringstream out;
        out << "RoutesMap with " << routes.size() << " segments";
        for (std::size_t i = 0; i < routes.size(); ++i) {
          const auto & s = routes[i];
          out << "\n  [" << i << "] from (" << s.start.position.x << ", "
              << s.start.position.y << ") to (" << s.end.position.x << ", "
              << s.end.position.y << ")";
        }
        return out.str();
      });
}

RoutesMapsManager::~RoutesMapsManager() = default;

std::expected<void, std::string> RoutesMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file;
  if (!node->has_parameter(plugin_name + ".package")) {
    node->declare_parameter(plugin_name + ".package", package_name);
  }
  if (!node->has_parameter(plugin_name + ".map_path_file")) {
    node->declare_parameter(plugin_name + ".map_path_file", map_path_file);
  }

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);

  map_path_.clear();
  if (!map_path_file.empty() && map_path_file[0] == '/') {
    // Absolute path: ignore package_name.
    map_path_ = map_path_file;
  } else if (!package_name.empty() && !map_path_file.empty()) {
    const auto pkgpath = ament_index_cpp::get_package_share_directory(package_name);
    map_path_ = pkgpath + "/" + map_path_file;
  } else {
    return std::unexpected(
      "Parameters '" + plugin_name + ".package' and '" + plugin_name +
      ".map_path_file' are not correctly set");
  }

  routes_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/routes",
    rclcpp::SystemDefaultsQoS());

  imarker_pub_ = node->create_publisher<visualization_msgs::msg::InteractiveMarker>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/routes_imarkers",
    rclcpp::SystemDefaultsQoS());

  imarker_feedback_sub_ = node->create_subscription<
    visualization_msgs::msg::InteractiveMarkerFeedback>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/routes_imarkers/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
      handle_interactive_feedback(feedback);
    });

  save_routes_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/save_routes",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      try {
        // Persist current routes_ back to YAML file using the
        // structure:
        // routes: [route1, route2]
        // route1: { start: ..., end: ... }
        YAML::Emitter out;
        out << YAML::BeginMap;

        // Collect route names from ids (or generate generic ones).
        std::vector<std::string> names;
        names.reserve(routes_.size());
        for (std::size_t i = 0; i < routes_.size(); ++i) {
          const auto & seg = routes_[i];
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
        for (std::size_t i = 0; i < routes_.size(); ++i) {
          const auto & seg = routes_[i];
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

        std::ofstream file(map_path_);
        file << out.c_str();
        file.close();

        response->success = true;
        response->message = "Routes saved";
      } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
      }
    });

  try {
    load_routes_from_yaml();
    publish_routes_markers();
    publish_interactive_markers();
  } catch (const std::exception & e) {
    return std::unexpected(std::string{"Failed to load routes: "} + e.what());
  }

  return {};
}

void RoutesMapsManager::update(NavState & nav_state)
{
  // Expose routes map through NavState for other modules.
  nav_state.set("routes", routes_);
}

void RoutesMapsManager::load_routes_from_yaml()
{
  routes_.clear();

  if (map_path_.empty()) {
    throw std::runtime_error{"Parameter 'routes_map_path' is empty"};
  }

  YAML::Node root = YAML::LoadFile(map_path_);

  if (!root["routes"]) {
    throw std::runtime_error{"YAML file must contain a 'routes' sequence"};
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

    routes_.push_back(segment);
  }
}

void RoutesMapsManager::publish_routes_markers()
{
  if (!routes_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  int id = 0;
  for (const auto & seg : routes_) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.ns = "routes";
    line.id = id++;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.scale.x = 0.05;  // line width

    line.color.r = 0.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;

    line.points.resize(2);
    line.points[0].x = seg.start.position.x;
    line.points[0].y = seg.start.position.y;
    line.points[0].z = seg.start.position.z;

    line.points[1].x = seg.end.position.x;
    line.points[1].y = seg.end.position.y;
    line.points[1].z = seg.end.position.z;

    array.markers.push_back(line);
  }

  routes_pub_->publish(array);
}

void RoutesMapsManager::publish_interactive_markers()
{
  if (!imarker_pub_) {
    return;
  }

  int id = 0;
  for (const auto & seg : routes_) {
    // Start marker
    visualization_msgs::msg::InteractiveMarker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.name = seg.id + "_start";
    start_marker.description = "Route " + seg.id + " start";
    start_marker.pose = seg.start;

    visualization_msgs::msg::InteractiveMarker end_marker;
    end_marker.header.frame_id = "map";
    end_marker.name = seg.id + "_end";
    end_marker.description = "Route " + seg.id + " end";
    end_marker.pose = seg.end;

    imarker_pub_->publish(start_marker);
    imarker_pub_->publish(end_marker);

    (void)id;
  }
}

void RoutesMapsManager::handle_interactive_feedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (!feedback) {
    return;
  }

  const auto & name = feedback->marker_name;

  for (auto & seg : routes_) {
    if (name == seg.id + "_start") {
      seg.start = feedback->pose;
      publish_routes_markers();
      return;
    } else if (name == seg.id + "_end") {
      seg.end = feedback->pose;
      publish_routes_markers();
      return;
    }
  }
}

}  // namespace easynav
