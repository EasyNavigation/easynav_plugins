#include "easynav_routes_maps_manager/RoutesMapsManager.hpp"

#include <expected>

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

  reload_routes_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/reload_routes",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      try {
        load_routes_from_yaml();
        publish_routes_markers();
        response->success = true;
        response->message = "Routes reloaded";
      } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
      }
    });

  try {
    load_routes_from_yaml();
    publish_routes_markers();
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

  for (const auto & route_node : root["routes"]) {
    if (!route_node["start"] || !route_node["end"]) {
      continue;
    }

    RouteSegment segment;

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

}  // namespace easynav
