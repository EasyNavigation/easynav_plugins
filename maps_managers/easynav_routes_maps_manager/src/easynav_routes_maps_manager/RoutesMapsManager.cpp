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
    rclcpp::QoS(10).transient_local().reliable());

  imarker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    plugin_name + std::string("_imarkers"), node, false);

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

  // Initialize next_route_id_ so that newly created routes get
  // unique IDs that don't clash with existing ones.
  next_route_id_ = 0;
  for (const auto & seg : routes_) {
    if (seg.id.rfind("route", 0) == 0 && seg.id.size() > 5) {
      try {
        const int n = std::stoi(seg.id.substr(5));
        if (n >= next_route_id_) {
          next_route_id_ = n + 1;
        }
      } catch (...) {
        // Non-numeric suffixes are ignored.
      }
    }
  }
}

void RoutesMapsManager::publish_routes_markers()
{
  if (!routes_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

   // First, delete all previous markers in our namespaces so that
   // removed segments do not leave orphaned markers behind.
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.action = visualization_msgs::msg::Marker::DELETEALL;

    m.ns = "routes_line";
    array.markers.push_back(m);

    m.ns = "routes_arrow";
    array.markers.push_back(m);
  }

  int id = 0;
  for (const auto & seg : routes_) {
    // Line between start and end
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.ns = "routes_line";
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

    // Arrow for start orientation (same style as end)
    visualization_msgs::msg::Marker start_arrow;
    start_arrow.header.frame_id = "map";
    start_arrow.ns = "routes_arrow";
    start_arrow.id = id++;
    start_arrow.type = visualization_msgs::msg::Marker::ARROW;
    start_arrow.action = visualization_msgs::msg::Marker::ADD;
    start_arrow.pose = seg.start;
    start_arrow.scale.x = 0.25;   // shaft length
    start_arrow.scale.y = 0.05;   // shaft diameter
    start_arrow.scale.z = 0.1;   // head diameter
    start_arrow.color.r = 1.0f;
    start_arrow.color.g = 1.0f;
    start_arrow.color.b = 0.0f;
    start_arrow.color.a = 0.9f;
    array.markers.push_back(start_arrow);

    // Arrow for end orientation (same style)
    visualization_msgs::msg::Marker end_arrow;
    end_arrow.header.frame_id = "map";
    end_arrow.ns = "routes_arrow";
    end_arrow.id = id++;
    end_arrow.type = visualization_msgs::msg::Marker::ARROW;
    end_arrow.action = visualization_msgs::msg::Marker::ADD;
    end_arrow.pose = seg.end;
    end_arrow.scale.x = 0.25;
    end_arrow.scale.y = 0.05;
    end_arrow.scale.z = 0.1;
    end_arrow.color.r = 1.0f;
    end_arrow.color.g = 1.0f;
    end_arrow.color.b = 0.0f;
    end_arrow.color.a = 0.9f;
    array.markers.push_back(end_arrow);
  }

  routes_pub_->publish(array);
}

void RoutesMapsManager::publish_interactive_markers()
{
  if (!imarker_server_) {
    return;
  }
  imarker_server_->clear();

  for (const auto & seg : routes_) {
    // Per-segment toggle cube (red in normal mode, green in edit mode).
    visualization_msgs::msg::InteractiveMarker mode_marker;
    mode_marker.header.frame_id = "map";
    mode_marker.name = seg.id + "_mode";
    mode_marker.scale = 1.0;

    // Mid-point between start and end (no vertical offset)
    mode_marker.pose.position.x = 0.5 * (seg.start.position.x + seg.end.position.x);
    mode_marker.pose.position.y = 0.5 * (seg.start.position.y + seg.end.position.y);
    mode_marker.pose.position.z = 0.5 * (seg.start.position.z + seg.end.position.z);

    visualization_msgs::msg::InteractiveMarkerControl mode_ctrl;
    mode_ctrl.name = "toggle_edit";
    mode_ctrl.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    mode_ctrl.always_visible = true;

    visualization_msgs::msg::Marker cube;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.scale.x = 0.15;  // even smaller cube
    cube.scale.y = 0.15;
    cube.scale.z = 0.15;
    if (!seg.edit_mode) {
      // Red in normal mode
      cube.color.r = 1.0f;
      cube.color.g = 0.0f;
      cube.color.b = 0.0f;
    } else {
      // Green in edit mode
      cube.color.r = 0.0f;
      cube.color.g = 1.0f;
      cube.color.b = 0.0f;
    }
    cube.color.a = 0.9f;

    // Text label for the toggle control
    visualization_msgs::msg::Marker toggle_text;
    toggle_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    toggle_text.text = "toggle edit";
    toggle_text.scale.z = 0.1;  // font height
    toggle_text.color.r = 1.0f;
    toggle_text.color.g = 1.0f;
    toggle_text.color.b = 1.0f;
    toggle_text.color.a = 1.0f;
    toggle_text.pose.position.z = 0.3;  // slightly above the cube

    mode_ctrl.markers.push_back(cube);
    mode_ctrl.markers.push_back(toggle_text);

    mode_marker.controls.push_back(mode_ctrl);

    imarker_server_->insert(
      mode_marker,
      std::bind(
        &RoutesMapsManager::handle_interactive_feedback,
        this,
        std::placeholders::_1));

    if (!seg.edit_mode) {
      // In normal mode we only show the cube and skip endpoint controls.
      continue;
    }

    visualization_msgs::msg::InteractiveMarker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.name = seg.id + "_start";
    start_marker.description = "Route " + seg.id + " start";
    start_marker.pose = seg.start;
    start_marker.scale = 1.0;

    visualization_msgs::msg::InteractiveMarker end_marker;
    end_marker.header.frame_id = "map";
    end_marker.name = seg.id + "_end";
    end_marker.description = "Route " + seg.id + " end";
    end_marker.pose = seg.end;
    end_marker.scale = 1.0;

    auto add_controls = [](visualization_msgs::msg::InteractiveMarker & marker) {
      visualization_msgs::msg::InteractiveMarkerControl control;

      // Move along X
      control.orientation.w = 1.0;
      control.orientation.x = 1.0;
      control.orientation.y = 0.0;
      control.orientation.z = 0.0;
      control.name = "move_x";
      control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      marker.controls.push_back(control);

      // Move along Y
      control.orientation.x = 0.0;
      control.orientation.y = 1.0;
      control.name = "move_y";
      marker.controls.push_back(control);

      // Move along Z
      control.orientation.y = 0.0;
      control.orientation.z = 1.0;
      control.name = "move_z";
      marker.controls.push_back(control);

      // Rotate around Z (yaw), orientation as in interactive_markers examples
      control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      control.orientation.w = 1.0;
      control.orientation.x = 0.0;
      control.orientation.y = 1.0;
      control.orientation.z = 0.0;
      control.name = "rotate_z";
      marker.controls.push_back(control);

      // Button control to add a new segment starting from this endpoint
      visualization_msgs::msg::InteractiveMarkerControl add_ctrl;
      add_ctrl.name = "add_segment";
      add_ctrl.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
      add_ctrl.always_visible = true;

      visualization_msgs::msg::Marker add_marker;
      add_marker.type = visualization_msgs::msg::Marker::SPHERE;
      // Visual sphere for the add control
      add_marker.scale.x = 0.2;
      add_marker.scale.y = 0.2;
      add_marker.scale.z = 0.2;
      add_marker.color.r = 1.0f;
      add_marker.color.g = 0.5f;
      add_marker.color.b = 0.0f;
      add_marker.color.a = 0.9f;

      // Text label for the add control
      visualization_msgs::msg::Marker add_text;
      add_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      add_text.text = "add";
      add_text.scale.z = 0.1;  // font height (half)
      add_text.color.r = 1.0f;
      add_text.color.g = 1.0f;
      add_text.color.b = 1.0f;
      add_text.color.a = 1.0f;
      add_text.pose.position.z = 0.4;  // slightly above the sphere

      add_ctrl.markers.push_back(add_marker);
      add_ctrl.markers.push_back(add_text);

      marker.controls.push_back(add_ctrl);

      // Button control to remove the segment this endpoint belongs to
      visualization_msgs::msg::InteractiveMarkerControl remove_ctrl;
      remove_ctrl.name = "remove_segment";
      remove_ctrl.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
      remove_ctrl.always_visible = true;

      visualization_msgs::msg::Marker remove_marker;
      remove_marker.type = visualization_msgs::msg::Marker::SPHERE;
      // Place the red sphere 1 m above the endpoint so that it
      // does not overlap with the orange "add" sphere.
      remove_marker.pose.position.z = 1.0;
      remove_marker.scale.x = 0.15;
      remove_marker.scale.y = 0.15;
      remove_marker.scale.z = 0.15;
      remove_marker.color.r = 1.0f;
      remove_marker.color.g = 0.0f;
      remove_marker.color.b = 0.0f;
      remove_marker.color.a = 0.9f;

      // Text label for the remove control
      visualization_msgs::msg::Marker remove_text;
      remove_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      remove_text.text = "remove";
      remove_text.scale.z = 0.1;  // font height (half)
      remove_text.color.r = 1.0f;
      remove_text.color.g = 1.0f;
      remove_text.color.b = 1.0f;
      remove_text.color.a = 1.0f;
      remove_text.pose.position.z = 1.3;  // slightly above the red sphere

      remove_ctrl.markers.push_back(remove_marker);
      remove_ctrl.markers.push_back(remove_text);

      marker.controls.push_back(remove_ctrl);
    };

    add_controls(start_marker);
    add_controls(end_marker);

    imarker_server_->insert(
      start_marker,
      std::bind(
        &RoutesMapsManager::handle_interactive_feedback,
        this,
        std::placeholders::_1));
    imarker_server_->insert(
      end_marker,
      std::bind(
        &RoutesMapsManager::handle_interactive_feedback,
        this,
        std::placeholders::_1));
  }

  imarker_server_->applyChanges();
}

void RoutesMapsManager::handle_interactive_feedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (!feedback) {
    return;
  }

  const auto & name = feedback->marker_name;

  // Toggle per-segment edit mode when clicking the central cube.
  if (feedback->control_name == "toggle_edit" &&
      (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK ||
       feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
  {
    // name is <id>_mode
    const auto underscore_pos = name.rfind("_");
    if (underscore_pos != std::string::npos) {
      const auto base_id = name.substr(0, underscore_pos);
      for (auto & seg : routes_) {
        if (seg.id == base_id) {
          seg.edit_mode = !seg.edit_mode;
          break;
        }
      }
      publish_interactive_markers();
    }
    return;
  }

  // Creation of a new segment from this endpoint
  if (feedback->control_name == "add_segment" &&
      (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK ||
       feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
  {
    // Compute forward direction from marker orientation (assume x-forward).
    const auto & p = feedback->pose.position;
    const auto & q = feedback->pose.orientation;

    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;

    // Forward vector in world coordinates: q * (1,0,0) * q^{-1}
    const double fx = 2.0 * (qx * qx + qw * qw) - 1.0;
    const double fy = 2.0 * (qx * qy + qw * qz);
    const double fz = 2.0 * (qx * qz - qw * qy);

    const double length = 2.0;  // meters

    RouteSegment new_seg;
    // New unique segment id based on a monotonic counter
    new_seg.id = "route" + std::to_string(next_route_id_++);

    new_seg.start = feedback->pose;
    new_seg.end = feedback->pose;
    new_seg.end.position.x = p.x + fx * length;
    new_seg.end.position.y = p.y + fy * length;
    new_seg.end.position.z = p.z + fz * length;

    routes_.push_back(new_seg);

    publish_routes_markers();
    publish_interactive_markers();
    return;
  }

  // Removal of the segment this endpoint belongs to
  if (feedback->control_name == "remove_segment" &&
      (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK ||
       feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
  {
    // marker_name is either <id>_start or <id>_end
    const auto underscore_pos = name.rfind("_");
    if (underscore_pos != std::string::npos) {
      const auto base_id = name.substr(0, underscore_pos);
      for (auto it = routes_.begin(); it != routes_.end(); ++it) {
        if (it->id == base_id) {
          routes_.erase(it);
          break;
        }
      }
      publish_routes_markers();
      publish_interactive_markers();
    }
    return;
  }

  for (auto & seg : routes_) {
    if (name == seg.id + "_start") {
      seg.start = feedback->pose;
      publish_routes_markers();
      publish_interactive_markers();
      return;
    } else if (name == seg.id + "_end") {
      seg.end = feedback->pose;
      publish_routes_markers();
      publish_interactive_markers();
      return;
    }
  }
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::RoutesMapsManager, easynav::MapsManagerBase)
