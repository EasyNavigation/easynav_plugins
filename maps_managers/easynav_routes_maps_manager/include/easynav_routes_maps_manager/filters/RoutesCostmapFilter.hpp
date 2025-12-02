#ifndef EASYNAV_ROUTES_MAPS_MANAGER_FILTERS_ROUTES_COSTMAP_FILTER_HPP_
#define EASYNAV_ROUTES_MAPS_MANAGER_FILTERS_ROUTES_COSTMAP_FILTER_HPP_

#include <expected>
#include <string>

#include "easynav_routes_maps_manager/RoutesFilter.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace easynav
{

class RoutesCostmapFilter : public RoutesFilter
{
public:
  RoutesCostmapFilter();

  std::expected<void, std::string> initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_ns,
    const std::string & tf_prefix) override;

  void update(NavState & nav_state) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_ns_;
  std::string tf_prefix_;

  int min_cost_{50};
  double route_width_{0.0};  // meters; 0.0 means default = one cell width

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr routes_occ_pub_;
  nav_msgs::msg::OccupancyGrid routes_grid_msg_;
};

}  // namespace easynav

#endif  // EASYNAV_ROUTES_MAPS_MANAGER_FILTERS_ROUTES_COSTMAP_FILTER_HPP_
