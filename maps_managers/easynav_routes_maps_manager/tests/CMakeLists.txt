cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_costmap_common REQUIRED)
find_package(easynav_routes_maps_manager REQUIRED)

ament_add_gtest(routes_mapsmanager_tests
  routes_mapsmanager_tests.cpp)
target_link_libraries(routes_mapsmanager_tests
  easynav_routes_maps_manager::easynav_routes_maps_manager
  easynav_common::easynav_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)

ament_add_gtest(routes_costmap_filter_tests
  routes_costmap_filter_tests.cpp)
target_link_libraries(routes_costmap_filter_tests
  easynav_routes_maps_manager::easynav_routes_maps_manager
  easynav_common::easynav_common
  easynav_costmap_common::easynav_costmap_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)

