cmake_minimum_required(VERSION 3.8)
project(easynav_bigmap_maps_manager)

# Without a build type colcon compiles with no optimisation: the NavMap geometry rebuild of every
# re-mesh took 470 ms instead of 7 ms (measured on the campus map). Release unless told otherwise.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(navmap_core REQUIRED)
find_package(easynav_navmap_maps_manager REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Pure C++ LOD engine (RTIN error map + distance-driven extraction); ROS-free, unit-testable
add_library(${PROJECT_NAME}_lod SHARED
  src/dem_grid.cpp
  src/rtin_error_map.cpp
  src/lod_policy.cpp
  src/narrow_passages.cpp
  src/lod_mesher.cpp
  src/known_map.cpp
  src/navmap_dem.cpp
  src/surface_lod.cpp
  src/dynamic_layer.cpp
  src/obstacle_memory.cpp
  src/pipeline_check.cpp
)
target_include_directories(${PROJECT_NAME}_lod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_lod PUBLIC
  navmap_core::navmap_core
  Eigen3::Eigen
)

# The EasyNav plugin (loaded by maps_manager_node through pluginlib)
add_library(${PROJECT_NAME} SHARED
  src/BigMapMapsManager.cpp
  src/navmap_filter_base.cpp
  src/marker_publisher.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Headers of the NavMap filters only; do NOT link that library (see src/navmap_filter_base.cpp)
get_target_property(NAVMAP_FILTERS_INCLUDES
  easynav_navmap_maps_manager::easynav_navmap_maps_manager INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${NAVMAP_FILTERS_INCLUDES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_lod
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_sensors::easynav_sensors
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  navmap_ros::navmap_ros
  ${navmap_ros_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lod ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lod_mesher test/test_lod_mesher.cpp)
  target_link_libraries(test_lod_mesher ${PROJECT_NAME}_lod)
  ament_add_gtest(test_lod_closure test/test_lod_closure.cpp TIMEOUT 600)
  target_link_libraries(test_lod_closure ${PROJECT_NAME}_lod)
  # the DEM file: also written by navmap_tools' navmap_gis_tool (a golden file keeps both honest)
  ament_add_gtest(test_dem_grid test/test_dem_grid.cpp)
  target_link_libraries(test_dem_grid ${PROJECT_NAME}_lod)
  target_compile_definitions(test_dem_grid PRIVATE
    BIGMAP_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  ament_add_gtest(test_main_piece test/test_main_piece.cpp)
  target_link_libraries(test_main_piece ${PROJECT_NAME}_lod)
  ament_add_gtest(test_navmap_dem test/test_navmap_dem.cpp)
  target_link_libraries(test_navmap_dem ${PROJECT_NAME}_lod)
  ament_add_gtest(test_update_watchdog test/test_update_watchdog.cpp)
  target_include_directories(test_update_watchdog PRIVATE include)
  ament_add_gtest(test_pipeline_check test/test_pipeline_check.cpp)
  target_link_libraries(test_pipeline_check ${PROJECT_NAME}_lod)
  ament_add_gtest(test_obstacle_memory test/test_obstacle_memory.cpp)
  target_link_libraries(test_obstacle_memory ${PROJECT_NAME}_lod)
  ament_add_gtest(test_surface_lod test/test_surface_lod.cpp)
  target_link_libraries(test_surface_lod ${PROJECT_NAME}_lod)
  # the plugin itself: parameters, initialize(), update() with a pose and sensor points
  ament_add_gtest(test_bigmap_plugin test/test_bigmap_plugin.cpp TIMEOUT 120)
  target_link_libraries(test_bigmap_plugin ${PROJECT_NAME})
  ament_add_gtest(test_garage_lod test/test_garage_lod.cpp)
  target_link_libraries(test_garage_lod ${PROJECT_NAME}_lod navmap_ros::navmap_ros)
  # the garage map is bigmap_bringup's: next to this package in the BigMap repo, or there from
  # easynav_plugins; the test also reads BIGMAP_GARAGE_NAVMAP from the environment
  set(BIGMAP_GARAGE_NAVMAP_FILE "")
  foreach(candidate
      "${CMAKE_CURRENT_SOURCE_DIR}/../bigmap_bringup/maps/parking_garage.navmap"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../BigMap/bigmap_bringup/maps/parking_garage.navmap")
    if(EXISTS "${candidate}" AND NOT BIGMAP_GARAGE_NAVMAP_FILE)
      set(BIGMAP_GARAGE_NAVMAP_FILE "${candidate}")
    endif()
  endforeach()
  target_compile_definitions(test_garage_lod PRIVATE
    BIGMAP_GARAGE_NAVMAP="${BIGMAP_GARAGE_NAVMAP_FILE}")
  ament_add_gtest(test_real_maps test/test_real_maps.cpp)
  target_link_libraries(test_real_maps ${PROJECT_NAME}_lod navmap_ros::navmap_ros)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME}_lod ${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
# every package whose targets the exported library links (a consumer's find_package() fails on
# "target not found" otherwise)
ament_export_dependencies(easynav_common easynav_core easynav_sensors easynav_navmap_maps_manager pluginlib rclcpp rclcpp_lifecycle
  navmap_core navmap_ros navmap_ros_interfaces std_msgs geometry_msgs nav_msgs std_srvs
  visualization_msgs tf2 Eigen3)

# Register the plugin
pluginlib_export_plugin_description_file(easynav_core easynav_bigmap_maps_manager_plugins.xml)

ament_package()
