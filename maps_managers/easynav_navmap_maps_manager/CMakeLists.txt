cmake_minimum_required(VERSION 3.20)
project(easynav_navmap_maps_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaets REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/easynav_navmap_maps_manager/NavMapMapsManager.cpp
  src/easynav_navmap_maps_manager/map_io.cpp
  src/easynav_navmap_maps_manager/filters/NavMapFilter.cpp
  src/easynav_navmap_maps_manager/filters/ObstacleFilter.cpp
  src/easynav_navmap_maps_manager/filters/InflationFilter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${yaml-cpp_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${GRAPHICSMAGICKCPP_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ament_index_cpp::ament_index_cpp
  navmap_core::navmap_core
  navmap_ros::navmap_ros
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${navmap_ros_interfaces_TARGETS}
  yaets::yaets
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
  yaml-cpp
  ${GRAPHICSMAGICKCPP_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the plugin
pluginlib_export_plugin_description_file(easynav_core easynav_navmap_maps_manager_plugins.xml)

pluginlib_export_plugin_description_file(easynav_navmap_maps_manager easynav_navmap_maps_manager_obstacle_filter.xml)
pluginlib_export_plugin_description_file(easynav_navmap_maps_manager easynav_navmap_maps_manager_inflation_filter.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  tf2
  tf2_ros
  pluginlib
  nav_msgs
  ament_index_cpp
  navmap_core
  navmap_ros
  navmap_ros_interfaces
  std_srvs
  yaets
  yaml_cpp_vendor
)
ament_package()
