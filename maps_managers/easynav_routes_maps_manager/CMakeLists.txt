cmake_minimum_required(VERSION 3.20)
project(easynav_routes_maps_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_costmap_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaets REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(interactive_markers REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/easynav_routes_maps_manager/RoutesMapsManager.cpp
  src/easynav_routes_maps_manager/filters/RoutesCostmapFilter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_costmap_common::easynav_costmap_common
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ament_index_cpp::ament_index_cpp
  yaets::yaets
  yaml-cpp
  interactive_markers::interactive_markers
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
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

  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the plugins
pluginlib_export_plugin_description_file(easynav_core easynav_routes_maps_manager_plugins.xml)
pluginlib_export_plugin_description_file(easynav_routes_maps_manager easynav_routes_filters_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  tf2_ros
  pluginlib
  easynav_costmap_common
  nav_msgs
  ament_index_cpp
  std_srvs
  geometry_msgs
  visualization_msgs
  interactive_markers
  yaets
  yaml-cpp
)
ament_package()
