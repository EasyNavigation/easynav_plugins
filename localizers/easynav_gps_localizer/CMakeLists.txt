cmake_minimum_required(VERSION 3.20)
project(easynav_gps_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(${GeographicLib_INCLUDE_DIRS})


set(dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_core
  easynav_common
  pluginlib
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(gps_localizer SHARED
  src/easynav_gps_localizer/GpsLocalizer.cpp
)
target_include_directories(gps_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gps_localizer
  easynav_core::easynav_core
  easynav_common::easynav_common
  pluginlib::pluginlib
  ${geographic_msgs_TARGETS}
  GeographicLib
  ${GeographicLib_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  gps_localizer
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

ament_export_include_directories(include)
ament_export_libraries(
  gps_localizer
)
ament_export_targets(
  export_${PROJECT_NAME}
)
# Register the localization plugins
pluginlib_export_plugin_description_file(easynav_core easynav_gps_localizer_plugins.xml)
ament_export_dependencies(${dependencies})
ament_package()
