cmake_minimum_required(VERSION 3.20)
project(easynav_fusion_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_localizer REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_srvs REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(${GeographicLib_INCLUDE_DIRS})


set(dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_core
  easynav_common
  easynav_localizer
  easynav_sensors
  pluginlib
  geographic_msgs
  robot_localization
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  eigen
)

add_library(fusion_localizer SHARED
  src/easynav_fusion_localizer/FusionLocalizer.cpp
  src/easynav_fusion_localizer/ukf_wrapper.cpp
)
target_include_directories(fusion_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${robot_localization_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)
target_link_libraries(fusion_localizer
  easynav_core::easynav_core
  easynav_common::easynav_common
  easynav_localizer::easynav_localizer
  easynav_sensors::easynav_sensors
  pluginlib::pluginlib
  ${geographic_msgs_TARGETS}
  GeographicLib
  ${GeographicLib_LIBRARIES}
  ${robot_localization_LIBRARIES}
  angles::angles
  yaets::yaets
  diagnostic_updater::diagnostic_updater
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  Eigen3::Eigen
)

target_compile_definitions(fusion_localizer PUBLIC EASYNAV_DEBUG_WITH_YAETS)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  fusion_localizer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
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
  fusion_localizer
)
ament_export_targets(
  export_${PROJECT_NAME}
)
# Register the localization plugins
pluginlib_export_plugin_description_file(easynav_core easynav_fusion_localizer_plugins.xml)
ament_export_dependencies(${dependencies})
ament_package()
