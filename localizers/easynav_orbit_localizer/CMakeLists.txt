cmake_minimum_required(VERSION 3.20)
project(easynav_orbit_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_bonxai_maps_manager REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_costmap_common REQUIRED)
find_package(easynav_localizer REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navmap_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# The estimation core: no ROS, reusable from tools and tests
add_library(${PROJECT_NAME}_core SHARED
  src/easynav_orbit_localizer/DistanceField.cpp
  src/easynav_orbit_localizer/Matcher.cpp
  src/easynav_orbit_localizer/SymmetryGraph.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  easynav_costmap_common::easynav_costmap_common
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} SHARED
  src/easynav_orbit_localizer/OrbitLocalizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${GeographicLib_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_core
  easynav_bonxai_maps_manager::bonxai
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_localizer::easynav_localizer
  easynav_sensors::easynav_sensors
  navmap_core::navmap_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${GeographicLib_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  endif()
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(easynav_core easynav_orbit_localizer_plugins.xml)
ament_export_dependencies(
  easynav_bonxai_maps_manager easynav_common easynav_core easynav_costmap_common
  easynav_localizer easynav_sensors Eigen3 geometry_msgs nav_msgs navmap_core PCL pluginlib
  rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros GeographicLib
)
ament_package()
