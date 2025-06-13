cmake_minimum_required(VERSION 3.20)
project(easynav_gps_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geographic_msgs REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(${GeographicLib_INCLUDE_DIRS})


set(dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_core
  pluginlib
  geographic_msgs
)

add_library(gps_localizer SHARED
  src/easynav_gps_localizer/GpsLocalizer.cpp
)
target_include_directories(gps_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gps_localizer
  easynav_core
  pluginlib
  geographic_msgs
)

target_link_libraries(gps_localizer
  GeographicLib
  ${GeographicLib_LIBRARIES}
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
