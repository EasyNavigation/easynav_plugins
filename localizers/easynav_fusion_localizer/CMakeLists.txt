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
find_package(pluginlib REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_srvs REQUIRED)
find_package(angles REQUIRED)

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
  pluginlib
  geographic_msgs
  robot_localization
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
  pluginlib::pluginlib
  ${geographic_msgs_TARGETS}
  GeographicLib
  ${GeographicLib_LIBRARIES}
  ${robot_localization_LIBRARIES}
  angles::angles
  yaets::yaets
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
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
