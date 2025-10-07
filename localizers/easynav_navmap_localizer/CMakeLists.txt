cmake_minimum_required(VERSION 3.20)
project(easynav_navmap_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)
find_package(easynav_localizer REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_bonxai_maps_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/easynav_navmap_localizer/AMCLLocalizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  navmap_core::navmap_core
  navmap_ros::navmap_ros
  easynav_bonxai_maps_manager::easynav_bonxai_maps_manager
  ${navmap_ros_interfaces_TARGETS}
  easynav_localizer::easynav_localizer
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ${tf2_geometry_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
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
  # add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the planning plugins
pluginlib_export_plugin_description_file(easynav_core easynav_navmap_localizer_plugins.xml)

ament_export_dependencies(
  easynav_common
  navmap_core
  navmap_ros
  navmap_ros_interfaces
  easynav_localizer
  easynav_core
  easynav_bonxai_maps_manager
  tf2
  tf2_ros
  pluginlib
  geometry_msgs
  std_srvs
  nav_msgs
)
ament_package()
