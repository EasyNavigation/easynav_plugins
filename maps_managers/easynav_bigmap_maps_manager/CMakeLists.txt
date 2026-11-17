cmake_minimum_required(VERSION 3.8)
project(easynav_bigmap_maps_manager)

# Without a build type colcon compiles with no optimisation, and simplifying a large map takes
# minutes instead of seconds. Release unless told otherwise.
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
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(easynav_navmap_maps_manager REQUIRED)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# The level-of-detail engine: ROS-free
add_library(${PROJECT_NAME}_lod SHARED
  src/tiled_lod.cpp
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
)
# Headers of the NavMap filters only; do NOT link that library (see src/navmap_filter_base.cpp)
get_target_property(NAVMAP_FILTERS_INCLUDES
  easynav_navmap_maps_manager::easynav_navmap_maps_manager INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${NAVMAP_FILTERS_INCLUDES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_lod
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_sensors::easynav_sensors
  navmap_ros::navmap_ros
  ${navmap_ros_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(easynav_core easynav_bigmap_maps_manager_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}_lod ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Humble's ament_copyright does not recognize the license with the EasyNav line in the header
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  endif()
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_tiled_lod test/test_tiled_lod.cpp)
  target_link_libraries(test_tiled_lod ${PROJECT_NAME}_lod)
  ament_add_gtest(test_bigmap_plugin test/test_bigmap_plugin.cpp TIMEOUT 300)
  target_link_libraries(test_bigmap_plugin ${PROJECT_NAME})
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle pluginlib nav_msgs std_msgs visualization_msgs
  easynav_common easynav_core easynav_sensors easynav_navmap_maps_manager
  navmap_core navmap_ros navmap_ros_interfaces Eigen3)
ament_package()
