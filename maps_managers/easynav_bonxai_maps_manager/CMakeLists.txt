cmake_minimum_required(VERSION 3.20)
project(easynav_bonxai_maps_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaets REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)


add_library(bonxai SHARED
  src/bonxai/pcl_utils.cpp
  src/bonxai/probabilistic_map.cpp
)
target_include_directories(bonxai PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(bonxai PUBLIC
  tf2::tf2
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} SHARED
  src/easynav_bonxai_maps_manager/BonxaiMapsManager.cpp
  src/easynav_bonxai_maps_manager/map_io.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${yaml-cpp_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${GRAPHICSMAGICKCPP_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${bonxai_msgs_TARGETS}
  yaets::yaets
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
  yaml-cpp
  ${GRAPHICSMAGICKCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  bonxai
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  bonxai
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

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the plugin
pluginlib_export_plugin_description_file(easynav_core easynav_bonxai_maps_manager_plugins.xml)

# pluginlib_export_plugin_description_file(easynav_bonxai_filter easynav_bonxai_maps_manager_obstacle_filter.xml)
# pluginlib_export_plugin_description_file(easynav_bonxai_filter easynav_bonxai_maps_manager_inflation_filter.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  tf2
  tf2_ros
  pluginlib
  nav_msgs
  ament_index_cpp
  std_srvs
  yaets
  yaml_cpp_vendor
  pcl_conversions
  PCL
)
ament_package()
