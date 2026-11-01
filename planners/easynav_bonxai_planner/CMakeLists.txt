cmake_minimum_required(VERSION 3.20)
project(easynav_bonxai_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Unoptimized, A* over millions of cells takes seconds
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_bonxai_maps_manager REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} SHARED
  src/easynav_bonxai_planner/BonxaiPlanner.cpp
  src/easynav_bonxai_planner/Planner3D.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_bonxai_maps_manager::easynav_bonxai_maps_manager
  easynav_common::easynav_common
  easynav_core::easynav_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  Eigen3::Eigen
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
  # Humble's ament_copyright does not recognize the license with the EasyNav line in the header
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(planner3d_tests tests/planner3d_tests.cpp)
  target_link_libraries(planner3d_tests ${PROJECT_NAME})
  ament_add_gtest(bonxai_planner_tests tests/bonxai_planner_tests.cpp)
  target_link_libraries(bonxai_planner_tests ${PROJECT_NAME})
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

pluginlib_export_plugin_description_file(easynav_core easynav_bonxai_planner_plugins.xml)

ament_export_dependencies(
  easynav_bonxai_maps_manager
  easynav_common
  easynav_core
  pluginlib
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
)

ament_package()
