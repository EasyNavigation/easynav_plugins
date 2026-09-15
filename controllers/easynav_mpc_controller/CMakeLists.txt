cmake_minimum_required(VERSION 3.20)
project(easynav_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_system REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io)

# Try to find NLopt package from system first, or get it through CMake FetchContent to build it
find_package(NLopt QUIET)
if(NOT NLopt_FOUND)
  include(FetchContent)
  # Force FetchContent to download and build in the workspace build dir
  set(FETCH_CONTENT_BASE_DIR "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/_deps>")

  # Disable unneeded NLopt features
  set(NLOPT_PYTHON OFF)
  set(NLOPT_OCTAVE OFF)
  set(NLOPT_GUILE OFF)
  set(NLOPT_JAVA OFF)
  fetchcontent_declare(nlopt
    GIT_REPOSITORY  https://github.com/stevengj/nlopt
    GIT_TAG         v2.11.0
  )
  fetchcontent_makeavailable(
    nlopt
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/easynav_mpc_controller/MPCController.cpp
  src/easynav_mpc_controller/MPCOptimizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_system::easynav_system
  easynav_sensors::easynav_sensors
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  Eigen3::Eigen
  nlopt
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
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
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the planning plugins
pluginlib_export_plugin_description_file(easynav_core easynav_mpc_controller_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  easynav_system
  easynav_sensors
  pluginlib
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  Eigen3
  nlopt
)

ament_package()
