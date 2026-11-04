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

# OSQP: 0.6 (Humble, Jazzy, Kilted) or 1.0 (Lyrical, Rolling), both handled in QPSolver.cpp
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)

# Unoptimized, the QP does not fit in a control cycle
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(${PROJECT_NAME} SHARED
  src/easynav_mpc_controller/MPCController.cpp
  src/easynav_mpc_controller/MPCOptimizer.cpp
  src/easynav_mpc_controller/MotionModel.cpp
  src/easynav_mpc_controller/Obstacles.cpp
  src/easynav_mpc_controller/QPSolver.cpp
  src/easynav_mpc_controller/Reference.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
# pcl_conversions exports a CMake target only since 2.8 (Kilted); older ones, variables
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${pcl_conversions_INCLUDE_DIRS})
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
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${pcl_conversions_TARGETS}
  ${pcl_conversions_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)

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
  PCL
  osqp_vendor
)

ament_package()
