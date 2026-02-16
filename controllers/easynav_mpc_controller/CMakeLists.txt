cmake_minimum_required(VERSION 3.20)
project(easynav_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_system REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(NLopt QUIET)
if(NOT NLopt_FOUND)
  find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
  find_library(NLOPT_LIBRARY nlopt)
  if(NLOPT_INCLUDE_DIR AND NLOPT_LIBRARY)
    set(NLopt_FOUND TRUE)
    set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
    set(NLopt_LIBRARIES ${NLOPT_LIBRARY})
  endif()
endif()
if(NOT NLopt_FOUND)
  message(FATAL_ERROR "NLopt not found. Install libnlopt-dev or set NLopt_DIR/CMAKE_PREFIX_PATH.")
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
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  Eigen3::Eigen
  nlopt
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
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
  # add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the planning plugins
pluginlib_export_plugin_description_file(easynav_core easynav_mpc_controller_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  pluginlib
  tf2_ros
  geometry_msgs
  nav_msgs
  eigen
  nlopt
)

ament_package()
