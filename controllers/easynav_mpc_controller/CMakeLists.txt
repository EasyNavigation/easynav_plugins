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

# NLopt: system package if available (rosdep, Ubuntu). Otherwise (e.g. RHEL, no rosdep rule)
# a pinned, hash-checked release built as a private static library: MIT-only (no LGPL
# Luksan solvers), nothing of it is installed or exported.
find_package(NLopt QUIET)
if(NLopt_FOUND)
  set(nlopt_target ${NLOPT_LIBRARIES})  # NLopt::nlopt or NLopt::nlopt_cxx
else()
  set(nlopt_version 2.11.0)
  message(STATUS "NLopt not found: building bundled NLopt ${nlopt_version} (static)")
  include(ExternalProject)
  set(nlopt_prefix ${CMAKE_CURRENT_BINARY_DIR}/nlopt)
  set(nlopt_lib
    ${nlopt_prefix}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}nlopt${CMAKE_STATIC_LIBRARY_SUFFIX})
  if(CMAKE_BUILD_TYPE)
    set(nlopt_build_type ${CMAKE_BUILD_TYPE})
  else()
    set(nlopt_build_type Release)
  endif()
  # DOWNLOAD_EXTRACT_TIMESTAMP only exists since CMake 3.24 (Humble has 3.22)
  set(nlopt_download_options)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(nlopt_download_options DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  endif()
  externalproject_add(nlopt_bundled
    URL https://github.com/stevengj/nlopt/archive/refs/tags/v${nlopt_version}.tar.gz
    URL_HASH SHA256=53e552d83e9294d67db37f0f4a23f15933a9ef698485301a18b98b40004cf0de
    ${nlopt_download_options}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${nlopt_prefix}
      -DCMAKE_INSTALL_LIBDIR=lib
      -DCMAKE_BUILD_TYPE=${nlopt_build_type}
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
      # Third-party code: kept out of our LTO link (its warnings are not ours)
      -DCMAKE_C_FLAGS_INIT=-fno-lto
      -DCMAKE_CXX_FLAGS_INIT=-fno-lto
      -DBUILD_SHARED_LIBS=OFF
      -DNLOPT_LUKSAN=OFF
      -DNLOPT_CXX=OFF
      -DNLOPT_FORTRAN=OFF
      -DNLOPT_PYTHON=OFF
      -DNLOPT_OCTAVE=OFF
      -DNLOPT_MATLAB=OFF
      -DNLOPT_GUILE=OFF
      -DNLOPT_JAVA=OFF
      -DNLOPT_SWIG=OFF
      -DNLOPT_TESTS=OFF
    BUILD_BYPRODUCTS ${nlopt_lib}
  )
  file(MAKE_DIRECTORY ${nlopt_prefix}/include)
  add_library(nlopt_static STATIC IMPORTED)
  set_target_properties(nlopt_static PROPERTIES
    IMPORTED_LOCATION ${nlopt_lib}
    INTERFACE_INCLUDE_DIRECTORIES ${nlopt_prefix}/include
  )
  if(UNIX)
    set_property(TARGET nlopt_static PROPERTY INTERFACE_LINK_LIBRARIES m)
  endif()
  if(UNIX AND NOT APPLE)
    # Keep the bundled symbols private: no clash with another NLopt in the process
    set_property(TARGET nlopt_static PROPERTY
      INTERFACE_LINK_OPTIONS "LINKER:--exclude-libs,${CMAKE_STATIC_LIBRARY_PREFIX}nlopt${CMAKE_STATIC_LIBRARY_SUFFIX}")
  endif()
  add_dependencies(nlopt_static nlopt_bundled)
  set(nlopt_target nlopt_static)
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
target_link_libraries(${PROJECT_NAME} PRIVATE ${nlopt_target})

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
)

ament_package()
