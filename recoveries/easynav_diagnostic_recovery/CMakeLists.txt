cmake_minimum_required(VERSION 3.20)
project(easynav_diagnostic_recovery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# A diagnosis-driven recovery system (DiagnosticRecoveryManager), the plugin interfaces it is composed
# of (safety reflexes, evaluators, mitigations), the helpers its plugins share and the reference plugins.
add_library(${PROJECT_NAME} SHARED
  src/easynav_diagnostic_recovery/DiagnosticRecoveryManager.cpp
  src/easynav_diagnostic_recovery/ObstacleProximity.cpp
  src/easynav_diagnostic_recovery/RecoveryEvaluatorBase.cpp
  src/easynav_diagnostic_recovery/RecoveryMitigationBase.cpp
  src/easynav_diagnostic_recovery/SafetyReflexBase.cpp
  src/easynav_diagnostic_recovery/evaluators/ControllerStuckEvaluator.cpp
  src/easynav_diagnostic_recovery/evaluators/NoPathEvaluator.cpp
  src/easynav_diagnostic_recovery/evaluators/ObstacleTooCloseEvaluator.cpp
  src/easynav_diagnostic_recovery/evaluators/RosGraphEvaluator.cpp
  src/easynav_diagnostic_recovery/evaluators/SafetyChannelEvaluator.cpp
  src/easynav_diagnostic_recovery/mitigations/AdvanceRecovery.cpp
  src/easynav_diagnostic_recovery/mitigations/CancelMissionRecovery.cpp
  src/easynav_diagnostic_recovery/mitigations/HumanAssistanceRecovery.cpp
  src/easynav_diagnostic_recovery/mitigations/SafeRetreatRecovery.cpp
  src/easynav_diagnostic_recovery/mitigations/ShutdownRecovery.cpp
  src/easynav_diagnostic_recovery/reflexes/CollisionSafetyReflex.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_sensors::easynav_sensors
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${PCL_LIBRARIES}
)

add_library(dummy_evaluator SHARED
  src/easynav_diagnostic_recovery/DummyEvaluator.cpp
)
target_link_libraries(dummy_evaluator PUBLIC ${PROJECT_NAME})

add_library(dummy_mitigation SHARED
  src/easynav_diagnostic_recovery/DummyMitigation.cpp
)
target_link_libraries(dummy_mitigation PUBLIC ${PROJECT_NAME})

add_library(dummy_safety_reflex SHARED
  src/easynav_diagnostic_recovery/DummySafetyReflex.cpp
)
target_link_libraries(dummy_safety_reflex PUBLIC ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  dummy_evaluator
  dummy_mitigation
  dummy_safety_reflex
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
ament_export_libraries(
  ${PROJECT_NAME}
  dummy_evaluator
  dummy_mitigation
  dummy_safety_reflex
)
ament_export_targets(export_${PROJECT_NAME})

# DiagnosticRecoveryManager is a whole recovery system (easynav_core's RecoveryManagerBase)...
pluginlib_export_plugin_description_file(easynav_core easynav_diagnostic_recovery_plugins.xml)
# ...composed of plugins of its own interfaces (reference and test implementations here).
pluginlib_export_plugin_description_file(
  ${PROJECT_NAME} easynav_diagnostic_recovery_components_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  easynav_sensors
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  visualization_msgs
  diagnostic_msgs
  rcl_interfaces
  PCL
)
ament_package()
