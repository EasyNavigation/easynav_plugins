find_package(easynav_recovery REQUIRED)
find_package(lifecycle_msgs REQUIRED)

ament_add_gtest(diagnostic_recovery_bases_tests diagnostic_recovery_bases_tests.cpp)
target_link_libraries(diagnostic_recovery_bases_tests ${PROJECT_NAME})

ament_add_gtest(obstacle_proximity_tests obstacle_proximity_tests.cpp)
target_link_libraries(obstacle_proximity_tests ${PROJECT_NAME})

ament_add_gtest(diagnostic_recovery_manager_tests diagnostic_recovery_manager_tests.cpp)
target_link_libraries(diagnostic_recovery_manager_tests
  ${PROJECT_NAME} easynav_recovery::easynav_recovery ${lifecycle_msgs_TARGETS})

ament_add_gtest(diagnostic_recovery_manager_reflex_tests diagnostic_recovery_manager_reflex_tests.cpp)
target_link_libraries(diagnostic_recovery_manager_reflex_tests
  ${PROJECT_NAME} easynav_recovery::easynav_recovery ${lifecycle_msgs_TARGETS})

# Evaluators
foreach(test
    controller_stuck_evaluator_tests
    no_path_evaluator_tests
    obstacle_too_close_evaluator_tests
    ros_graph_evaluator_tests
    safety_channel_evaluator_tests)
  ament_add_gtest(${test} evaluators/${test}.cpp)
  target_link_libraries(${test} ${PROJECT_NAME})
endforeach()

find_package(easynav_planner REQUIRED)
ament_add_gtest(no_path_faulty_planner_tests evaluators/no_path_faulty_planner_tests.cpp)
target_link_libraries(no_path_faulty_planner_tests ${PROJECT_NAME} easynav_planner::faulty_planner)

# Mitigations
foreach(test
    advance_recovery_tests
    cancel_mission_recovery_tests
    human_assistance_recovery_tests
    safe_retreat_recovery_tests
    shutdown_recovery_tests)
  ament_add_gtest(${test} mitigations/${test}.cpp)
  target_link_libraries(${test} ${PROJECT_NAME})
endforeach()

# Reflexes
ament_add_gtest(collision_safety_reflex_tests reflexes/collision_safety_reflex_tests.cpp)
target_link_libraries(collision_safety_reflex_tests ${PROJECT_NAME})
