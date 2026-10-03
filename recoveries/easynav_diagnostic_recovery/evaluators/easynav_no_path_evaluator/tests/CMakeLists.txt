ament_add_gtest(no_path_evaluator_tests no_path_evaluator_tests.cpp)
target_link_libraries(no_path_evaluator_tests ${PROJECT_NAME})

find_package(easynav_planner REQUIRED)
ament_add_gtest(no_path_faulty_planner_tests no_path_faulty_planner_tests.cpp)
target_link_libraries(no_path_faulty_planner_tests
  ${PROJECT_NAME}
  easynav_planner::faulty_planner
)
