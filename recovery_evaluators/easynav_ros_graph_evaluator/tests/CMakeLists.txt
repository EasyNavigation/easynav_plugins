find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

ament_add_gtest(ros_graph_evaluator_tests ros_graph_evaluator_tests.cpp)
target_link_libraries(ros_graph_evaluator_tests ${PROJECT_NAME} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})
