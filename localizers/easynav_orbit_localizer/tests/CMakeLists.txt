find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(lie_tests lie_tests.cpp)
target_link_libraries(lie_tests ${PROJECT_NAME}_core)

ament_add_gtest(distance_field_tests distance_field_tests.cpp)
target_link_libraries(distance_field_tests ${PROJECT_NAME}_core)

ament_add_gtest(matcher_tests matcher_tests.cpp)
target_link_libraries(matcher_tests ${PROJECT_NAME}_core)

ament_add_gtest(symmetry_graph_tests symmetry_graph_tests.cpp TIMEOUT 300)
target_link_libraries(symmetry_graph_tests ${PROJECT_NAME}_core)

ament_add_gtest(orbit_localizer_tests orbit_localizer_tests.cpp TIMEOUT 300)
target_link_libraries(orbit_localizer_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
