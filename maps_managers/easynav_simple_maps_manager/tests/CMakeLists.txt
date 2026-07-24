ament_add_gtest(simple_mapsmanager_tests simple_mapsmanager_tests.cpp)
target_link_libraries(simple_mapsmanager_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
