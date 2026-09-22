find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(serest_controller_reconfigure_tests serest_controller_reconfigure_tests.cpp)

target_link_libraries(serest_controller_reconfigure_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
