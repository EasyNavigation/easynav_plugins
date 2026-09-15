find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(vff_controller_reconfigure_tests vff_controller_reconfigure_tests.cpp)

target_link_libraries(vff_controller_reconfigure_tests
  vff_controller
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
