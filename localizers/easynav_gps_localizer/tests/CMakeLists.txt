find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(gps_localizer_tests gps_localizer_tests.cpp)

target_link_libraries(gps_localizer_tests
  gps_localizer
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
