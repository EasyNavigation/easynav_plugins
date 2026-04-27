find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_localizer REQUIRED)

ament_add_gtest(fusion_localizer_tests fusion_localizer_tests.cpp)

target_link_libraries(fusion_localizer_tests
  fusion_localizer
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_localizer::easynav_localizer
)
