find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

ament_add_gtest(navmap_localizer_tests navmap_localizer_tests.cpp)
target_link_libraries(navmap_localizer_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
)
