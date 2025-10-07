find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_simple_common REQUIRED)
find_package(std_srvs REQUIRED)

ament_add_gtest(simple_controller_tests simple_controller_tests.cpp)
target_link_libraries(simple_controller_tests
  ${PROJECT_NAME}
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_simple_common::easynav_simple_common
  ${std_srvs_TARGETS}
)
