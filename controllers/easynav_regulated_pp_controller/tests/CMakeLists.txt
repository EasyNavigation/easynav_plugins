ament_add_gtest(regulated_pp_controller_tests regulated_pp_controller_tests.cpp)
target_link_libraries(regulated_pp_controller_tests
  ${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(regulated_pp_controller_reconfigure_tests
  regulated_pp_controller_reconfigure_tests.cpp)
target_link_libraries(regulated_pp_controller_reconfigure_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
