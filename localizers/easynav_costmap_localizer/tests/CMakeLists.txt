find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

ament_add_gtest(costmap_localizer_tests costmap_localizer_tests.cpp)
target_link_libraries(costmap_localizer_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
)

ament_add_gtest(amcl_convergence_evaluator_tests amcl_convergence_evaluator_tests.cpp)
target_link_libraries(amcl_convergence_evaluator_tests
  amcl_convergence_evaluator
)

ament_add_gtest(amcl_relocalize_mitigation_tests amcl_relocalize_mitigation_tests.cpp)
target_link_libraries(amcl_relocalize_mitigation_tests
  amcl_relocalize_mitigation
)
