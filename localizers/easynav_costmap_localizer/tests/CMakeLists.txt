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

ament_add_gtest(costmap_localizer_reconfigure_tests costmap_localizer_reconfigure_tests.cpp)
target_link_libraries(costmap_localizer_reconfigure_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(costmap_localizer_last_known_pose_tests costmap_localizer_last_known_pose_tests.cpp)
target_link_libraries(costmap_localizer_last_known_pose_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

ament_add_gtest(amcl_convergence_evaluator_tests amcl_convergence_evaluator_tests.cpp)
target_link_libraries(amcl_convergence_evaluator_tests
  amcl_convergence_evaluator
)

ament_add_gtest(amcl_relocalize_mitigation_tests amcl_relocalize_mitigation_tests.cpp)
target_link_libraries(amcl_relocalize_mitigation_tests
  amcl_relocalize_mitigation
)

ament_add_gtest(costmap_localizer_min_height_tests costmap_localizer_min_height_tests.cpp)
target_link_libraries(costmap_localizer_min_height_tests ${PROJECT_NAME})
