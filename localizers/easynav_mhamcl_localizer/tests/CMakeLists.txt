find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

ament_add_gtest(mhamcl_algorithm_tests mhamcl_algorithm_tests.cpp)
target_link_libraries(mhamcl_algorithm_tests
  ${PROJECT_NAME}
)

ament_add_gtest(mhamcl_localizer_tests mhamcl_localizer_tests.cpp)
target_link_libraries(mhamcl_localizer_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
)

ament_add_gtest(mhamcl_localizer_reconfigure_tests mhamcl_localizer_reconfigure_tests.cpp)
target_link_libraries(mhamcl_localizer_reconfigure_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

ament_add_gtest(mhamcl_localizer_last_known_pose_tests mhamcl_localizer_last_known_pose_tests.cpp)
target_link_libraries(mhamcl_localizer_last_known_pose_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

ament_add_gtest(mhamcl_localizer_min_height_tests mhamcl_localizer_min_height_tests.cpp)
target_link_libraries(mhamcl_localizer_min_height_tests ${PROJECT_NAME})
