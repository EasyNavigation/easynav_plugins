find_package(easynav_controller REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_simple_common REQUIRED)
find_package(std_srvs REQUIRED)

# Outdated (uses an API that no longer exists): disabled until it is rewritten.
# ament_add_gtest(simple_controller_tests simple_controller_tests.cpp)
# target_link_libraries(simple_controller_tests
#   ${PROJECT_NAME}
#   rclcpp_lifecycle::rclcpp_lifecycle
#   easynav_simple_common::easynav_simple_common
#   ${std_srvs_TARGETS}
# )

ament_add_gtest(simple_controller_robot_limits_tests simple_controller_robot_limits_tests.cpp)
target_link_libraries(simple_controller_robot_limits_tests
  ${PROJECT_NAME}
  easynav_controller::easynav_controller
)

ament_add_gtest(simple_controller_legacy_params_file_tests simple_controller_legacy_params_file_tests.cpp)
target_link_libraries(simple_controller_legacy_params_file_tests easynav_controller::easynav_controller)
target_compile_definitions(simple_controller_legacy_params_file_tests
  PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
