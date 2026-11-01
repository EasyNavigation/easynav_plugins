find_package(easynav_controller REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(mpc_controller_reconfigure_tests mpc_controller_reconfigure_tests.cpp)

target_link_libraries(mpc_controller_reconfigure_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(mpc_controller_robot_limits_tests mpc_controller_robot_limits_tests.cpp)
target_link_libraries(mpc_controller_robot_limits_tests
  ${PROJECT_NAME}
  easynav_controller::easynav_controller
)

ament_add_gtest(mpc_controller_legacy_params_file_tests mpc_controller_legacy_params_file_tests.cpp)
target_link_libraries(mpc_controller_legacy_params_file_tests easynav_controller::easynav_controller)
target_compile_definitions(mpc_controller_legacy_params_file_tests
  PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

ament_add_gtest(mpc_controller_params_tests mpc_controller_params_tests.cpp)
target_link_libraries(mpc_controller_params_tests ${PROJECT_NAME})

ament_add_gtest(mpc_core_tests mpc_core_tests.cpp)
target_link_libraries(mpc_core_tests ${PROJECT_NAME})

ament_add_gtest(mpc_scenarios_tests mpc_scenarios_tests.cpp TIMEOUT 300)
target_link_libraries(mpc_scenarios_tests
  ${PROJECT_NAME}
  easynav_controller::easynav_controller
)

ament_add_gtest(mpc_controller_min_height_tests mpc_controller_min_height_tests.cpp)
target_link_libraries(mpc_controller_min_height_tests ${PROJECT_NAME})
