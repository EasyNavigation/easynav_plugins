find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(bonxai_maps_manager_reconfigure_tests bonxai_maps_manager_reconfigure_tests.cpp)

target_link_libraries(bonxai_maps_manager_reconfigure_tests
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(bonxai_mask_move_tests bonxai_mask_move_tests.cpp)
target_include_directories(bonxai_mask_move_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
