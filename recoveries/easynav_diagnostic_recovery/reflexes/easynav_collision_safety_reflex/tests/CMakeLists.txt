find_package(easynav_sensors REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_add_gtest(collision_safety_reflex_tests collision_safety_reflex_tests.cpp)
target_link_libraries(collision_safety_reflex_tests
  ${PROJECT_NAME}
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_sensors::easynav_sensors
)
